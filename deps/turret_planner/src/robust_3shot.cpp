#include "turret_planner/robust_3shot.h"
#include "turret_planner/flight_time.h"
#include <cmath>
#include <algorithm>
#include <queue>
#include <vector>

// ---------------------------------------------------------------------------
// Interpolate a FutureState at time t from a sorted trajectory array.
// ---------------------------------------------------------------------------
static FutureState interp_state(const FutureState* traj, int n, float t) {
    if (n <= 0) return {};
    if (t <= traj[0].t || n == 1) return traj[0];
    if (t >= traj[n-1].t) return traj[n-1];

    int lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (traj[mid].t <= t) lo = mid; else hi = mid;
    }

    float dt = traj[hi].t - traj[lo].t;
    if (dt < 1e-9f) return traj[lo];
    float frac = (t - traj[lo].t) / dt;

    FutureState s;
    s.t       = t;
    s.x       = traj[lo].x  + frac * (traj[hi].x  - traj[lo].x);
    s.y       = traj[lo].y  + frac * (traj[hi].y  - traj[lo].y);
    s.heading = traj[lo].heading + frac * (traj[hi].heading - traj[lo].heading);
    s.vx      = traj[lo].vx + frac * (traj[hi].vx - traj[lo].vx);
    s.vy      = traj[lo].vy + frac * (traj[hi].vy - traj[lo].vy);
    return s;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Check that a shot's launch direction is within 90° of the geometric
// direction from the turret to the target.  When the robot's velocity
// dominates dx/T, atan2 can flip theta ~180° — the ball still reaches the
// target (carried by the robot's momentum) but the turret points away from
// the goal, which is impractical.
static bool is_forward_shot(const ShotParams& p, float dx, float dy) {
    // Positive dot product with target direction => launch is toward the goal.
    return std::cos(p.theta) * dx + std::sin(p.theta) * dy > 0.f;
}

namespace {

float move_cost(const ShotParams& from, const ShotParams& to,
                float drop_frac, const TurretWeights& w, const OmegaMapParams& om) {
    float om_from = omega_map_eval(om, from.phi, from.v_exit) * (1.f - drop_frac);
    float om_to   = omega_map_eval(om, to.phi, to.v_exit);
    float d_omega = std::abs(om_from - om_to);
    float d_phi   = std::abs(from.phi - to.phi);
    float d_theta = std::abs(from.theta - to.theta);
    return std::max({w.w_omega * d_omega, w.w_phi * d_phi, w.w_theta * d_theta});
}

struct Box2D {
    float t1Lo, t1Hi, t2Lo, t2Hi;
    float lb;
};
struct Box2DGreater {
    bool operator()(const Box2D& a, const Box2D& b) const { return a.lb > b.lb; }
};

struct Box3D {
    float t1Lo, t1Hi, t2Lo, t2Hi, t3Lo, t3Hi;
    float lb;
};
struct Box3DGreater {
    bool operator()(const Box3D& a, const Box3D& b) const { return a.lb > b.lb; }
};

} // namespace

// ---------------------------------------------------------------------------
// 1-ball solver: sweep trajectory, pick best tau from current state.
// ---------------------------------------------------------------------------
static Robust3ShotResult solve_1ball(
    const FutureState* traj, int n_states,
    float turret_z,
    float target_x, float target_y, float target_z,
    const TurretState& current,
    float t_min_shot,
    const TurretWeights& weights,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    float tol)
{
    Robust3ShotResult best{};
    best.feasible = false;
    best.J = 1e9f;
    best.J_12 = best.J_23 = 0.f;
    best.idx1 = best.idx2 = best.idx3 = -1;

    float T_warm = -1.f;

    for (int i = 0; i < n_states; ++i) {
        if (traj[i].t < t_min_shot) continue;

        FutureState st = traj[i];

        FlightTimeResult res;
        if (T_warm < 0.f) {
            res = flight_time_cold(
                st.x, st.y, turret_z,
                target_x, target_y, target_z,
                st.vx, st.vy,
                current, weights, bounds, cfg, omega, tol);
        } else {
            res = flight_time_warm(
                st.x, st.y, turret_z,
                target_x, target_y, target_z,
                st.vx, st.vy, T_warm,
                current, weights, bounds, cfg, omega);
        }

        if (!res.feasible) continue;

        // Reject solutions where the turret points away from the goal.
        float dx = target_x - st.x, dy = target_y - st.y;
        if (!is_forward_shot(res.params, dx, dy)) continue;

        // Only warm-start from forward solutions — a backward T biases
        // flight_time_warm toward backward local minima at all subsequent
        // trajectory points.
        T_warm = res.T_star;

        float cost = res.tau;  // J_0 only
        if (cost < best.J) {
            best.J = cost;
            best.feasible = true;
            best.idx1 = i;
            best.T1 = res.T_star;
            best.s1 = res.params;
        }
    }

    return best;
}

// ---------------------------------------------------------------------------
// 2-ball solver: for each t1, 2D B&B over (T1, T2).
// Objective: J_0 + J_12, constraint: J_12 <= transfer_time
// ---------------------------------------------------------------------------
static Robust3ShotResult solve_2ball(
    const FutureState* traj, int n_states,
    float turret_z,
    float target_x, float target_y, float target_z,
    const TurretState& current,
    float t_min_shot, float transfer_time,
    float drop_fraction,
    const TurretWeights& weights,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    float tol, int maxIter)
{
    Robust3ShotResult best{};
    best.feasible = false;
    best.J = 1e9f;
    best.J_12 = best.J_23 = 0.f;
    best.idx1 = best.idx2 = best.idx3 = -1;

    float traj_end = traj[n_states-1].t;

    for (int i = 0; i < n_states; ++i) {
        float t1 = traj[i].t;
        if (t1 < t_min_shot) continue;
        float t2 = t1 + transfer_time;
        if (t2 > traj_end) break;

        FutureState st1 = traj[i];
        FutureState st2 = interp_state(traj, n_states, t2);

        float dx1 = target_x - st1.x, dy1 = target_y - st1.y, dz = target_z - turret_z;
        float dx2 = target_x - st2.x, dy2 = target_y - st2.y;

        TInterval iv1 = ballistics_feasible_interval(dx1, dy1, dz, st1.vx, st1.vy, bounds, cfg, tol*0.25f);
        TInterval iv2 = ballistics_feasible_interval(dx2, dy2, dz, st2.vx, st2.vy, bounds, cfg, tol*0.25f);
        if (iv1.t_lo >= iv1.t_hi || iv2.t_lo >= iv2.t_hi) continue;

        // Lipschitz constants
        LipschitzBounds lip1 = ballistics_lipschitz(dx1, dy1, dz, st1.vx, st1.vy, iv1.t_lo, iv1.t_hi, cfg);
        LipschitzBounds lip2 = ballistics_lipschitz(dx2, dy2, dz, st2.vx, st2.vy, iv2.t_lo, iv2.t_hi, cfg);

        auto lip_omega_fn = [&](const LipschitzBounds& lip, float px, float py,
                                float vx, float vy, float lo, float hi) -> float {
            float tmid = 0.5f*(lo+hi), hw = 0.5f*(hi-lo);
            ShotParams sm = ballistics_solve(px, py, turret_z, target_x, target_y, target_z,
                                              vx, vy, tmid, cfg, omega);
            return omega_map_lipschitz(omega, sm.phi - lip.l_phi*hw, sm.phi + lip.l_phi*hw,
                                       sm.v_exit - lip.l_vexit*hw, sm.v_exit + lip.l_vexit*hw,
                                       lip.l_phi, lip.l_vexit);
        };

        float lom1 = lip_omega_fn(lip1, st1.x, st1.y, st1.vx, st1.vy, iv1.t_lo, iv1.t_hi);
        float lom2 = lip_omega_fn(lip2, st2.x, st2.y, st2.vx, st2.vy, iv2.t_lo, iv2.t_hi);

        // J_0 depends on T1, J_12 depends on T1 and T2
        float L_J0_T1  = std::max({weights.w_theta * lip1.l_theta, weights.w_phi * lip1.l_phi, weights.w_omega * lom1});
        float L_J12_T1 = std::max({weights.w_theta * lip1.l_theta, weights.w_phi * lip1.l_phi, weights.w_omega * (1.f - drop_fraction) * lom1});
        float L1 = L_J0_T1 + L_J12_T1;
        float L2 = std::max({weights.w_theta * lip2.l_theta, weights.w_phi * lip2.l_phi, weights.w_omega * lom2});

        // Objective with constraint
        auto J = [&](float T1, float T2) -> float {
            ShotParams s1 = ballistics_solve(st1.x, st1.y, turret_z, target_x, target_y, target_z,
                                              st1.vx, st1.vy, T1, cfg, omega);
            ShotParams s2 = ballistics_solve(st2.x, st2.y, turret_z, target_x, target_y, target_z,
                                              st2.vx, st2.vy, T2, cfg, omega);
            float j0  = flight_time_tau(s1, current, weights, omega);
            float j12 = move_cost(s1, s2, drop_fraction, weights, omega);
            // Constraint: J_12 must be achievable within transfer_time
            if (j12 > transfer_time) return 1e9f;
            return j0 + j12;
        };

        // 2D B&B
        auto make_rect = [&](float lo1, float hi1, float lo2, float hi2) -> Box2D {
            float tc1 = 0.5f*(lo1+hi1), tc2 = 0.5f*(lo2+hi2);
            float lb = J(tc1, tc2) - L1*(hi1-lo1)*0.5f - L2*(hi2-lo2)*0.5f;
            return {lo1, hi1, lo2, hi2, lb};
        };

        float tM1 = 0.5f*(iv1.t_lo+iv1.t_hi), tM2 = 0.5f*(iv2.t_lo+iv2.t_hi);
        float bestJ_local = J(tM1, tM2);
        float bestT1 = tM1, bestT2 = tM2;
        for (float c1 : {iv1.t_lo, iv1.t_hi}) for (float c2 : {iv2.t_lo, iv2.t_hi}) {
            float j = J(c1, c2);
            if (j < bestJ_local) { bestJ_local = j; bestT1 = c1; bestT2 = c2; }
        }

        std::priority_queue<Box2D, std::vector<Box2D>, Box2DGreater> queue;
        queue.push(make_rect(iv1.t_lo, iv1.t_hi, iv2.t_lo, iv2.t_hi));

        for (int iter = 0; iter < maxIter && !queue.empty(); ++iter) {
            Box2D r = queue.top(); queue.pop();
            if (bestJ_local - r.lb <= tol) break;

            float h1 = L1*(r.t1Hi-r.t1Lo)*0.5f;
            float h2 = L2*(r.t2Hi-r.t2Lo)*0.5f;
            Box2D children[2];
            if (h1 >= h2) {
                float tm = 0.5f*(r.t1Lo+r.t1Hi);
                children[0] = make_rect(r.t1Lo, tm, r.t2Lo, r.t2Hi);
                children[1] = make_rect(tm, r.t1Hi, r.t2Lo, r.t2Hi);
            } else {
                float tm = 0.5f*(r.t2Lo+r.t2Hi);
                children[0] = make_rect(r.t1Lo, r.t1Hi, r.t2Lo, tm);
                children[1] = make_rect(r.t1Lo, r.t1Hi, tm, r.t2Hi);
            }
            for (auto& c : children) {
                float tc1 = 0.5f*(c.t1Lo+c.t1Hi), tc2 = 0.5f*(c.t2Lo+c.t2Hi);
                float j = J(tc1, tc2);
                if (j < bestJ_local) { bestJ_local = j; bestT1 = tc1; bestT2 = tc2; }
                if (c.lb < bestJ_local) queue.push(c);
            }
        }

        if (bestJ_local < best.J) {
            ShotParams s1f = ballistics_solve(st1.x, st1.y, turret_z, target_x, target_y, target_z,
                                               st1.vx, st1.vy, bestT1, cfg, omega);
            ShotParams s2f = ballistics_solve(st2.x, st2.y, turret_z, target_x, target_y, target_z,
                                               st2.vx, st2.vy, bestT2, cfg, omega);
            // Reject if either shot points away from the goal.
            if (!is_forward_shot(s1f, dx1, dy1) || !is_forward_shot(s2f, dx2, dy2))
                continue;
            float j12 = move_cost(s1f, s2f, drop_fraction, weights, omega);
            if (j12 <= transfer_time) {
                best.J = bestJ_local;
                best.J_12 = j12;
                best.feasible = true;
                best.idx1 = i;
                best.idx2 = -1;
                best.T1 = bestT1; best.T2 = bestT2;
                best.s1 = s1f; best.s2 = s2f;
            }
        }
    }

    return best;
}

// ---------------------------------------------------------------------------
// 3-ball solver: for each t1, 3D B&B over (T1, T2, T3).
// Objective: J_0 + J_12 + J_23, constraints: J_12 <= tt, J_23 <= tt
// ---------------------------------------------------------------------------
static Robust3ShotResult solve_3ball(
    const FutureState* traj, int n_states,
    float turret_z,
    float target_x, float target_y, float target_z,
    const TurretState& current,
    float t_min_shot, float transfer_time,
    float drop_fraction,
    const TurretWeights& weights,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    float tol, int maxIter)
{
    Robust3ShotResult best{};
    best.feasible = false;
    best.J = 1e9f;
    best.J_12 = best.J_23 = 0.f;
    best.idx1 = best.idx2 = best.idx3 = -1;

    float traj_end = traj[n_states-1].t;

    for (int i = 0; i < n_states; ++i) {
        float t1 = traj[i].t;
        if (t1 < t_min_shot) continue;
        float t2 = t1 + transfer_time;
        float t3 = t1 + 2.f * transfer_time;
        if (t3 > traj_end) break;

        FutureState st1 = traj[i];
        FutureState st2 = interp_state(traj, n_states, t2);
        FutureState st3 = interp_state(traj, n_states, t3);

        float dx1 = target_x - st1.x, dy1 = target_y - st1.y, dz = target_z - turret_z;
        float dx2 = target_x - st2.x, dy2 = target_y - st2.y;
        float dx3 = target_x - st3.x, dy3 = target_y - st3.y;

        TInterval iv1 = ballistics_feasible_interval(dx1, dy1, dz, st1.vx, st1.vy, bounds, cfg, tol*0.25f);
        TInterval iv2 = ballistics_feasible_interval(dx2, dy2, dz, st2.vx, st2.vy, bounds, cfg, tol*0.25f);
        TInterval iv3 = ballistics_feasible_interval(dx3, dy3, dz, st3.vx, st3.vy, bounds, cfg, tol*0.25f);
        if (iv1.t_lo >= iv1.t_hi || iv2.t_lo >= iv2.t_hi || iv3.t_lo >= iv3.t_hi) continue;

        LipschitzBounds lip1 = ballistics_lipschitz(dx1, dy1, dz, st1.vx, st1.vy, iv1.t_lo, iv1.t_hi, cfg);
        LipschitzBounds lip2 = ballistics_lipschitz(dx2, dy2, dz, st2.vx, st2.vy, iv2.t_lo, iv2.t_hi, cfg);
        LipschitzBounds lip3 = ballistics_lipschitz(dx3, dy3, dz, st3.vx, st3.vy, iv3.t_lo, iv3.t_hi, cfg);

        auto lip_omega_fn = [&](const LipschitzBounds& lip, float px, float py,
                                float vx, float vy, float lo, float hi) -> float {
            float tmid = 0.5f*(lo+hi), hw = 0.5f*(hi-lo);
            ShotParams sm = ballistics_solve(px, py, turret_z, target_x, target_y, target_z,
                                              vx, vy, tmid, cfg, omega);
            return omega_map_lipschitz(omega, sm.phi - lip.l_phi*hw, sm.phi + lip.l_phi*hw,
                                       sm.v_exit - lip.l_vexit*hw, sm.v_exit + lip.l_vexit*hw,
                                       lip.l_phi, lip.l_vexit);
        };

        float lom1 = lip_omega_fn(lip1, st1.x, st1.y, st1.vx, st1.vy, iv1.t_lo, iv1.t_hi);
        float lom2 = lip_omega_fn(lip2, st2.x, st2.y, st2.vx, st2.vy, iv2.t_lo, iv2.t_hi);
        float lom3 = lip_omega_fn(lip3, st3.x, st3.y, st3.vx, st3.vy, iv3.t_lo, iv3.t_hi);

        // Separable Lipschitz: J = J_0(T1) + J_12(T1,T2) + J_23(T2,T3)
        float L_J0_T1  = std::max({weights.w_theta * lip1.l_theta, weights.w_phi * lip1.l_phi, weights.w_omega * lom1});
        float L_J12_T1 = std::max({weights.w_theta * lip1.l_theta, weights.w_phi * lip1.l_phi, weights.w_omega * (1.f - drop_fraction) * lom1});
        float L1 = L_J0_T1 + L_J12_T1;

        float L_J12_T2 = std::max({weights.w_theta * lip2.l_theta, weights.w_phi * lip2.l_phi, weights.w_omega * lom2});
        float L_J23_T2 = std::max({weights.w_theta * lip2.l_theta, weights.w_phi * lip2.l_phi, weights.w_omega * (1.f - drop_fraction) * lom2});
        float L2 = L_J12_T2 + L_J23_T2;

        float L3 = std::max({weights.w_theta * lip3.l_theta, weights.w_phi * lip3.l_phi, weights.w_omega * lom3});

        // Objective with constraints
        auto J = [&](float T1, float T2, float T3) -> float {
            ShotParams s1 = ballistics_solve(st1.x, st1.y, turret_z, target_x, target_y, target_z,
                                              st1.vx, st1.vy, T1, cfg, omega);
            ShotParams s2 = ballistics_solve(st2.x, st2.y, turret_z, target_x, target_y, target_z,
                                              st2.vx, st2.vy, T2, cfg, omega);
            ShotParams s3 = ballistics_solve(st3.x, st3.y, turret_z, target_x, target_y, target_z,
                                              st3.vx, st3.vy, T3, cfg, omega);
            float j0  = flight_time_tau(s1, current, weights, omega);
            float j12 = move_cost(s1, s2, drop_fraction, weights, omega);
            float j23 = move_cost(s2, s3, drop_fraction, weights, omega);
            // Hard constraints: transitions must fit within transfer_time
            if (j12 > transfer_time || j23 > transfer_time) return 1e9f;
            return j0 + j12 + j23;
        };

        // Seed from 27 points
        float seeds1[3] = {iv1.t_lo, 0.5f*(iv1.t_lo+iv1.t_hi), iv1.t_hi};
        float seeds2[3] = {iv2.t_lo, 0.5f*(iv2.t_lo+iv2.t_hi), iv2.t_hi};
        float seeds3[3] = {iv3.t_lo, 0.5f*(iv3.t_lo+iv3.t_hi), iv3.t_hi};

        float bestJ_local = 1e9f;
        float bestT1 = seeds1[1], bestT2 = seeds2[1], bestT3 = seeds3[1];
        for (float c1 : seeds1) for (float c2 : seeds2) for (float c3 : seeds3) {
            float j = J(c1, c2, c3);
            if (j < bestJ_local) { bestJ_local = j; bestT1 = c1; bestT2 = c2; bestT3 = c3; }
        }

        // 3D B&B
        auto make_box = [&](float lo1, float hi1, float lo2, float hi2, float lo3, float hi3) -> Box3D {
            float tc1 = 0.5f*(lo1+hi1), tc2 = 0.5f*(lo2+hi2), tc3 = 0.5f*(lo3+hi3);
            float lb = J(tc1,tc2,tc3) - L1*(hi1-lo1)*0.5f - L2*(hi2-lo2)*0.5f - L3*(hi3-lo3)*0.5f;
            return {lo1, hi1, lo2, hi2, lo3, hi3, lb};
        };

        std::priority_queue<Box3D, std::vector<Box3D>, Box3DGreater> queue;
        queue.push(make_box(iv1.t_lo, iv1.t_hi, iv2.t_lo, iv2.t_hi, iv3.t_lo, iv3.t_hi));

        for (int iter = 0; iter < maxIter && !queue.empty(); ++iter) {
            Box3D r = queue.top(); queue.pop();
            if (bestJ_local - r.lb <= tol) break;

            float h1 = L1 * (r.t1Hi - r.t1Lo) * 0.5f;
            float h2 = L2 * (r.t2Hi - r.t2Lo) * 0.5f;
            float h3 = L3 * (r.t3Hi - r.t3Lo) * 0.5f;

            Box3D children[2];
            if (h1 >= h2 && h1 >= h3) {
                float tm = 0.5f*(r.t1Lo + r.t1Hi);
                children[0] = make_box(r.t1Lo, tm,     r.t2Lo, r.t2Hi, r.t3Lo, r.t3Hi);
                children[1] = make_box(tm,     r.t1Hi, r.t2Lo, r.t2Hi, r.t3Lo, r.t3Hi);
            } else if (h2 >= h1 && h2 >= h3) {
                float tm = 0.5f*(r.t2Lo + r.t2Hi);
                children[0] = make_box(r.t1Lo, r.t1Hi, r.t2Lo, tm,     r.t3Lo, r.t3Hi);
                children[1] = make_box(r.t1Lo, r.t1Hi, tm,     r.t2Hi, r.t3Lo, r.t3Hi);
            } else {
                float tm = 0.5f*(r.t3Lo + r.t3Hi);
                children[0] = make_box(r.t1Lo, r.t1Hi, r.t2Lo, r.t2Hi, r.t3Lo, tm);
                children[1] = make_box(r.t1Lo, r.t1Hi, r.t2Lo, r.t2Hi, tm,     r.t3Hi);
            }

            for (auto& c : children) {
                float tc1 = 0.5f*(c.t1Lo+c.t1Hi), tc2 = 0.5f*(c.t2Lo+c.t2Hi), tc3 = 0.5f*(c.t3Lo+c.t3Hi);
                float j = J(tc1, tc2, tc3);
                if (j < bestJ_local) { bestJ_local = j; bestT1 = tc1; bestT2 = tc2; bestT3 = tc3; }
                if (c.lb < bestJ_local) queue.push(c);
            }
        }

        if (bestJ_local < best.J) {
            ShotParams s1f = ballistics_solve(st1.x, st1.y, turret_z, target_x, target_y, target_z,
                                               st1.vx, st1.vy, bestT1, cfg, omega);
            ShotParams s2f = ballistics_solve(st2.x, st2.y, turret_z, target_x, target_y, target_z,
                                               st2.vx, st2.vy, bestT2, cfg, omega);
            ShotParams s3f = ballistics_solve(st3.x, st3.y, turret_z, target_x, target_y, target_z,
                                               st3.vx, st3.vy, bestT3, cfg, omega);
            // Reject if any shot points away from the goal.
            if (!is_forward_shot(s1f, dx1, dy1) ||
                !is_forward_shot(s2f, dx2, dy2) ||
                !is_forward_shot(s3f, dx3, dy3))
                continue;
            float j12 = move_cost(s1f, s2f, drop_fraction, weights, omega);
            float j23 = move_cost(s2f, s3f, drop_fraction, weights, omega);
            if (j12 <= transfer_time && j23 <= transfer_time) {
                best.J = bestJ_local;
                best.J_12 = j12; best.J_23 = j23;
                best.feasible = true;
                best.idx1 = i; best.idx2 = -1; best.idx3 = -1;
                best.T1 = bestT1; best.T2 = bestT2; best.T3 = bestT3;
                best.s1 = s1f; best.s2 = s2f; best.s3 = s3f;
            }
        }
    }

    return best;
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------
Robust3ShotResult robust_3shot_plan(
    const FutureState* trajectory, int n_states,
    float turret_z,
    float target_x, float target_y, float target_z,
    const TurretState& current,
    int   n_balls,
    float t_remaining,
    float transfer_time,
    float drop_fraction,
    const TurretWeights& weights,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    float tol,
    int   maxIter)
{
    if (n_states <= 0 || n_balls <= 0) {
        Robust3ShotResult r{};
        r.feasible = false;
        r.J = 1e9f;
        r.idx1 = r.idx2 = r.idx3 = -1;
        return r;
    }

    float t_min_shot = t_remaining;
    n_balls = std::clamp(n_balls, 1, 3);

    if (n_balls == 1) {
        return solve_1ball(trajectory, n_states, turret_z,
                           target_x, target_y, target_z,
                           current, t_min_shot,
                           weights, bounds, cfg, omega, tol);
    } else if (n_balls == 2) {
        return solve_2ball(trajectory, n_states, turret_z,
                           target_x, target_y, target_z,
                           current, t_min_shot, transfer_time,
                           drop_fraction,
                           weights, bounds, cfg, omega, tol, maxIter);
    } else {
        return solve_3ball(trajectory, n_states, turret_z,
                           target_x, target_y, target_z,
                           current, t_min_shot, transfer_time,
                           drop_fraction,
                           weights, bounds, cfg, omega, tol, maxIter);
    }
}
