#include "turret_planner/robust_shot.h"
#include "turret_planner/flight_time.h"
#include <cmath>
#include <algorithm>
#include <queue>
#include <vector>

// ---------------------------------------------------------------------------
// 2D Lipschitz branch-and-bound port of Kotlin ShotSolver.optimalRobustShot.
//
// The objective is the move cost from (s1 with flywheel scaled by 1-drop_fraction)
// to s2. Constants L1, L2 are computed once over the full feasible rectangle;
// subtracting a constant from one omega doesn't change the Lipschitz constant
// of the absolute difference.
// ---------------------------------------------------------------------------

namespace {

struct Rect {
    float t1Lo, t1Hi;
    float t2Lo, t2Hi;
    float lb;
};

struct RectGreater {
    bool operator()(const Rect& a, const Rect& b) const { return a.lb > b.lb; }
};

} // namespace

RobustShotResult flight_time_robust(
    float turret_x, float turret_y, float turret_z,
    float target1_x, float target1_y, float target1_z,
    float target2_x, float target2_y, float target2_z,
    float robot_vx, float robot_vy,
    float drop_fraction,
    const TurretWeights& weights,
    const TurretBounds&  bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    float tol,
    int   maxIter)
{
    float dx1 = target1_x - turret_x;
    float dy1 = target1_y - turret_y;
    float dz1 = target1_z - turret_z;
    float dx2 = target2_x - turret_x;
    float dy2 = target2_y - turret_y;
    float dz2 = target2_z - turret_z;

    TInterval iv1 = ballistics_feasible_interval(dx1, dy1, dz1, robot_vx, robot_vy,
                                                 bounds, cfg, tol * 0.25f);
    TInterval iv2 = ballistics_feasible_interval(dx2, dy2, dz2, robot_vx, robot_vy,
                                                 bounds, cfg, tol * 0.25f);

    if (iv1.t_lo >= iv1.t_hi || iv2.t_lo >= iv2.t_hi) {
        RobustShotResult r{};
        r.feasible = false;
        r.J = 1e9f;
        return r;
    }

    // --- Lipschitz constants on the full rectangle ---
    auto compute_L_raw = [&](float dx, float dy, float dz,
                             float tx, float ty, float tz,
                             float t_lo, float t_hi) -> std::pair<float, float> {
        LipschitzBounds lip = ballistics_lipschitz(dx, dy, dz, robot_vx, robot_vy,
                                                   t_lo, t_hi, cfg);
        float t_mid = 0.5f * (t_lo + t_hi);
        float hw    = 0.5f * (t_hi - t_lo);
        ShotParams s_mid = ballistics_solve(turret_x, turret_y, turret_z,
                                            tx, ty, tz,
                                            robot_vx, robot_vy, t_mid, cfg, omega);
        float phi_lo = s_mid.phi    - lip.l_phi   * hw;
        float phi_hi = s_mid.phi    + lip.l_phi   * hw;
        float v_lo   = s_mid.v_exit - lip.l_vexit * hw;
        float v_hi   = s_mid.v_exit + lip.l_vexit * hw;
        float l_om = omega_map_lipschitz(omega, phi_lo, phi_hi, v_lo, v_hi,
                                         lip.l_phi, lip.l_vexit);
        float l_geom = std::max(weights.w_theta * lip.l_theta,
                                weights.w_phi   * lip.l_phi);
        return {l_geom, l_om};
    };

    // L1: omega1 is scaled by (1-drop_fraction), so its Lipschitz constant scales too
    auto [l1_geom, l1_om] = compute_L_raw(dx1, dy1, dz1, target1_x, target1_y, target1_z, iv1.t_lo, iv1.t_hi);
    float L1 = std::max(l1_geom, weights.w_omega * (1.f - drop_fraction) * l1_om);
    // L2: omega2 is unscaled
    auto [l2_geom, l2_om] = compute_L_raw(dx2, dy2, dz2, target2_x, target2_y, target2_z, iv2.t_lo, iv2.t_hi);
    float L2 = std::max(l2_geom, weights.w_omega * l2_om);

    // --- Objective ---
    auto J = [&](float t1, float t2, ShotParams* out_s1 = nullptr,
                                      ShotParams* out_s2 = nullptr) -> float {
        ShotParams s1 = ballistics_solve(turret_x, turret_y, turret_z,
                                         target1_x, target1_y, target1_z,
                                         robot_vx, robot_vy, t1, cfg, omega);
        ShotParams s2 = ballistics_solve(turret_x, turret_y, turret_z,
                                         target2_x, target2_y, target2_z,
                                         robot_vx, robot_vy, t2, cfg, omega);
        float om1 = omega_map_eval(omega, s1.phi, s1.v_exit) * (1.f - drop_fraction);
        float om2 = omega_map_eval(omega, s2.phi, s2.v_exit);
        float d_omega = std::abs(om1 - om2);
        float d_phi   = std::abs(s1.phi   - s2.phi);
        float d_theta = std::abs(s1.theta - s2.theta);
        if (out_s1) *out_s1 = s1;
        if (out_s2) *out_s2 = s2;
        return std::max({weights.w_omega * d_omega,
                         weights.w_phi   * d_phi,
                         weights.w_theta * d_theta});
    };

    // --- Rectangle with Lipschitz lower bound on J inside it ---
    auto make_rect = [&](float t1Lo, float t1Hi, float t2Lo, float t2Hi) -> Rect {
        float tc1 = 0.5f * (t1Lo + t1Hi);
        float tc2 = 0.5f * (t2Lo + t2Hi);
        float lb = J(tc1, tc2) - L1 * (t1Hi - t1Lo) * 0.5f - L2 * (t2Hi - t2Lo) * 0.5f;
        return Rect{t1Lo, t1Hi, t2Lo, t2Hi, lb};
    };

    // --- Seed best upper bound from center + 4 corners ---
    float tMid1 = 0.5f * (iv1.t_lo + iv1.t_hi);
    float tMid2 = 0.5f * (iv2.t_lo + iv2.t_hi);

    float bestT1 = tMid1;
    float bestT2 = tMid2;
    float bestJ  = J(tMid1, tMid2);

    const float corners1[2] = {iv1.t_lo, iv1.t_hi};
    const float corners2[2] = {iv2.t_lo, iv2.t_hi};
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            float t1 = corners1[i];
            float t2 = corners2[j];
            float j_val = J(t1, t2);
            if (j_val < bestJ) { bestJ = j_val; bestT1 = t1; bestT2 = t2; }
        }
    }

    // --- Branch-and-bound loop ---
    std::priority_queue<Rect, std::vector<Rect>, RectGreater> queue;
    queue.push(make_rect(iv1.t_lo, iv1.t_hi, iv2.t_lo, iv2.t_hi));

    const float min_half = tol * 1e-3f;

    for (int iter = 0; iter < maxIter; ++iter) {
        if (queue.empty()) break;
        Rect r = queue.top();
        queue.pop();
        if (bestJ - r.lb <= tol) break;

        float half1 = L1 * (r.t1Hi - r.t1Lo) * 0.5f;
        float half2 = L2 * (r.t2Hi - r.t2Lo) * 0.5f;
        if (half1 <= min_half && half2 <= min_half) break;

        Rect children[2];
        if (half1 >= half2) {
            float tm = 0.5f * (r.t1Lo + r.t1Hi);
            children[0] = make_rect(r.t1Lo, tm,     r.t2Lo, r.t2Hi);
            children[1] = make_rect(tm,     r.t1Hi, r.t2Lo, r.t2Hi);
        } else {
            float tm = 0.5f * (r.t2Lo + r.t2Hi);
            children[0] = make_rect(r.t1Lo, r.t1Hi, r.t2Lo, tm);
            children[1] = make_rect(r.t1Lo, r.t1Hi, tm,     r.t2Hi);
        }

        for (int c = 0; c < 2; ++c) {
            float tc1 = 0.5f * (children[c].t1Lo + children[c].t1Hi);
            float tc2 = 0.5f * (children[c].t2Lo + children[c].t2Hi);
            float j_val = J(tc1, tc2);
            if (j_val < bestJ) { bestJ = j_val; bestT1 = tc1; bestT2 = tc2; }
            if (children[c].lb < bestJ) queue.push(children[c]);
        }
    }

    // --- Final shots ---
    RobustShotResult out;
    out.T1 = bestT1;
    out.T2 = bestT2;
    out.s1 = ballistics_solve(turret_x, turret_y, turret_z,
                              target1_x, target1_y, target1_z,
                              robot_vx, robot_vy, bestT1, cfg, omega);
    out.s2 = ballistics_solve(turret_x, turret_y, turret_z,
                              target2_x, target2_y, target2_z,
                              robot_vx, robot_vy, bestT2, cfg, omega);
    out.J = bestJ;
    out.feasible = ballistics_is_feasible(out.s1, bestT1, bounds, cfg, omega)
                && ballistics_is_feasible(out.s2, bestT2, bounds, cfg, omega);
    return out;
}

// ---------------------------------------------------------------------------
// flight_time_robust_adjust
//
// Minimizes total time to fire two balls from the current turret state:
//     J(T1, T2) = J_Δ(cur → s1(T1)) + J_Δ(s1(T1)_reduced → s2(T2))
//
// Lipschitz analysis:
//   - Term A = J_Δ(cur, s1) depends only on T1.
//   - Term B = J_Δ(s1_reduced, s2) depends on both T1 and T2; the
//     proportional drop (1-drop_fraction) scales the omega Lipschitz
//     constant in B's T1 arm.
//   So L1 = L_A + L_B_T1 (A and B both contribute in T1),
//      L2 = L_B_T2        (only B contributes in T2).
// ---------------------------------------------------------------------------
RobustShotResult flight_time_robust_adjust(
    float turret_x, float turret_y, float turret_z,
    float target1_x, float target1_y, float target1_z,
    float target2_x, float target2_y, float target2_z,
    float robot_vx, float robot_vy,
    const TurretState& current,
    float drop_fraction,
    const TurretWeights& weights,
    const TurretBounds&  bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    float tol,
    int   maxIter)
{
    float dx1 = target1_x - turret_x;
    float dy1 = target1_y - turret_y;
    float dz1 = target1_z - turret_z;
    float dx2 = target2_x - turret_x;
    float dy2 = target2_y - turret_y;
    float dz2 = target2_z - turret_z;

    TInterval iv1 = ballistics_feasible_interval(dx1, dy1, dz1, robot_vx, robot_vy,
                                                 bounds, cfg, tol * 0.25f);
    TInterval iv2 = ballistics_feasible_interval(dx2, dy2, dz2, robot_vx, robot_vy,
                                                 bounds, cfg, tol * 0.25f);

    if (iv1.t_lo >= iv1.t_hi || iv2.t_lo >= iv2.t_hi) {
        RobustShotResult r{};
        r.feasible = false;
        r.J = 1e9f;
        return r;
    }

    // --- Per-target Lipschitz constants ---
    auto compute_L_raw = [&](float dx, float dy, float dz,
                             float tx, float ty, float tz,
                             float t_lo, float t_hi) -> std::pair<float, float> {
        LipschitzBounds lip = ballistics_lipschitz(dx, dy, dz, robot_vx, robot_vy,
                                                   t_lo, t_hi, cfg);
        float t_mid = 0.5f * (t_lo + t_hi);
        float hw    = 0.5f * (t_hi - t_lo);
        ShotParams s_mid = ballistics_solve(turret_x, turret_y, turret_z,
                                            tx, ty, tz,
                                            robot_vx, robot_vy, t_mid, cfg, omega);
        float phi_lo = s_mid.phi    - lip.l_phi   * hw;
        float phi_hi = s_mid.phi    + lip.l_phi   * hw;
        float v_lo   = s_mid.v_exit - lip.l_vexit * hw;
        float v_hi   = s_mid.v_exit + lip.l_vexit * hw;
        float l_om = omega_map_lipschitz(omega, phi_lo, phi_hi, v_lo, v_hi,
                                         lip.l_phi, lip.l_vexit);
        float l_geom = std::max(weights.w_theta * lip.l_theta,
                                weights.w_phi   * lip.l_phi);
        return {l_geom, l_om};
    };

    auto [l1_geom, l1_om] = compute_L_raw(dx1, dy1, dz1, target1_x, target1_y, target1_z, iv1.t_lo, iv1.t_hi);
    auto [l2_geom, l2_om] = compute_L_raw(dx2, dy2, dz2, target2_x, target2_y, target2_z, iv2.t_lo, iv2.t_hi);

    // Term A = J_Δ(cur, s1): L_A in T1 uses full omega Lipschitz (unscaled)
    float L_A = std::max(l1_geom, weights.w_omega * l1_om);
    // Term B = J_Δ(s1_reduced, s2): s1's omega is scaled by (1-drop_fraction)
    float L_B_T1 = std::max(l1_geom, weights.w_omega * (1.f - drop_fraction) * l1_om);
    float L_B_T2 = std::max(l2_geom, weights.w_omega * l2_om);

    float L1 = L_A + L_B_T1;  // A and B both contribute in T1
    float L2 = L_B_T2;        // only B contributes in T2

    // --- Objective: A + B ---
    auto J = [&](float t1, float t2) -> float {
        ShotParams s1 = ballistics_solve(turret_x, turret_y, turret_z,
                                         target1_x, target1_y, target1_z,
                                         robot_vx, robot_vy, t1, cfg, omega);
        ShotParams s2 = ballistics_solve(turret_x, turret_y, turret_z,
                                         target2_x, target2_y, target2_z,
                                         robot_vx, robot_vy, t2, cfg, omega);

        // A = J_Δ(cur → s1), wraps Δθ to [-π, π].
        float A = flight_time_tau(s1, current, weights, omega);

        // B = J_Δ(s1_reduced → s2), no Δθ wrap (same-goal consecutive shots).
        float b_om1 = s1.omega_flywheel * (1.f - drop_fraction);
        float b_om2 = s2.omega_flywheel;
        float b_d_omega = std::abs(b_om1 - b_om2);
        float b_d_phi   = std::abs(s1.phi   - s2.phi);
        float b_d_theta = std::abs(s1.theta - s2.theta);
        float B = std::max({weights.w_omega * b_d_omega,
                            weights.w_phi   * b_d_phi,
                            weights.w_theta * b_d_theta});

        return A + B;
    };

    auto make_rect = [&](float t1Lo, float t1Hi, float t2Lo, float t2Hi) -> Rect {
        float tc1 = 0.5f * (t1Lo + t1Hi);
        float tc2 = 0.5f * (t2Lo + t2Hi);
        float lb = J(tc1, tc2) - L1 * (t1Hi - t1Lo) * 0.5f - L2 * (t2Hi - t2Lo) * 0.5f;
        return Rect{t1Lo, t1Hi, t2Lo, t2Hi, lb};
    };

    // --- Seed best upper bound from center + 4 corners ---
    float tMid1 = 0.5f * (iv1.t_lo + iv1.t_hi);
    float tMid2 = 0.5f * (iv2.t_lo + iv2.t_hi);

    float bestT1 = tMid1;
    float bestT2 = tMid2;
    float bestJ  = J(tMid1, tMid2);

    const float corners1[2] = {iv1.t_lo, iv1.t_hi};
    const float corners2[2] = {iv2.t_lo, iv2.t_hi};
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            float t1 = corners1[i];
            float t2 = corners2[j];
            float j_val = J(t1, t2);
            if (j_val < bestJ) { bestJ = j_val; bestT1 = t1; bestT2 = t2; }
        }
    }

    std::priority_queue<Rect, std::vector<Rect>, RectGreater> queue;
    queue.push(make_rect(iv1.t_lo, iv1.t_hi, iv2.t_lo, iv2.t_hi));

    const float min_half = tol * 1e-3f;

    for (int iter = 0; iter < maxIter; ++iter) {
        if (queue.empty()) break;
        Rect r = queue.top();
        queue.pop();
        if (bestJ - r.lb <= tol) break;

        float half1 = L1 * (r.t1Hi - r.t1Lo) * 0.5f;
        float half2 = L2 * (r.t2Hi - r.t2Lo) * 0.5f;
        if (half1 <= min_half && half2 <= min_half) break;

        Rect children[2];
        if (half1 >= half2) {
            float tm = 0.5f * (r.t1Lo + r.t1Hi);
            children[0] = make_rect(r.t1Lo, tm,     r.t2Lo, r.t2Hi);
            children[1] = make_rect(tm,     r.t1Hi, r.t2Lo, r.t2Hi);
        } else {
            float tm = 0.5f * (r.t2Lo + r.t2Hi);
            children[0] = make_rect(r.t1Lo, r.t1Hi, r.t2Lo, tm);
            children[1] = make_rect(r.t1Lo, r.t1Hi, tm,     r.t2Hi);
        }

        for (int c = 0; c < 2; ++c) {
            float tc1 = 0.5f * (children[c].t1Lo + children[c].t1Hi);
            float tc2 = 0.5f * (children[c].t2Lo + children[c].t2Hi);
            float j_val = J(tc1, tc2);
            if (j_val < bestJ) { bestJ = j_val; bestT1 = tc1; bestT2 = tc2; }
            if (children[c].lb < bestJ) queue.push(children[c]);
        }
    }

    // --- Height-feasibility post-processing ---
    // The B&B minimizes A + B but doesn't enforce that shot 2 is achievable with the
    // post-drop omega from shot 1. If s2.omega > s1.omega*(1-drop), the ball won't
    // reach target height. Scan T2 over the feasible interval to find the best T2
    // that satisfies the constraint, keeping T1 fixed.
    {
        ShotParams s1_bbest = ballistics_solve(turret_x, turret_y, turret_z,
                                               target1_x, target1_y, target1_z,
                                               robot_vx, robot_vy, bestT1, cfg, omega);
        float om_after_drop = s1_bbest.omega_flywheel * (1.f - drop_fraction);
        ShotParams s2_bbest = ballistics_solve(turret_x, turret_y, turret_z,
                                               target2_x, target2_y, target2_z,
                                               robot_vx, robot_vy, bestT2, cfg, omega);

        if (s2_bbest.omega_flywheel > om_after_drop) {
            // B&B result violates height constraint. Scan T2 for the best feasible T2.
            float best_adj_T2 = bestT2;
            float best_adj_B  = 1e30f;
            const int N_SCAN = 64;
            for (int i = 0; i <= N_SCAN; ++i) {
                float t2c = iv2.t_lo + (iv2.t_hi - iv2.t_lo) * float(i) / float(N_SCAN);
                ShotParams s2c = ballistics_solve(turret_x, turret_y, turret_z,
                                                  target2_x, target2_y, target2_z,
                                                  robot_vx, robot_vy, t2c, cfg, omega);
                if (!ballistics_is_feasible(s2c, t2c, bounds, cfg, omega)) continue;
                if (s2c.omega_flywheel > om_after_drop) continue;
                float b_d_omega = om_after_drop - s2c.omega_flywheel;
                float b_d_phi   = std::abs(s1_bbest.phi   - s2c.phi);
                float b_d_theta = std::abs(s1_bbest.theta - s2c.theta);
                float B = std::max({weights.w_omega * b_d_omega,
                                    weights.w_phi   * b_d_phi,
                                    weights.w_theta * b_d_theta});
                if (B < best_adj_B) { best_adj_B = B; best_adj_T2 = t2c; }
            }
            bestT2 = best_adj_T2;
            bestJ  = J(bestT1, bestT2);
        }
    }

    RobustShotResult out;
    out.T1 = bestT1;
    out.T2 = bestT2;
    out.s1 = ballistics_solve(turret_x, turret_y, turret_z,
                              target1_x, target1_y, target1_z,
                              robot_vx, robot_vy, bestT1, cfg, omega);
    out.s2 = ballistics_solve(turret_x, turret_y, turret_z,
                              target2_x, target2_y, target2_z,
                              robot_vx, robot_vy, bestT2, cfg, omega);
    out.J = bestJ;
    // Feasible only if both shots pass ballistics constraints AND shot 2's required
    // omega does not exceed the post-drop omega available from shot 1.
    bool height_ok = out.s2.omega_flywheel <= out.s1.omega_flywheel * (1.f - drop_fraction);
    out.feasible = height_ok
                && ballistics_is_feasible(out.s1, bestT1, bounds, cfg, omega)
                && ballistics_is_feasible(out.s2, bestT2, bounds, cfg, omega);
    return out;
}
