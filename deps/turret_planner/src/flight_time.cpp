#include "turret_planner/flight_time.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <utility>

// ---------------------------------------------------------------------------
// flight_time_tau: τ(T) = max(w_θ|Δθ|, w_φ|Δφ|, w_ω|Δω|)
// ---------------------------------------------------------------------------

float flight_time_tau(
    const ShotParams& target_shot,
    const TurretState& current,
    const TurretWeights& weights,
    const OmegaMapParams& omega)
{
    float d_theta = std::abs(target_shot.theta - current.theta);
    // Wrap Δθ to [-π, π]
    if (d_theta > 3.14159265f) d_theta = 6.28318530f - d_theta;

    float d_phi   = std::abs(target_shot.phi   - current.phi);
    float d_omega = std::abs(target_shot.omega_flywheel - current.omega_flywheel);

    return std::max({weights.w_theta * d_theta,
                     weights.w_phi   * d_phi,
                     weights.w_omega * d_omega});
}

// ---------------------------------------------------------------------------
// Cold-start: Piyavskii-Shubert 1D Lipschitz minimization
// Mirrors ShotSolver.optimalAdjust from Kotlin.
// ---------------------------------------------------------------------------

FlightTimeResult flight_time_cold(
    float turret_x, float turret_y, float turret_z,
    float target_x, float target_y, float target_z,
    float robot_vx, float robot_vy,
    const TurretState& current,
    const TurretWeights& weights,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    float tol,
    int   maxIter)
{
    float dx = target_x - turret_x;
    float dy = target_y - turret_y;
    float dz = target_z - turret_z;

    TInterval iv = ballistics_feasible_interval(dx, dy, dz, robot_vx, robot_vy,
                                                bounds, cfg, tol / 2.f);
    if (iv.t_lo >= iv.t_hi) {
        // No feasible window
        FlightTimeResult r{};
        r.feasible = false;
        r.T_star   = 0.f;
        r.tau      = 1e9f;
        return r;
    }

    float t_lo = iv.t_lo, t_hi = iv.t_hi;

    // Compute Lipschitz constant of τ(T) over [t_lo, t_hi].
    // L_tau = max(w_theta*L_theta, w_phi*L_phi, w_omega*(dOmega/dphi*L_phi + dOmega/dv*L_v))
    auto compute_L = [&](float lo, float hi) -> float {
        LipschitzBounds lip = ballistics_lipschitz(dx, dy, dz, robot_vx, robot_vy, lo, hi, cfg);
        float t_mid = 0.5f * (lo + hi);
        float hw    = 0.5f * (hi - lo);
        ShotParams s_mid = ballistics_solve(turret_x, turret_y, turret_z,
                                            target_x, target_y, target_z,
                                            robot_vx, robot_vy, t_mid, cfg, omega);
        float phi_lo_range = s_mid.phi - lip.l_phi * hw;
        float phi_hi_range = s_mid.phi + lip.l_phi * hw;
        float v_lo_range   = s_mid.v_exit - lip.l_vexit * hw;
        float v_hi_range   = s_mid.v_exit + lip.l_vexit * hw;
        float l_om = omega_map_lipschitz(omega,
                                         phi_lo_range, phi_hi_range,
                                         v_lo_range,   v_hi_range,
                                         lip.l_phi, lip.l_vexit);
        return std::max({weights.w_theta * lip.l_theta,
                         weights.w_phi   * lip.l_phi,
                         weights.w_omega * l_om});
    };

    float L = compute_L(t_lo, t_hi);

    auto eval = [&](float T) -> float {
        ShotParams p = ballistics_solve(turret_x, turret_y, turret_z,
                                        target_x, target_y, target_z,
                                        robot_vx, robot_vy, T, cfg, omega);
        return flight_time_tau(p, current, weights, omega);
    };

    // Piyavskii-Shubert: sorted vector of (T, tau) pairs.
    // Contiguous storage avoids tree-pointer overhead of std::map.
    std::vector<std::pair<float,float>> samples;
    samples.reserve(maxIter + 4);

    auto insert_sample = [&](float T) {
        float v = eval(T);
        auto pos = std::lower_bound(samples.begin(), samples.end(),
                                    std::make_pair(T, -1e30f));
        samples.insert(pos, {T, v});
        return v;
    };

    float f_lo = insert_sample(t_lo);
    float f_hi = insert_sample(t_hi);

    float best_T    = (f_lo <= f_hi) ? t_lo : t_hi;
    float best_cost = std::min(f_lo, f_hi);
    float lb_global = -1e30f;

    float lip_lo = t_lo, lip_hi = t_hi;

    for (int iter = 0; iter < maxIter && best_cost - lb_global > tol; ++iter) {
        float new_lb     = 1e30f;
        float next_T     = 0.5f * (t_lo + t_hi);
        float active_lo  = 1e30f, active_hi = -1e30f;

        for (int j = 0; j + 1 < (int)samples.size(); ++j) {
            float ta = samples[j].first,   fa = samples[j].second;
            float tb = samples[j+1].first, fb = samples[j+1].second;
            float lb = 0.5f * (fa + fb) - 0.5f * L * (tb - ta);
            if (lb < best_cost) {
                active_lo = std::min(active_lo, ta);
                active_hi = std::max(active_hi, tb);
            }
            if (lb < new_lb) {
                new_lb = lb;
                next_T = std::clamp(0.5f * (ta + tb) + (fa - fb) / (2.f * L), ta, tb);
            }
        }
        lb_global = new_lb;

        // Tighten L on shrinking active range
        if (active_lo < 1e29f && (active_hi - active_lo) <= 0.5f * (lip_hi - lip_lo)) {
            L = compute_L(active_lo, active_hi);
            lip_lo = active_lo; lip_hi = active_hi;
            // Recompute lb_global with tighter L
            lb_global = 1e30f;
            for (int j = 0; j + 1 < (int)samples.size(); ++j) {
                float ta = samples[j].first,   fa = samples[j].second;
                float tb = samples[j+1].first, fb = samples[j+1].second;
                if (ta >= active_lo && tb <= active_hi) {
                    float lb = 0.5f * (fa + fb) - 0.5f * L * (tb - ta);
                    lb_global = std::min(lb_global, lb);
                }
            }
            if (lb_global > 1e29f) lb_global = new_lb;
            if (best_cost - lb_global <= tol) break;
            continue;
        }

        float cost = insert_sample(next_T);
        if (cost < best_cost) { best_cost = cost; best_T = next_T; }
    }

    ShotParams best_p = ballistics_solve(turret_x, turret_y, turret_z,
                                         target_x, target_y, target_z,
                                         robot_vx, robot_vy, best_T, cfg, omega);
    FlightTimeResult r;
    r.T_star   = best_T;
    r.params   = best_p;
    r.tau      = best_cost;
    r.feasible = ballistics_is_feasible(best_p, best_T, bounds, cfg, omega);
    r.iv       = iv;
    return r;
}

// ---------------------------------------------------------------------------
// Warm-started Newton refinement
// ---------------------------------------------------------------------------

// Inner implementation: Newton refinement given a pre-computed feasible interval.
static FlightTimeResult flight_time_warm_iv(
    float turret_x, float turret_y, float turret_z,
    float target_x, float target_y, float target_z,
    float robot_vx, float robot_vy,
    float T_init,
    TInterval iv,
    const TurretState& current,
    const TurretWeights& weights,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    int maxIter)
{
    float T = std::clamp(T_init, iv.t_lo, iv.t_hi);

    for (int iter = 0; iter < maxIter; ++iter) {
        ShotDerivatives derivs;
        ShotParams p = ballistics_solve_with_derivs(
            turret_x, turret_y, turret_z,
            target_x, target_y, target_z,
            robot_vx, robot_vy, T, cfg, omega, &derivs);

        // Current deltas from turret state to shot params
        float d_theta = p.theta - current.theta;
        if (d_theta >  3.14159265f) d_theta -= 6.28318530f;
        if (d_theta < -3.14159265f) d_theta += 6.28318530f;
        float d_phi   = p.phi   - current.phi;
        float d_omega = p.omega_flywheel - current.omega_flywheel;

        float arm_theta = weights.w_theta * std::abs(d_theta);
        float arm_phi   = weights.w_phi   * std::abs(d_phi);

        float d_om_dphi, d_om_dv;
        omega_map_partials(omega, p.phi, p.v_exit, &d_om_dphi, &d_om_dv);
        float domega_dT_total = d_om_dphi * derivs.dphi_dT + d_om_dv * derivs.dvexit_dT;
        float arm_omega = weights.w_omega * std::abs(d_omega);

        // Identify the active arm
        float tau = std::max({arm_theta, arm_phi, arm_omega});

        float dT;
        if (arm_theta >= arm_phi && arm_theta >= arm_omega && std::abs(d_theta) > 1e-9f) {
            // Newton step on theta arm: d/dT [w_theta * |d_theta(T)|] = 0
            // w_theta * sign(d_theta) * dtheta_dT = 0 → not directly solvable,
            // but we want to minimize |d_theta(T)| → d_theta(T) = 0 → T step
            float dT_theta = -d_theta / (derivs.dtheta_dT + 1e-12f);
            dT = dT_theta;
        } else if (arm_phi >= arm_theta && arm_phi >= arm_omega && std::abs(d_phi) > 1e-9f) {
            float dT_phi = -d_phi / (derivs.dphi_dT + 1e-12f);
            dT = dT_phi;
        } else if (std::abs(d_omega) > 1e-9f) {
            float dT_om = -d_omega / (domega_dT_total + 1e-12f);
            dT = dT_om;
        } else {
            break; // converged
        }

        // Line search: clamp and ensure tau decreases
        float T_new = std::clamp(T + dT, iv.t_lo, iv.t_hi);
        ShotParams p_new = ballistics_solve(turret_x, turret_y, turret_z,
                                            target_x, target_y, target_z,
                                            robot_vx, robot_vy, T_new, cfg, omega);
        float tau_new = flight_time_tau(p_new, current, weights, omega);

        if (tau_new < tau) {
            T = T_new;
        } else {
            // Bisect toward T_new if it doesn't help
            T_new = std::clamp(0.5f * (T + T_new), iv.t_lo, iv.t_hi);
            ShotParams p_half = ballistics_solve(turret_x, turret_y, turret_z,
                                                  target_x, target_y, target_z,
                                                  robot_vx, robot_vy, T_new, cfg, omega);
            if (flight_time_tau(p_half, current, weights, omega) < tau) {
                T = T_new;
            } else {
                break; // can't improve
            }
        }

        if (std::abs(dT) < 1e-5f) break;
    }

    ShotParams p_final = ballistics_solve(turret_x, turret_y, turret_z,
                                          target_x, target_y, target_z,
                                          robot_vx, robot_vy, T, cfg, omega);
    FlightTimeResult r;
    r.T_star   = T;
    r.params   = p_final;
    r.tau      = flight_time_tau(p_final, current, weights, omega);
    r.feasible = ballistics_is_feasible(p_final, T, bounds, cfg, omega);
    r.iv       = iv;
    return r;
}

// Public overload: pre-computed interval — skips 64-point feasibility sweep.
FlightTimeResult flight_time_warm(
    float turret_x, float turret_y, float turret_z,
    float target_x, float target_y, float target_z,
    float robot_vx, float robot_vy,
    float T_init,
    TInterval iv,
    const TurretState& current,
    const TurretWeights& weights,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    int maxIter)
{
    if (iv.t_lo >= iv.t_hi) {
        FlightTimeResult r{};
        r.feasible = false;
        r.T_star   = T_init;
        r.tau      = 1e9f;
        r.iv       = iv;
        return r;
    }
    return flight_time_warm_iv(turret_x, turret_y, turret_z,
                               target_x, target_y, target_z,
                               robot_vx, robot_vy, T_init, iv,
                               current, weights, bounds, cfg, omega, maxIter);
}

// Public overload: no interval — computes feasible interval first (original behaviour).
FlightTimeResult flight_time_warm(
    float turret_x, float turret_y, float turret_z,
    float target_x, float target_y, float target_z,
    float robot_vx, float robot_vy,
    float T_init,
    const TurretState& current,
    const TurretWeights& weights,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    int maxIter)
{
    float dx = target_x - turret_x;
    float dy = target_y - turret_y;
    float dz = target_z - turret_z;
    TInterval iv = ballistics_feasible_interval(dx, dy, dz, robot_vx, robot_vy,
                                                bounds, cfg, 1e-4f);
    if (iv.t_lo >= iv.t_hi) {
        FlightTimeResult r{};
        r.feasible = false;
        r.T_star   = T_init;
        r.tau      = 1e9f;
        r.iv       = iv;
        return r;
    }
    return flight_time_warm_iv(turret_x, turret_y, turret_z,
                               target_x, target_y, target_z,
                               robot_vx, robot_vy, T_init, iv,
                               current, weights, bounds, cfg, omega, maxIter);
}
