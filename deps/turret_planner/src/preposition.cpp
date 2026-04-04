#include "turret_planner/preposition.h"
#include <cmath>
#include <algorithm>

PrepositionResult preposition_compute(
    const PathSample* path, int n_samples,
    float turret_x, float turret_y, float turret_z,
    float robot_vx, float robot_vy,
    float target_z,
    const TurretState& current,
    float t_available,
    float lambda_decay,
    int   k_samples,
    const TurretWeights& weights,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega)
{
    int K = std::min(k_samples, n_samples);

    float sum_w      = 0.f;
    float sum_theta  = 0.f;
    float sum_phi    = 0.f;
    float sum_om     = 0.f;

    float T_warm = -1.f;

    for (int i = 0; i < K; ++i) {
        const PathSample& s = path[i];

        FlightTimeResult res;
        if (T_warm < 0.f) {
            res = flight_time_cold(turret_x, turret_y, turret_z,
                                   s.x, s.y, target_z,
                                   robot_vx, robot_vy,
                                   current, weights, bounds, cfg, omega);
        } else {
            res = flight_time_warm(turret_x, turret_y, turret_z,
                                   s.x, s.y, target_z,
                                   robot_vx, robot_vy,
                                   T_warm, current, weights, bounds, cfg, omega);
        }
        if (!res.feasible) continue;
        T_warm = res.T_star;

        float w = std::exp(-lambda_decay * float(i));
        sum_w     += w;
        sum_theta += w * res.params.theta;
        sum_phi   += w * res.params.phi;
        sum_om    += w * res.params.omega_flywheel;
    }

    PrepositionResult result{};
    result.expected_earliest_t = (n_samples > 0) ? path[0].t : 0.f;

    if (sum_w < 1e-12f) {
        // No feasible samples: return current state unchanged
        result.target = current;
        return result;
    }

    float target_theta = sum_theta / sum_w;
    float target_phi   = sum_phi   / sum_w;
    float target_om    = sum_om    / sum_w;

    // Project onto reachable set given t_available:
    // Reachable range = [cur - t_avail/w_i, cur + t_avail/w_i] ∩ bounds
    auto reach = [](float cur, float target, float t_avail, float w,
                    float lo, float hi) -> float {
        float reach_lo = (w > 1e-9f) ? std::max(cur - t_avail / w, lo) : lo;
        float reach_hi = (w > 1e-9f) ? std::min(cur + t_avail / w, hi) : hi;
        reach_lo = std::max(reach_lo, lo);
        reach_hi = std::min(reach_hi, hi);
        return std::clamp(target, reach_lo, reach_hi);
    };

    result.target.theta = reach(current.theta, target_theta, t_available,
                                weights.w_theta, bounds.theta_min, bounds.theta_max);
    result.target.phi   = reach(current.phi,   target_phi,   t_available,
                                weights.w_phi,   bounds.phi_min,   bounds.phi_max);
    result.target.omega_flywheel = reach(current.omega_flywheel, target_om, t_available,
                                         weights.w_omega, 0.f, bounds.omega_max);

    return result;
}
