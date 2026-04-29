#include "turret_planner/path_scan.h"
#include <cmath>
#include <algorithm>

EarliestShotResult path_scan_earliest(
    const PathSample* path, int n_samples,
    float turret_x, float turret_y, float turret_z,
    float robot_vx, float robot_vy,
    float target_z,
    float t_now,
    const TurretState& current,
    const TurretWeights& weights,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    int bisect_iters)
{
    EarliestShotResult result{};
    result.found = false;

    if (n_samples <= 0) return result;

    // Find first sample at or after t_now
    int start = 0;
    while (start < n_samples && path[start].t < t_now) ++start;
    if (start >= n_samples) return result;

    // Warm start T* and feasible interval from cold solve on the first usable sample.
    // Passing the cached interval to flight_time_warm skips the 64-point sweep.
    float T_warm = -1.f;
    TInterval iv_warm = {0.f, 0.f};

    bool prev_feasible = false;
    int  prev_idx      = -1;

    for (int i = start; i < n_samples; ++i) {
        const PathSample& s = path[i];

        FlightTimeResult res;
        if (T_warm < 0.f) {
            // Cold start on first sample
            res = flight_time_cold(turret_x, turret_y, turret_z,
                                   s.x, s.y, target_z,
                                   robot_vx, robot_vy,
                                   current, weights, bounds, cfg, omega);
        } else {
            // Warm start: reuse prior interval to skip feasibility sweep
            res = flight_time_warm(turret_x, turret_y, turret_z,
                                   s.x, s.y, target_z,
                                   robot_vx, robot_vy,
                                   T_warm, iv_warm,
                                   current, weights, bounds, cfg, omega);
        }

        if (res.feasible) { T_warm = res.T_star; iv_warm = res.iv; }

        if (!prev_feasible && res.feasible) {
            // Feasibility transition: bisect [prev_idx, i] for sub-sample precision
            if (prev_idx >= 0 && bisect_iters > 0) {
                const PathSample& pa = path[prev_idx];
                const PathSample& pb = path[i];
                float lo_t = pa.t, hi_t = pb.t;

                for (int b = 0; b < bisect_iters; ++b) {
                    float mid_t  = 0.5f * (lo_t + hi_t);
                    float alpha  = (n_samples > 1)
                                 ? (mid_t - pa.t) / (pb.t - pa.t + 1e-30f)
                                 : 0.f;
                    float mid_x  = pa.x  + alpha * (pb.x  - pa.x);
                    float mid_y  = pa.y  + alpha * (pb.y  - pa.y);

                    FlightTimeResult mid_res = flight_time_warm(
                        turret_x, turret_y, turret_z,
                        mid_x, mid_y, target_z,
                        robot_vx, robot_vy,
                        res.T_star, iv_warm,
                        current, weights, bounds, cfg, omega);

                    if (mid_res.feasible) {
                        hi_t = mid_t;
                        res  = mid_res;
                    } else {
                        lo_t = mid_t;
                    }
                }

                result.t_path      = hi_t;
                float alpha = (pb.t - pa.t > 1e-10f)
                            ? (hi_t - pa.t) / (pb.t - pa.t)
                            : 0.f;
                float final_x = pa.x + alpha * (pb.x - pa.x);
                float final_y = pa.y + alpha * (pb.y - pa.y);

                FlightTimeResult final_res = flight_time_warm(
                    turret_x, turret_y, turret_z,
                    final_x, final_y, target_z,
                    robot_vx, robot_vy,
                    res.T_star, iv_warm,
                    current, weights, bounds, cfg, omega);

                result.T_star       = final_res.T_star;
                result.params       = final_res.params;
                result.sample_index = i;
                result.found        = true;
            } else {
                result.t_path       = s.t;
                result.T_star       = res.T_star;
                result.params       = res.params;
                result.sample_index = i;
                result.found        = true;
            }
            return result;
        }

        // Fast-forward heuristic: if infeasible and T* is far from bounds,
        // we can skip ahead by estimating how many samples it takes for
        // the target to move into the feasible cone. Here we do a simple
        // skip of 0 extra (future optimization: estimate skip count).

        prev_feasible = res.feasible;
        prev_idx      = i;
        if (res.feasible) T_warm = res.T_star;
    }

    // If all remaining samples are feasible (started feasible), return first
    if (prev_feasible && start < n_samples) {
        const PathSample& s = path[start];
        FlightTimeResult res = (T_warm >= 0.f)
            ? flight_time_warm(turret_x, turret_y, turret_z,
                               s.x, s.y, target_z, robot_vx, robot_vy,
                               T_warm, iv_warm,
                               current, weights, bounds, cfg, omega)
            : flight_time_cold(turret_x, turret_y, turret_z,
                               s.x, s.y, target_z, robot_vx, robot_vy,
                               current, weights, bounds, cfg, omega);
        result.t_path       = s.t;
        result.T_star       = res.T_star;
        result.params       = res.params;
        result.sample_index = start;
        result.found        = res.feasible;
    }

    return result;
}
