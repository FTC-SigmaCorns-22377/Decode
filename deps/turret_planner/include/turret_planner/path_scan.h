#pragma once
#include "types.h"
#include "flight_time.h"

// ---------------------------------------------------------------------------
// Earliest feasible shot on a sampled path.
//
// Scans path samples in order, warm-starting T* across samples.
// On a feasibility transition, bisects the interval to sub-sample precision.
// ---------------------------------------------------------------------------

struct EarliestShotResult {
    float t_path;       // path time of the earliest feasible shot point
    float T_star;       // optimal flight time at that point
    ShotParams params;  // shot params
    int   sample_index; // which sample interval contains the result
    bool  found;        // false if no feasible shot exists on the path
};

// turret_pos_{x,y,z}: turret pivot in field frame (derived from robot pose + offset).
// target_z: target height (constant, or interpolate from samples if needed).
// t_now: current time; samples with t < t_now are skipped.
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
    int bisect_iters = 5
);
