#pragma once
#include "types.h"
#include "flight_time.h"

// ---------------------------------------------------------------------------
// Robust pre-positioning before path entry.
//
// Computes a weighted centroid of shot params over the first K path samples,
// then projects onto the reachable set given available slew time.
// ---------------------------------------------------------------------------

struct PrepositionResult {
    TurretState target;         // recommended turret state to slew to
    float expected_earliest_t;  // estimated earliest-shot path time if we pre-position here
};

// path[0..n_samples-1]: the upcoming path.
// t_available: time before the path starts (s). Limits how far we can slew.
// target_z: constant target height.
// lambda_decay: exponential weighting e^{-lambda*i} on sample index i.
// k_samples: how many early samples to use (clamped to n_samples).
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
    const OmegaMapParams& omega
);
