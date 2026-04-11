#pragma once
#include "types.h"
#include "ballistics.h"
#include "flywheel_model.h"

// ---------------------------------------------------------------------------
// Optimal flight time T* solver.
//
// Minimizes the turret readiness time:
//   τ(T) = max(w_θ|Δθ|, w_φ|Δφ|, w_ω|Δω|)
// where Δ = desired - current, over the feasible interval [t_lo, t_hi].
//
// Two entry points:
//   - Cold start (Piyavskii-Shubert): globally convergent, ~200-400ns.
//   - Warm Newton: 2-3 iterations from a good initial guess, ~80-150ns.
// ---------------------------------------------------------------------------

struct FlightTimeResult {
    float T_star;       // optimal flight time
    ShotParams params;  // shot params at T*
    float tau;          // readiness time τ(T*)
    bool feasible;      // params within TurretBounds
};

// Cold-start Piyavskii-Shubert 1D Lipschitz minimization.
// Converges to tol in the move-cost objective.
// Max ~maxIter evaluations of τ(T) = O(maxIter) ballistics solves.
FlightTimeResult flight_time_cold(
    float turret_x, float turret_y, float turret_z,
    float target_x, float target_y, float target_z,
    float robot_vx, float robot_vy,
    const TurretState& current,
    const TurretWeights& weights,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    float tol     = 5e-4f,
    int   maxIter = 50
);

// Warm-started Newton refinement from T_init.
// Identifies the active arm of the max, Newton-steps on it, re-checks.
// Usually 2-3 iterations from a warm start off a nearby path sample.
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
    int maxIter = 4
);

// Move cost τ(T): the max weighted change needed from current to the shot at T.
float flight_time_tau(
    const ShotParams& target_shot,
    const TurretState& current,
    const TurretWeights& weights,
    const OmegaMapParams& omega
);
