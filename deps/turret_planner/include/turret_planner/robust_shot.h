#pragma once
#include "types.h"
#include "ballistics.h"
#include "flywheel_model.h"

// ---------------------------------------------------------------------------
// Robust shot pair planner.
//
// Given two consecutive targets and an expected flywheel-speed drop Δω
// incurred between shots, find (T1, T2) minimizing:
//
//   J(T1, T2) = max(w_θ|Δθ|, w_φ|Δφ|, w_ω|Δω_adj|)
//
// where s1 = ballistics_solve(target1, T1), s2 = ballistics_solve(target2, T2),
// Δω_adj = (ω(s1.phi, s1.v_exit) - omega_drop) - ω(s2.phi, s2.v_exit).
//
// This matches ShotSolver.optimalRobustShot on the Kotlin side.
// ---------------------------------------------------------------------------

struct RobustShotResult {
    float T1;
    float T2;
    ShotParams s1;     // ballistics_solve(target1, T1)
    ShotParams s2;     // ballistics_solve(target2, T2)
    float J;           // best objective value
    bool  feasible;    // both s1, s2 pass ballistics_is_feasible
};

// 2D Lipschitz branch-and-bound over (T1, T2) in the feasible rectangle.
RobustShotResult flight_time_robust(
    float turret_x, float turret_y, float turret_z,
    float target1_x, float target1_y, float target1_z,
    float target2_x, float target2_y, float target2_z,
    float robot_vx, float robot_vy,
    float omega_drop,
    const TurretWeights& weights,
    const TurretBounds&  bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    float tol     = 5e-4f,
    int   maxIter = 40
);

// ---------------------------------------------------------------------------
// Robust shot pair with an explicit current-state slew term.
//
// Minimizes the total time to fire two balls from a given current turret
// state under the same flywheel-drop model as flight_time_robust:
//
//     J(T1, T2) = J_Δ(cur → s1(T1))  +  J_Δ(s1(T1)_reduced → s2(T2))
//
// where J_Δ is the max-of-weighted-arms slew time and s1_reduced is s1 with
// its required flywheel speed decreased by omega_drop. The first term wraps
// Δθ to [-π, π] because `cur` may be in a different half-turn than s1; the
// second term matches the plain flight_time_robust convention (no wrap).
// ---------------------------------------------------------------------------
RobustShotResult flight_time_robust_adjust(
    float turret_x, float turret_y, float turret_z,
    float target1_x, float target1_y, float target1_z,
    float target2_x, float target2_y, float target2_z,
    float robot_vx, float robot_vy,
    const TurretState& current,
    float omega_drop,
    const TurretWeights& weights,
    const TurretBounds&  bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    float tol     = 5e-4f,
    int   maxIter = 40
);
