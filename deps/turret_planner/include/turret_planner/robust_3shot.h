#pragma once
#include "types.h"
#include "ballistics.h"
#include "flywheel_model.h"
#include "flight_time.h"

// ---------------------------------------------------------------------------
// Trajectory-aware N-shot robust planner (1, 2, or 3 balls).
//
// Given a predicted trajectory (array of FutureState), a stationary target,
// and the current turret state, finds when and how to fire up to 3 shots
// that minimize the total transition cost between consecutive shots.
//
// Shot timing is deterministic: t2 = t1 + transfer_time, t3 = t1 + 2*transfer_time.
// The solver sweeps over t1 candidates, and for each one runs a joint B&B over
// (T1, T2, T3) — the flight times — to minimize:
//
//   J = w_urgency * tau(cur -> s1(T1))
//     + moveCost(s1(T1)*(1-drop) -> s2(T2))
//     + moveCost(s2(T2)*(1-drop) -> s3(T3))
//
// where w_urgency = exp(-urgency_lambda * t_remaining) so that the slew to
// the first shot matters less when there's plenty of preparation time.
//
// For n_balls < 3, lower terms are omitted:
//   n_balls=1: J = w_urgency * tau(cur -> s1)
//   n_balls=2: J = w_urgency * tau(cur -> s1) + moveCost(s1_dropped -> s2)
// ---------------------------------------------------------------------------

struct FutureState {
    float t;           // seconds from now
    float x, y;        // predicted position (field frame)
    float heading;     // predicted heading (rad)
    float vx, vy;      // predicted velocity (field frame, m/s)
};

struct Robust3ShotResult {
    int   idx1, idx2, idx3;    // selected trajectory indices (-1 if unused)
    float T1, T2, T3;         // flight times (0 if unused)
    ShotParams s1, s2, s3;    // shot params at each trajectory point
    float J;                   // total objective value
    bool  feasible;            // true if a valid solution was found
};

Robust3ShotResult robust_3shot_plan(
    const FutureState* trajectory, int n_states,
    float turret_z,
    float target_x, float target_y, float target_z,
    const TurretState& current,
    int   n_balls,                     // 1, 2, or 3
    float t_remaining,                 // time until first shot can begin (s)
    float transfer_time,               // minimum time between consecutive shots (s)
    float drop_fraction,               // proportional flywheel speed loss per shot
    float urgency_lambda,              // decay rate for J_0 weight: w = exp(-lambda*t_remaining)
    const TurretWeights& weights,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    float tol       = 5e-4f,
    int   maxIter   = 80
);
