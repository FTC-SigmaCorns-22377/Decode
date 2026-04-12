#pragma once
#include "types.h"
#include "flywheel_model.h"

// ---------------------------------------------------------------------------
// Ballistics: closed-form (θ, φ, v_exit) solver for a given flight time T.
//
// Coordinate conventions (matching Kotlin Ballistics.kt):
//   - theta: azimuth in field frame, measured from +x toward +y
//   - phi:   elevation from horizontal (positive = up)
//   - v_exit: required launch speed at turret exit
//   - r_h:   barrel offset (distance from pivot to launch point along barrel)
//   - g:     gravity, positive downward (~9.81 m/s²)
//
// "vR" = turret velocity in field frame = (robot.vx, robot.vy).
// For a stationary target: vR corrects for the robot's own motion.
//
// Full derivation:
//   b = dx/T - vR.x,  c = dy/T - vR.y,  a = -r_h/T
//   theta = atan2(c, b)
//   B = a + cos(theta)*b + sin(theta)*c
//   C = dz/T + 0.5*g*T
//   v_exit = sqrt(B² + C² - a²)
//   phi = atan2(C*v_exit + B*a,  B*v_exit - C*a)
// ---------------------------------------------------------------------------

// Intermediate scalar that all hot-path functions share.
// Exposed so callers can reuse it across repeated evaluations.
struct BallisticsIntermediate {
    float dx, dy, dz;     // target - turret displacement
    float vRx, vRy;       // turret velocity in field frame
    float b, c, a;        // = dx/T-vRx, dy/T-vRy, -r_h/T
    float theta;          // atan2(c,b)
    float sin_theta, cos_theta;
    float B, C;
    float v_exit;
    float phi;
};

// Compute the intermediate block. All other functions call this.
BallisticsIntermediate ballistics_intermediate(
    float dx, float dy, float dz,
    float vRx, float vRy,
    float T,
    const PhysicsConfig& cfg
);

// Solve for (theta, phi, v_exit, omega) given flight time T.
// `turret_pos_{x,y,z}` is the turret pivot in field frame.
ShotParams ballistics_solve(
    float turret_x, float turret_y, float turret_z,
    float target_x, float target_y, float target_z,
    float robot_vx, float robot_vy,
    float T,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega
);

// Same, but also fills in dθ/dT, dφ/dT, dv/dT, dω/dT analytically.
// Shares all intermediate computation with ballistics_solve.
ShotParams ballistics_solve_with_derivs(
    float turret_x, float turret_y, float turret_z,
    float target_x, float target_y, float target_z,
    float robot_vx, float robot_vy,
    float T,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    ShotDerivatives* out_derivs
);

// Feasibility check: v_exit in (0, v_max], phi in [phi_min, phi_max],
// shot is a lob (vertical velocity at T is ≤ 0),
// and theta (field frame) is in [theta_min+robot_heading, theta_max+robot_heading].
// Pass robot_heading = 0 if theta bounds are already in field frame, or if you
// don't want to check theta bounds (e.g., for continuous-rotation turrets).
bool ballistics_is_feasible(
    const ShotParams& p,
    float T,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    float robot_heading = 0.f
);

// dθ/dT standalone (useful for Lipschitz bounds).
float ballistics_dtheta_dT(float dx, float dy, float vRx, float vRy, float T,
                           const PhysicsConfig& cfg);

// Lipschitz bounds on |dθ/dT|, |dφ/dT|, |dv/dT| over [t_lo, t_hi].
// Used by the cold-start Lipschitz minimizer in flight_time.h.
struct LipschitzBounds {
    float l_theta;
    float l_phi;
    float l_vexit;
    float t_lo, t_hi;
};

LipschitzBounds ballistics_lipschitz(
    float dx, float dy, float dz,
    float vRx, float vRy,
    float t_lo, float t_hi,
    const PhysicsConfig& cfg
);

// Find the outer feasible T interval [t_min, t_max] via binary search.
// Returns {0,0} if no feasible interval exists.
struct TInterval { float t_lo, t_hi; };

TInterval ballistics_feasible_interval(
    float dx, float dy, float dz,
    float vRx, float vRy,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    float tol = 1e-4f
);
