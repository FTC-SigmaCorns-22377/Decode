#pragma once

// ---------------------------------------------------------------------------
// Core data types for turret_planner.
// All floats; 4-byte aligned for NEON loads.
// ---------------------------------------------------------------------------

// Robot pose and velocity in field frame.
struct RobotState {
    float x, y, heading;   // position (m) + yaw (rad, 0 = +x field axis)
    float vx, vy;          // linear velocity in field frame (m/s)
    float omega_body;      // yaw rate (rad/s)
};

// Current turret state (angles relative to robot heading).
struct TurretState {
    float theta;           // azimuth relative to robot (rad)
    float phi;             // elevation from horizontal (rad)
    float omega_flywheel;  // flywheel speed (rad/s or RPM, same units as OmegaMap)
};

// Complete description of a single shot, given a flight time T.
struct ShotParams {
    float theta;           // azimuth (field frame, rad)
    float phi;             // elevation (rad)
    float v_exit;          // required launch speed (m/s)
    float omega_flywheel;  // required flywheel speed (from flywheel model)
};

// Partial derivatives of ShotParams w.r.t. flight time T and target position.
struct ShotDerivatives {
    // d/dT partials
    float dtheta_dT;
    float dphi_dT;
    float dvexit_dT;
    float domega_dT;
    // d/d(target_x), d/d(target_y) — for position-sensitivity weighting
    float dtheta_dx, dtheta_dy;
    float dphi_dx,   dphi_dy;
    float domega_dx, domega_dy;
};

// One sample along a path: time + target position/velocity.
struct PathSample {
    float t;              // path time (s)
    float x, y;          // target position (field frame, m)
    float vx, vy;        // target velocity (field frame, m/s); use 0 for stationary goal
};

// Hard limits on turret motion.
struct TurretBounds {
    float theta_min, theta_max;  // azimuth limits (relative to robot heading, rad)
    float phi_min,   phi_max;    // elevation limits (rad)
    float v_exit_max;            // max launch speed (m/s)
    float omega_max;             // max flywheel speed (same units as OmegaMap)
};

// Slew rates: seconds of motion per unit change in each axis.
// τ(T) = max(w_theta*|Δθ|, w_phi*|Δφ|, w_omega*|Δω|)
struct TurretWeights {
    float w_theta;   // s / rad
    float w_phi;     // s / rad
    float w_omega;   // s / (rad/s) or s / RPM — matches OmegaMap units
};

// Physical constants for ballistics.
struct PhysicsConfig {
    float g;       // gravitational acceleration (m/s^2, positive downward, ~9.81)
    float r_h;     // barrel offset: distance from turret pivot to launch point (m)
    float drag_k;  // linear air drag coefficient (1/s), 0 = no drag
};

// Configuration for zone-entry tracker.
struct ZoneConfig {
    float nx, ny, d;       // half-plane boundary: n·p >= d (n should be unit)
    float Q_process;       // process noise variance per second (m^2/s)
    float p_low, p_high;   // urgency thresholds: [p_low, p_high] → effort [0, 1]
    float alpha_lpf;       // low-pass coefficient (0 = no smoothing, 1 = hold)
    float omega_idle;      // flywheel idle speed (pre-spin floor)
};
