#pragma once

// ---------------------------------------------------------------------------
// MecanumModel: constant-velocity state propagation with isotropic process noise.
//
// Used by ZoneTracker to propagate the distribution of the signed distance
// to the zone boundary (z = n·p - d) forward by t_pred seconds.
//
// State: (x, y) position in field frame.  Velocity is treated as known
// from odometry.  Process noise models unpredictable driver behaviour.
//
// Under constant-velocity prediction for time t:
//   μ_p(t)  = p(0) + v(0)*t
//   σ²_p(t) = σ²_p(0) + Q_pos * t       (additive position noise)
//
// Projected onto zone normal n = (nx, ny) with |n| = 1:
//   μ_z(t)  = n·μ_p(t) - d
//   σ²_z(t) = σ²_p(t)                   (isotropic → same variance along n)
// ---------------------------------------------------------------------------

// Result of projecting the propagated state onto the zone boundary normal.
struct ZoneProjection {
    float mu_z;   // mean signed distance at t: positive = inside zone
    float var_z;  // variance of signed distance at t
    float sigma_z; // sqrt(var_z), precomputed
};

// Project robot position + constant-velocity prediction onto the zone boundary.
// nx, ny: unit normal defining the zone boundary (n·p >= d is "inside").
// d: offset defining the half-plane.
// px, py: current robot position.
// vx, vy: current robot velocity in field frame.
// cov_pos_0: initial position variance (from odometry uncertainty), typically small.
// Q_pos: process noise variance per second (m²/s).
// t_pred: look-ahead time (s) — use the current turret readiness estimate τ(T*).
ZoneProjection mecanum_project_zone(
    float nx, float ny, float d,
    float px, float py,
    float vx, float vy,
    float cov_pos_0,
    float Q_pos,
    float t_pred
);
