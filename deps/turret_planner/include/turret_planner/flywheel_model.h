#pragma once
#include <algorithm>

// ---------------------------------------------------------------------------
// OmegaMap: maps (phi, v_exit) → required flywheel speed.
//
// Uses inverse-distance-weighting (IDW, p=2) over raw measured data points.
// The JNI array format is: [N, phi_0, v_exit_0, omega_0, phi_1, v_exit_1, omega_1, ...]
// Normalization scales are computed from the data range so distances are dimensionless.
// ---------------------------------------------------------------------------

struct OmegaMapParams {
    static constexpr int MAX_POINTS = 200;
    int   n          = 0;
    float phi[MAX_POINTS];
    float v_exit[MAX_POINTS];
    float omega[MAX_POINTS];
    // Normalization: multiply input by scale to get unit range
    float phi_scale  = 1.0f;
    float v_scale    = 1.0f;
};

// Evaluate IDW interpolant at (phi, v_exit).
float omega_map_eval(const OmegaMapParams& m, float phi, float v_exit);

// Partial derivatives via central finite differences.
void omega_map_partials(const OmegaMapParams& m, float phi, float v_exit,
                        float* d_omega_dphi, float* d_omega_dvexit);

// Conservative Lipschitz bound on omega over the box
// phi in [phi_lo, phi_hi], v_exit in [v_lo, v_hi].
// Evaluates gradient at a 3×3 grid and returns
//   max|∂ω/∂φ| * l_phi + max|∂ω/∂v| * l_vexit
float omega_map_lipschitz(const OmegaMapParams& m,
                          float phi_lo, float phi_hi,
                          float v_lo,   float v_hi,
                          float l_phi, float l_vexit);
