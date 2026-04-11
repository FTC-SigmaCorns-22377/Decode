#pragma once

// ---------------------------------------------------------------------------
// OmegaMap: maps (phi, v_exit) → required flywheel speed.
//
// Implementations can be polynomial fits, bilinear lookup tables, etc.
// The interface mirrors the Kotlin OmegaMap trait.
// ---------------------------------------------------------------------------

struct OmegaMapParams {
    // Polynomial coefficients for a simple bivariate fit:
    //   omega(phi, v) = c0 + c1*v + c2*phi + c3*v^2 + c4*phi*v + c5*phi^2
    float c[6];
};

// Evaluate the polynomial map.
float omega_map_eval(const OmegaMapParams& m, float phi, float v_exit);

// Partial derivatives: d(omega)/d(phi) and d(omega)/d(v_exit).
void omega_map_partials(const OmegaMapParams& m, float phi, float v_exit,
                        float* d_omega_dphi, float* d_omega_dvexit);

// Lipschitz bound on omega over phi in [phi_lo, phi_hi], v_exit in [v_lo, v_hi].
// Given external Lipschitz bounds l_phi and l_vexit on how fast those inputs
// can change (e.g., from ShotDerivatives scaled by ΔT), this returns
//   L_omega ≤ |∂ω/∂φ|_max * l_phi + |∂ω/∂v|_max * l_vexit
float omega_map_lipschitz(const OmegaMapParams& m,
                          float phi_lo, float phi_hi,
                          float v_lo,   float v_hi,
                          float l_phi, float l_vexit);
