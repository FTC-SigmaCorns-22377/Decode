#include "turret_planner/flywheel_model.h"
#include <cmath>
#include <algorithm>

// IDW p=2: omega = sum(w_i * omega_i) / sum(w_i)  where w_i = 1 / dist2_i
float omega_map_eval(const OmegaMapParams& m, float phi, float v_exit) {
    if (m.n <= 0) return 0.f;

    float phi_s = phi    * m.phi_scale;
    float v_s   = v_exit * m.v_scale;

    float num = 0.f, den = 0.f;
    for (int i = 0; i < m.n; ++i) {
        float dp = phi_s  - m.phi[i]    * m.phi_scale;
        float dv = v_s    - m.v_exit[i] * m.v_scale;
        float d2 = dp*dp + dv*dv;
        if (d2 < 1e-12f) return m.omega[i];
        float w = 1.f / d2;
        num += w * m.omega[i];
        den += w;
    }
    return (den > 0.f) ? num / den : 0.f;
}

bool omega_map_in_region(const OmegaMapParams& m, float phi, float v_exit) {
    if (m.n_levels < 2) return true;
    if (phi < m.phi_levels[0] || phi > m.phi_levels[m.n_levels - 1]) return false;

    // Binary search for the surrounding phi levels.
    int lo = 0, hi = m.n_levels - 2;
    while (lo < hi) {
        int mid = (lo + hi + 1) / 2;
        if (m.phi_levels[mid] <= phi) lo = mid; else hi = mid - 1;
    }
    // Linearly interpolate v_lo and v_hi between levels lo and lo+1.
    float span = m.phi_levels[lo + 1] - m.phi_levels[lo];
    float t = (span > 1e-6f) ? (phi - m.phi_levels[lo]) / span : 0.f;
    float v_lo = m.v_lo[lo] + t * (m.v_lo[lo + 1] - m.v_lo[lo]);
    float v_hi = m.v_hi[lo] + t * (m.v_hi[lo + 1] - m.v_hi[lo]);
    return v_exit >= v_lo && v_exit <= v_hi;
}

// Analytical IDW gradient: single O(n) pass using quotient rule.
// ω = Σ(w_i·ω_i) / Σw_i, w_i = 1/d²  →
// ∂ω/∂φ = (Σ(∂w_i/∂φ·ω_i)·Σw_i - Σ(w_i·ω_i)·Σ(∂w_i/∂φ)) / (Σw_i)²
// ∂w_i/∂φ = -2·dp·phi_scale / d⁴
void omega_map_partials(const OmegaMapParams& m, float phi, float v_exit,
                        float* d_omega_dphi, float* d_omega_dvexit) {
    if (m.n <= 0) { *d_omega_dphi = *d_omega_dvexit = 0.f; return; }

    float phi_s = phi    * m.phi_scale;
    float v_s   = v_exit * m.v_scale;

    float A = 0.f, B = 0.f;
    float C_phi = 0.f, D_phi = 0.f;
    float C_v   = 0.f, D_v   = 0.f;

    for (int i = 0; i < m.n; ++i) {
        float dp = phi_s - m.phi[i]    * m.phi_scale;
        float dv = v_s   - m.v_exit[i] * m.v_scale;
        float d2 = dp*dp + dv*dv;
        if (d2 < 1e-12f) { *d_omega_dphi = *d_omega_dvexit = 0.f; return; }
        float inv_d2  = 1.f / d2;
        float inv_d4  = inv_d2 * inv_d2;
        float dw_dphi = -2.f * dp * m.phi_scale * inv_d4;
        float dw_dv   = -2.f * dv * m.v_scale   * inv_d4;
        float w = inv_d2, om = m.omega[i];
        A += w;
        B += w * om;
        C_phi += dw_dphi * om;  D_phi += dw_dphi;
        C_v   += dw_dv   * om;  D_v   += dw_dv;
    }

    float inv_A2 = 1.f / (A * A);
    *d_omega_dphi   = (C_phi * A - B * D_phi) * inv_A2;
    *d_omega_dvexit = (C_v   * A - B * D_v  ) * inv_A2;
}

// Evaluate gradient at a 5-point plus-pattern over the box; take max absolute partials.
// 5 points (center + 4 cardinal extrema) vs prior 3x3=9 grid: 44% fewer IDW passes.
float omega_map_lipschitz(const OmegaMapParams& m,
                          float phi_lo, float phi_hi,
                          float v_lo,   float v_hi,
                          float l_phi, float l_vexit) {
    float max_dphi = 0.f, max_dv = 0.f;
    float phi_mid = 0.5f*(phi_lo+phi_hi), v_mid = 0.5f*(v_lo+v_hi);
    const float phi_pts[5] = {phi_mid, phi_lo, phi_hi, phi_mid, phi_mid};
    const float v_pts[5]   = {v_mid,   v_mid,  v_mid,  v_lo,    v_hi  };
    for (int k = 0; k < 5; ++k) {
        float dp, dv;
        omega_map_partials(m, phi_pts[k], v_pts[k], &dp, &dv);
        max_dphi = std::max(max_dphi, std::abs(dp));
        max_dv   = std::max(max_dv,   std::abs(dv));
    }
    return max_dphi * l_phi + max_dv * l_vexit;
}
