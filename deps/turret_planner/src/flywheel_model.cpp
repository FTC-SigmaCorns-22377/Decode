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

// Central finite differences, step ~0.1% of normalized range
void omega_map_partials(const OmegaMapParams& m, float phi, float v_exit,
                        float* d_omega_dphi, float* d_omega_dvexit) {
    const float h_phi = (m.phi_scale > 0.f) ? 1e-3f / m.phi_scale : 1e-4f;
    const float h_v   = (m.v_scale   > 0.f) ? 1e-3f / m.v_scale   : 1e-4f;
    *d_omega_dphi   = (omega_map_eval(m, phi + h_phi, v_exit) -
                       omega_map_eval(m, phi - h_phi, v_exit)) / (2.f * h_phi);
    *d_omega_dvexit = (omega_map_eval(m, phi, v_exit + h_v) -
                       omega_map_eval(m, phi, v_exit - h_v)) / (2.f * h_v);
}

// Evaluate gradient at a 3×3 grid over the box; take max absolute partials.
float omega_map_lipschitz(const OmegaMapParams& m,
                          float phi_lo, float phi_hi,
                          float v_lo,   float v_hi,
                          float l_phi, float l_vexit) {
    float max_dphi = 0.f, max_dv = 0.f;
    float phi_pts[3] = {phi_lo, 0.5f*(phi_lo+phi_hi), phi_hi};
    float v_pts[3]   = {v_lo,   0.5f*(v_lo+v_hi),     v_hi};
    for (float phi : phi_pts) {
        for (float v : v_pts) {
            float dp, dv;
            omega_map_partials(m, phi, v, &dp, &dv);
            max_dphi = std::max(max_dphi, std::abs(dp));
            max_dv   = std::max(max_dv,   std::abs(dv));
        }
    }
    return max_dphi * l_phi + max_dv * l_vexit;
}
