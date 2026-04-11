#include "turret_planner/flywheel_model.h"
#include <cmath>
#include <algorithm>

// omega(phi, v) = c0 + c1*v + c2*phi + c3*v^2 + c4*phi*v + c5*phi^2
float omega_map_eval(const OmegaMapParams& m, float phi, float v_exit) {
    return m.c[0]
         + m.c[1] * v_exit
         + m.c[2] * phi
         + m.c[3] * v_exit * v_exit
         + m.c[4] * phi * v_exit
         + m.c[5] * phi * phi;
}

void omega_map_partials(const OmegaMapParams& m, float phi, float v_exit,
                        float* d_omega_dphi, float* d_omega_dvexit) {
    *d_omega_dphi   = m.c[2] + m.c[4] * v_exit + 2.f * m.c[5] * phi;
    *d_omega_dvexit = m.c[1] + 2.f * m.c[3] * v_exit + m.c[4] * phi;
}

float omega_map_lipschitz(const OmegaMapParams& m,
                          float phi_lo, float phi_hi,
                          float v_lo,   float v_hi,
                          float l_phi, float l_vexit) {
    // |∂ω/∂φ| max over box: c2 + c4*v + 2*c5*phi
    // Maximized at the extremes — evaluate at all four corners.
    float max_dphi   = 0.f;
    float max_dvexit = 0.f;
    for (float phi : {phi_lo, phi_hi}) {
        for (float v : {v_lo, v_hi}) {
            float dp, dv;
            omega_map_partials(m, phi, v, &dp, &dv);
            max_dphi   = std::max(max_dphi,   std::abs(dp));
            max_dvexit = std::max(max_dvexit, std::abs(dv));
        }
    }
    return max_dphi * l_phi + max_dvexit * l_vexit;
}
