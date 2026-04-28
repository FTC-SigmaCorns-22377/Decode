#include "turret_planner/ballistics.h"
#include "turret_planner/math/fast_trig.h"
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static inline float sq(float x) { return x * x; }

// ---------------------------------------------------------------------------
// BallisticsIntermediate
// ---------------------------------------------------------------------------

BallisticsIntermediate ballistics_intermediate(
    float dx, float dy, float dz,
    float vRx, float vRy,
    float T,
    const PhysicsConfig& cfg)
{
    BallisticsIntermediate s;
    s.dx  = dx;  s.dy  = dy;  s.dz  = dz;
    s.vRx = vRx; s.vRy = vRy;

    // alpha replaces 1/T when drag is present.
    // No drag:   alpha = 1/T
    // With drag: alpha = k / (1 - exp(-k*T))
    // The correction accounts for exponential velocity decay: horizontal
    // distance = v0/k * (1 - exp(-kT)) instead of v0*T.
    float inv_T = 1.f / T;
    float alpha;
    if (cfg.drag_k > 1e-6f) {
        float kT = cfg.drag_k * T;
        float exp_neg_kT = std::exp(-kT);
        alpha = cfg.drag_k / (1.f - exp_neg_kT);
        // Vertical: with drag, z(T) = (vz + g/k)/k * (1-e^{-kT}) - g*T/k = dz
        // Solving for the "C" combination:
        s.C = (dz + cfg.g * T / cfg.drag_k) * alpha - cfg.g / cfg.drag_k;
    } else {
        alpha = inv_T;
        s.C = dz * inv_T + 0.5f * cfg.g * T;
    }
    s.b = dx * alpha - vRx;
    s.c = dy * alpha - vRy;
    s.a = -cfg.r_h * alpha;

    s.theta = fast_atan2(s.c, s.b);
    fast_sincos(s.theta, &s.sin_theta, &s.cos_theta);

    s.B = s.a + s.cos_theta * s.b + s.sin_theta * s.c;

    float v_sq = s.B * s.B + s.C * s.C - s.a * s.a;
    // v_sq < 0 can happen outside the feasible T range — clamp to 0.
    s.v_exit = (v_sq > 0.f) ? fast_sqrt(v_sq) : 0.f;

    // phi = atan2(C*v + B*a,  B*v - C*a)
    s.phi = fast_atan2(s.C * s.v_exit + s.B * s.a,
                       s.B * s.v_exit - s.C * s.a);
    return s;
}

// ---------------------------------------------------------------------------
// ballistics_solve
// ---------------------------------------------------------------------------

ShotParams ballistics_solve(
    float turret_x, float turret_y, float turret_z,
    float target_x, float target_y, float target_z,
    float robot_vx, float robot_vy,
    float T,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega)
{
    BallisticsIntermediate s = ballistics_intermediate(
        target_x - turret_x, target_y - turret_y, target_z - turret_z,
        robot_vx, robot_vy, T, cfg);

    ShotParams p;
    p.theta          = s.theta;
    p.phi            = s.phi;
    p.v_exit         = s.v_exit;
    p.omega_flywheel = omega_map_eval(omega, s.phi, s.v_exit);
    return p;
}

// ---------------------------------------------------------------------------
// ballistics_solve_with_derivs
//
// dθ/dT — from Kotlin dThetaDT, simplified form:
//   dθ/dT = (dy*vRx - dx*vRy) / (T² * (b² + c²))
//
// dv/dT — from chain rule on v² = B² + C² - a²:
//   2v * dv/dT = 2B*(dB/dT) + 2C*(dC/dT) - 2a*(da/dT)
//   dC/dT = -dz/T² + g/2
//   da/dT = r_h/T²
//   dB/dT = da/dT + cos(θ)*(dθ/dT*c - b/T) - sin(θ)*(dθ/dT*b + ... wait
//
// Full dB/dT derivation:
//   B = a + cos(θ)*b + sin(θ)*c
//   dB/dT = da/dT + (-sin(θ)*dθ/dT)*b + cos(θ)*db/dT
//                 + ( cos(θ)*dθ/dT)*c + sin(θ)*dc/dT
//   where db/dT = -dx/T², dc/dT = -dy/T²
//
// dφ/dT — from chain rule on φ = atan2(num, den):
//   num = C*v + B*a,  den = B*v - C*a
//   dφ/dT = (dnum/dT * den - num * dden/dT) / (num² + den²)
//         = (dnum/dT * den - num * dden/dT) / (B²+C²)
//         (since num² + den² = (Bv-Ca)² + (Cv+Ba)² = v²(B²+C²)/v² ... actually:
//          = (B²+C²) * v²  when a=0, but with a it expands; however
//          the atan2 denominator identity: num²+den² = (C*v+B*a)² + (B*v-C*a)²
//          = v²(C²+B²) + a²(B²+C²) - 0... let me just compute:
//          = C²v² + 2BCav + B²a² + B²v² - 2BCav + C²a²
//          = v²(B²+C²) + a²(B²+C²) = (v²+a²)(B²+C²)
//          and v² = B²+C²-a² → v²+a² = B²+C²
//          so num²+den² = (B²+C²)²  )
// ---------------------------------------------------------------------------

ShotParams ballistics_solve_with_derivs(
    float turret_x, float turret_y, float turret_z,
    float target_x, float target_y, float target_z,
    float robot_vx, float robot_vy,
    float T,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    ShotDerivatives* out_derivs)
{
    float dx = target_x - turret_x;
    float dy = target_y - turret_y;
    float dz = target_z - turret_z;

    BallisticsIntermediate s = ballistics_intermediate(
        dx, dy, dz, robot_vx, robot_vy, T, cfg);

    ShotParams p;
    p.theta          = s.theta;
    p.phi            = s.phi;
    p.v_exit         = s.v_exit;
    p.omega_flywheel = omega_map_eval(omega, s.phi, s.v_exit);

    if (out_derivs && s.v_exit > 1e-6f) {
        float inv_T  = 1.f / T;
        float inv_T2 = inv_T * inv_T;

        // d(alpha)/dT: derivative of the horizontal correction factor
        float dalpha_dT;
        float dC_dT;
        if (cfg.drag_k > 1e-6f) {
            float k = cfg.drag_k;
            float kT = k * T;
            float exp_neg_kT = std::exp(-kT);
            float one_minus_exp = 1.f - exp_neg_kT;
            // alpha = k / (1 - exp(-kT))
            // d(alpha)/dT = -k^2 * exp(-kT) / (1-exp(-kT))^2
            dalpha_dT = -k * k * exp_neg_kT / (one_minus_exp * one_minus_exp);
            // C = (dz + g*T/k) * alpha - g/k
            // dC/dT = (g/k) * alpha + (dz + g*T/k) * dalpha/dT
            float alpha = k / one_minus_exp;
            dC_dT = (cfg.g / k) * alpha + (dz + cfg.g * T / k) * dalpha_dT;
        } else {
            // alpha = 1/T, d(alpha)/dT = -1/T^2
            dalpha_dT = -inv_T2;
            dC_dT = -dz * inv_T2 + 0.5f * cfg.g;
        }

        // --- dθ/dT ---
        // b = dx*alpha - vRx, c = dy*alpha - vRy
        // db/dT = dx * dalpha_dT, dc/dT = dy * dalpha_dT
        // dθ/dT = (dc/dT * b - db/dT * c) / (b² + c²)
        //       = dalpha_dT * (dy*b - dx*c) / (b²+c²)
        float bc_sq = s.b * s.b + s.c * s.c;
        float dtheta_dT = dalpha_dT * (dy * s.b - dx * s.c) / bc_sq;

        // --- dB/dT ---
        float db_dT  = dx * dalpha_dT;
        float dc_dT  = dy * dalpha_dT;
        float da_dT  = -cfg.r_h * dalpha_dT;
        float dB_dT  = da_dT
                     + (-s.sin_theta * dtheta_dT) * s.b + s.cos_theta * db_dT
                     + ( s.cos_theta * dtheta_dT) * s.c + s.sin_theta * dc_dT;

        // --- dv/dT ---
        // 2v * dv/dT = 2B*dB/dT + 2C*dC/dT - 2a*da_dT
        float dv_dT  = (s.B * dB_dT + s.C * dC_dT - s.a * da_dT) / s.v_exit;

        // --- dφ/dT ---
        // φ = atan2(C*v+B*a, B*v-C*a)
        // num = C*v+B*a, den = B*v-C*a
        // d(num)/dT = dC/dT*v + C*dv/dT + dB/dT*a + B*da_dT
        // d(den)/dT = dB/dT*v + B*dv/dT - dC/dT*a - C*da_dT
        // num²+den² = (B²+C²)²  (shown above)
        float num = s.C * s.v_exit + s.B * s.a;
        float den = s.B * s.v_exit - s.C * s.a;
        float dnum_dT = dC_dT * s.v_exit + s.C * dv_dT + dB_dT * s.a + s.B * da_dT;
        float dden_dT = dB_dT * s.v_exit + s.B * dv_dT - dC_dT * s.a - s.C * da_dT;
        float BC_sq   = s.B * s.B + s.C * s.C;
        float dphi_dT = (dnum_dT * den - num * dden_dT) / (BC_sq * BC_sq);

        // --- dω/dT = ∂ω/∂φ * dφ/dT + ∂ω/∂v * dv/dT ---
        float d_om_dphi, d_om_dv;
        omega_map_partials(omega, s.phi, s.v_exit, &d_om_dphi, &d_om_dv);
        float domega_dT = d_om_dphi * dphi_dT + d_om_dv * dv_dT;

        out_derivs->dtheta_dT = dtheta_dT;
        out_derivs->dphi_dT   = dphi_dT;
        out_derivs->dvexit_dT = dv_dT;
        out_derivs->domega_dT = domega_dT;

        // Position partials (dθ/dx, dθ/dy, etc.) omitted for now — set to 0.
        // These are only needed for Mahalanobis weighting in preposition.
        out_derivs->dtheta_dx = out_derivs->dtheta_dy = 0.f;
        out_derivs->dphi_dx   = out_derivs->dphi_dy   = 0.f;
        out_derivs->domega_dx = out_derivs->domega_dy  = 0.f;
    }

    return p;
}

// ---------------------------------------------------------------------------
// ballistics_shot_error — forward-simulate and measure miss distance
// ---------------------------------------------------------------------------

float ballistics_shot_error(
    float turret_x, float turret_y, float turret_z,
    float target_x, float target_y, float target_z,
    float robot_vx, float robot_vy,
    const ShotParams& p,
    float T,
    const PhysicsConfig& cfg)
{
    float sin_theta, cos_theta, sin_phi, cos_phi;
    fast_sincos(p.theta, &sin_theta, &cos_theta);
    fast_sincos(p.phi, &sin_phi, &cos_phi);

    // Launch position: barrel arc projects the launch point based on hood angle.
    // This MUST match the solver's geometry exactly.
    // With hood arc: the launch point is offset along the barrel by r_h,
    // and the hood angle affects both vertical and horizontal projection.
    // Match the coordinate system from Kotlin Ballistics.kt:
    //   x0 = turret.x + rH * (1 - sin(phi)) * cos(theta)
    //   y0 = turret.y + rH * (1 - sin(phi)) * sin(theta)
    //   z0 = turret.z + rH * cos(phi)
    float sx = turret_x + cfg.r_h * (1.f - sin_phi) * cos_theta;
    float sy = turret_y + cfg.r_h * (1.f - sin_phi) * sin_theta;

    // Launch velocity components in field frame
    float vx0 = p.v_exit * cos_phi * cos_theta + robot_vx;
    float vy0 = p.v_exit * cos_phi * sin_theta + robot_vy;

    float xf, yf;
    if (cfg.drag_k > 1e-6f) {
        // With drag: pos(T) = pos0 + v0/k * (1 - exp(-kT))
        float k = cfg.drag_k;
        float decay = (1.f - std::exp(-k * T)) / k;
        xf = sx + vx0 * decay;
        yf = sy + vy0 * decay;
    } else {
        xf = sx + vx0 * T;
        yf = sy + vy0 * T;
    }

    float dx = xf - target_x;
    float dy = yf - target_y;
    return fast_sqrt(dx * dx + dy * dy);
}

// ---------------------------------------------------------------------------
// ballistics_is_feasible
// ---------------------------------------------------------------------------

bool ballistics_is_feasible(
    const ShotParams& p,
    float T,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    const OmegaMapParams& omega,
    float robot_heading)
{
    if (p.v_exit <= 0.f || p.v_exit > bounds.v_exit_max) return false;
    if (p.phi < bounds.phi_min || p.phi > bounds.phi_max) return false;

    // Theta check: convert to robot-relative frame and check bounds.
    // Skip if bounds span full circle (default: theta_min=-π, theta_max=π).
    if (bounds.theta_min > -3.14159f || bounds.theta_max < 3.14159f) {
        float theta_rel = p.theta - robot_heading;
        while (theta_rel >  3.14159265f) theta_rel -= 6.28318530f;
        while (theta_rel < -3.14159265f) theta_rel += 6.28318530f;
        if (theta_rel < bounds.theta_min || theta_rel > bounds.theta_max) return false;
    }

    // Lob check: vertical velocity at T must be < 0 (ball descending at target).
    float vz_at_T;
    if (cfg.drag_k > 1e-6f) {
        // With drag: vz(T) = (vz0 + g/k) * exp(-kT) - g/k
        float vz0 = p.v_exit * std::sin(p.phi);
        float g_over_k = cfg.g / cfg.drag_k;
        vz_at_T = (vz0 + g_over_k) * std::exp(-cfg.drag_k * T) - g_over_k;
    } else {
        vz_at_T = p.v_exit * std::sin(p.phi) - cfg.g * T;
    }
    return vz_at_T < 0.f;
}

// ---------------------------------------------------------------------------
// ballistics_dtheta_dT
// ---------------------------------------------------------------------------

float ballistics_dtheta_dT(float dx, float dy, float vRx, float vRy, float T,
                           const PhysicsConfig& cfg)
{
    float alpha, dalpha_dT;
    if (cfg.drag_k > 1e-6f) {
        float kT = cfg.drag_k * T;
        float exp_neg_kT = std::exp(-kT);
        float one_minus_exp = 1.f - exp_neg_kT;
        alpha = cfg.drag_k / one_minus_exp;
        dalpha_dT = -cfg.drag_k * cfg.drag_k * exp_neg_kT / (one_minus_exp * one_minus_exp);
    } else {
        alpha = 1.f / T;
        dalpha_dT = -1.f / (T * T);
    }
    float b = dx * alpha - vRx;
    float c = dy * alpha - vRy;
    // dθ/dT = dalpha_dT * (dy*b - dx*c) / (b²+c²)
    return dalpha_dT * (dy * b - dx * c) / (b * b + c * c);
}

// ---------------------------------------------------------------------------
// ballistics_lipschitz
//
// Matches the structure of Kotlin lipschitzBounds but in C++ / float.
// ---------------------------------------------------------------------------

LipschitzBounds ballistics_lipschitz(
    float dx, float dy, float dz,
    float vRx, float vRy,
    float t_lo, float t_hi,
    const PhysicsConfig& cfg)
{
    // Helper: compute alpha(t) for drag-corrected ballistics
    auto compute_alpha = [&](float t) -> float {
        if (cfg.drag_k > 1e-6f) {
            float kT = cfg.drag_k * t;
            return cfg.drag_k / (1.f - std::exp(-kT));
        }
        return 1.f / t;
    };

    // Helper: compute d(alpha)/dT
    auto compute_dalpha_dT = [&](float t) -> float {
        if (cfg.drag_k > 1e-6f) {
            float k = cfg.drag_k;
            float kT = k * t;
            float exp_neg_kT = std::exp(-kT);
            float one_minus_exp = 1.f - exp_neg_kT;
            return -k * k * exp_neg_kT / (one_minus_exp * one_minus_exp);
        }
        return -1.f / (t * t);
    };

    // Helper: compute C(t) for the vertical ballistics equation
    auto compute_C = [&](float t) -> float {
        if (cfg.drag_k > 1e-6f) {
            float alpha = compute_alpha(t);
            return (dz + cfg.g * t / cfg.drag_k) * alpha - cfg.g / cfg.drag_k;
        }
        return dz / t + 0.5f * cfg.g * t;
    };

    // --- L_theta ---
    // With drag: b = dx*alpha - vRx, c = dy*alpha - vRy
    // dθ/dT = dalpha_dT * (dy*b - dx*c) / (b²+c²)
    // Sample |dθ/dT| to get a tight bound.
    float l_theta = 0.f;

    // Denominator = b²+c²; minimize over [t_lo, t_hi] via sampling
    auto bc_sq = [&](float t) {
        float alpha = compute_alpha(t);
        float b = dx * alpha - vRx;
        float c = dy * alpha - vRy;
        return b * b + c * c;
    };
    float den_lo = std::min(bc_sq(t_lo), bc_sq(t_hi));
    // Sample interior for minimum
    for (int i = 1; i < 16; ++i) {
        float t = t_lo + (t_hi - t_lo) * float(i) / 16.f;
        den_lo = std::min(den_lo, bc_sq(t));
    }
    if (den_lo < 1e-12f) den_lo = 1e-12f;

    // --- L_B and L_phi (sample N points, denser near t_lo where derivatives spike) ---
    float l_B        = 0.f;
    float l_phi_samp = 0.f;
    const int N_lo = 16, N_rest = 48;
    float t_near = t_lo + 0.05f * (t_hi - t_lo);

    auto process_sample = [&](float t) {
        float alpha = compute_alpha(t);
        float dalpha = compute_dalpha_dT(t);
        float b = dx * alpha - vRx;
        float c = dy * alpha - vRy;
        float dtdT = ballistics_dtheta_dT(dx, dy, vRx, vRy, t, cfg);

        // |dθ/dT| for L_theta
        l_theta = std::max(l_theta, std::abs(dtdT));

        // L_B contribution: |dB/dT| upper bound via Cauchy-Schwarz
        float db_dT = dx * dalpha;
        float dc_dT = dy * dalpha;
        float term1 = dtdT * c + dc_dT;  // from sin(θ) component
        float term2 = dtdT * b + db_dT;  // from cos(θ) component — sign doesn't matter for norm
        float da_dT = -cfg.r_h * dalpha;
        l_B = std::max(l_B, std::abs(da_dT) + fast_sqrt(term1 * term1 + term2 * term2));

        // Analytical |dφ/dT| — reuse solve_with_derivs intermediates
        if (t > 0.f) {
            ShotDerivatives derivs_tmp;
            OmegaMapParams om_zero{};
            ballistics_solve_with_derivs(0,0,0, dx,dy,dz, vRx,vRy, t, cfg, om_zero, &derivs_tmp);
            l_phi_samp = std::max(l_phi_samp, std::abs(derivs_tmp.dphi_dT));
        }
    };

    for (int i = 0; i <= N_lo; ++i) {
        float t = t_lo + (t_near - t_lo) * float(i) / float(N_lo);
        process_sample(t);
    }
    for (int i = 1; i <= N_rest; ++i) {
        float t = t_near + (t_hi - t_near) * float(i) / float(N_rest);
        process_sample(t);
    }

    // --- Bounds on B, C, C' for L_v and L_phi ---
    float t_mid = 0.5f * (t_lo + t_hi);
    BallisticsIntermediate s_mid = ballistics_intermediate(dx, dy, dz, vRx, vRy, t_mid, cfg);
    float B0  = s_mid.B;
    float delta_T = 0.5f * (t_hi - t_lo);
    float B_up  = std::abs(B0) + l_B * delta_T;
    float B_lo  = std::max(std::abs(B0) - l_B * delta_T, 0.f);

    // C bounds via sampling (handles both drag and no-drag cases)
    float c_lo_t = compute_C(t_lo);
    float c_hi_t = compute_C(t_hi);
    float C_max  = std::max(std::abs(c_lo_t), std::abs(c_hi_t));
    float C_min  = std::min(std::abs(c_lo_t), std::abs(c_hi_t));
    // Sample interior for C_min/C_max (handles both drag and no-drag)
    for (int i = 1; i < 16; ++i) {
        float t = t_lo + (t_hi - t_lo) * float(i) / 16.f;
        float C_t = compute_C(t);
        C_max = std::max(C_max, std::abs(C_t));
        C_min = std::min(C_min, std::abs(C_t));
    }

    // C'(t) = dC/dT bounds via analytical formula (same as ballistics_solve_with_derivs).
    // No drag: dC/dT = -dz/T^2 + g/2
    // With drag: dC/dT = (g/k)*alpha + (dz + g*T/k)*dalpha_dT
    float C_prime_max = 0.f;
    for (int i = 0; i <= 16; ++i) {
        float t = t_lo + (t_hi - t_lo) * float(i) / 16.f;
        if (t <= 0.f) continue;
        float dC_dt;
        if (cfg.drag_k > 1e-6f) {
            float alpha = compute_alpha(t);
            float da    = compute_dalpha_dT(t);
            dC_dt = (cfg.g / cfg.drag_k) * alpha + (dz + cfg.g * t / cfg.drag_k) * da;
        } else {
            float inv_T2 = 1.f / (t * t);
            dC_dt = -dz * inv_T2 + 0.5f * cfg.g;
        }
        C_prime_max = std::max(C_prime_max, std::abs(dC_dt));
    }

    // v² = B²+C²-a²; lower-bound v.
    // a = -r_h * alpha(t); |a|_max occurs at the smallest t (largest alpha)
    float alpha_max = compute_alpha(t_lo);
    float a_max_sq = sq(cfg.r_h * alpha_max);
    float v_sq_lo  = std::max(B_lo * B_lo + C_min * C_min - a_max_sq, 0.f);
    float v_lo_val = fast_sqrt(v_sq_lo);
    float v_up     = fast_sqrt(B_up * B_up + C_max * C_max);

    // L_v: dv/dT = (B*dB/dT + C*dC/dT - a*da/dT) / v
    // Upper bound: (|B|_max * L_B + |C|_max * C'_max + |a|_max * |da/dT|_max) / v_lo
    float da_dT_max = cfg.r_h * std::abs(compute_dalpha_dT(t_lo)); // largest at t_lo
    float l_v = 0.f;
    if (v_lo_val > 1e-6f) {
        l_v = (l_B * B_up + C_max * C_prime_max + alpha_max * cfg.r_h * da_dT_max) / v_lo_val;
    } else {
        l_v = 1e6f; // degenerate
    }

    // L_phi
    float denom_phi = B_lo * B_lo + C_min * C_min;
    float l_phi = 0.f;
    if (denom_phi > 1e-12f) {
        l_phi = (C_prime_max * B_up + l_B * C_max
                 + alpha_max * cfg.r_h * v_up
                 + l_v * cfg.r_h * alpha_max) / denom_phi;
    } else {
        l_phi = 1e6f;
    }
    // Take the max with the directly-sampled bound to guard against
    // analytical formula breakdown near the feasibility boundary (B→0).
    l_phi = std::max(l_phi, l_phi_samp);

    return { l_theta, l_phi, l_v, t_lo, t_hi };
}

// ---------------------------------------------------------------------------
// ballistics_feasible_interval
// ---------------------------------------------------------------------------

TInterval ballistics_feasible_interval(
    float dx, float dy, float dz,
    float vRx, float vRy,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
    float tol)
{
    // Upper bound on T: the trajectory that maximizes flight time.
    // No-drag: solve g/2*T^2 - vMax/sqrt(2)*T + (dz - r_h/sqrt(2)) = 0
    // With drag the ball decelerates so max flight time increases; we apply a
    // generous multiplier to avoid cutting off feasible solutions.
    float a_coef = cfg.g / 2.f;
    float b_coef = -bounds.v_exit_max / 1.41421356f;
    float c_coef =  dz - cfg.r_h / 1.41421356f;
    float disc   = b_coef * b_coef - 4.f * a_coef * c_coef;
    if (disc < 0.f) return {0.f, 0.f};  // target unreachable

    float T_upper_bound = (-b_coef + fast_sqrt(disc)) / (2.f * a_coef);
    if (T_upper_bound <= 0.f) return {0.f, 0.f};

    // With drag, flight times are longer (velocity decays); scale up the bound.
    if (cfg.drag_k > 1e-6f) {
        T_upper_bound *= (1.f + cfg.drag_k * T_upper_bound);
    }

    // Check if any feasible T exists in (0, T_upper_bound]
    // Precompute unit target direction for the forward-shot check.
    float dist_sq = dx * dx + dy * dy;

    auto is_feas = [&](float T) -> bool {
        if (T <= 0.f) return false;
        ShotParams p = ballistics_solve(0,0,0, dx,dy,dz, vRx,vRy, T, cfg,
                                        OmegaMapParams{});
        if (!ballistics_is_feasible(p, T, bounds, cfg, OmegaMapParams{})) return false;

        // Reject backwards-pointing shots: the launch direction must have
        // a positive component along the geometric target direction.
        // When the robot's velocity dominates dx/T (or dy/T), atan2 can
        // flip theta ~180°, producing a physically valid but impractical
        // solution where the turret points away from the goal.
        if (dist_sq > 1e-6f) {
            float dot = std::cos(p.theta) * dx + std::sin(p.theta) * dy;
            if (dot <= 0.f) return false;
        }

        return true;
    };

    // Phase 1: coarse sweep to find the outer feasible envelope.
    // The lob constraint can make feasibility non-monotone in T, so a single
    // binary search may miss the true boundary. Sweep first, refine second.
    const int N_SWEEP = 64;
    float dt_sweep = T_upper_bound / float(N_SWEEP);
    float global_t_min = T_upper_bound;
    float global_t_max = 0.f;

    for (int i = 1; i <= N_SWEEP; ++i) {
        float t = float(i) * dt_sweep;
        if (is_feas(t)) {
            global_t_min = std::min(global_t_min, t);
            global_t_max = std::max(global_t_max, t);
        }
    }

    if (global_t_max <= 0.f) return {0.f, 0.f};

    // Phase 2: binary-search refine the outer edges of the feasible region.
    // Refine t_max in [global_t_max, global_t_max + dt_sweep]
    float lo = global_t_max;
    float hi = std::min(global_t_max + dt_sweep, T_upper_bound);
    while (hi - lo > tol) {
        float m = 0.5f * (lo + hi);
        if (is_feas(m)) lo = m; else hi = m;
    }
    float t_max = lo;

    // Refine t_min in [global_t_min - dt_sweep, global_t_min]
    lo = std::max(global_t_min - dt_sweep, tol);
    hi = global_t_min;
    while (hi - lo > tol) {
        float m = 0.5f * (lo + hi);
        if (is_feas(m)) hi = m; else lo = m;
    }
    float t_min = hi;

    return {t_min, t_max};
}
