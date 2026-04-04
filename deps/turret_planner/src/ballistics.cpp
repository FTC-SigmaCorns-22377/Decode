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

    float inv_T = 1.f / T;
    s.b = dx * inv_T - vRx;
    s.c = dy * inv_T - vRy;
    s.a = -cfg.r_h * inv_T;

    s.theta = fast_atan2(s.c, s.b);
    fast_sincos(s.theta, &s.sin_theta, &s.cos_theta);

    s.B = s.a + s.cos_theta * s.b + s.sin_theta * s.c;
    s.C = dz * inv_T + 0.5f * cfg.g * T;

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

        // --- dθ/dT ---
        float bc_sq = s.b * s.b + s.c * s.c;
        // Simplified: dθ/dT = (dy*vRx - dx*vRy) / (T² * bc_sq)
        float dtheta_dT = (dy * robot_vx - dx * robot_vy) / (T * T * bc_sq);

        // --- dB/dT ---
        float db_dT  = -dx * inv_T2;
        float dc_dT  = -dy * inv_T2;
        float da_dT  =  cfg.r_h * inv_T2;
        float dB_dT  = da_dT
                     + (-s.sin_theta * dtheta_dT) * s.b + s.cos_theta * db_dT
                     + ( s.cos_theta * dtheta_dT) * s.c + s.sin_theta * dc_dT;

        // --- dC/dT ---
        float dC_dT  = -dz * inv_T2 + 0.5f * cfg.g;

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
// ballistics_is_feasible
// ---------------------------------------------------------------------------

bool ballistics_is_feasible(
    const ShotParams& p,
    float T,
    const TurretBounds& bounds,
    const PhysicsConfig& cfg,
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

    // Lob check: vertical velocity at T must be ≤ 0 (ball descending at target).
    float vz_at_T = p.v_exit * std::sin(p.phi) - cfg.g * T;
    return vz_at_T <= 0.f;
}

// ---------------------------------------------------------------------------
// ballistics_dtheta_dT
// ---------------------------------------------------------------------------

float ballistics_dtheta_dT(float dx, float dy, float vRx, float vRy, float T)
{
    float b = dx / T - vRx;
    float c = dy / T - vRy;
    // Simplified form: numerator = (dy*vRx - dx*vRy) / T²
    return (dy * vRx - dx * vRy) / (T * T * (b * b + c * c));
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
    // --- L_theta ---
    float cross_term = dy * vRx - dx * vRy;
    float num_up = std::abs(cross_term) / (t_lo * t_lo);

    // Denominator = b²+c²; minimize over [t_lo, t_hi]
    auto bc_sq = [&](float t) {
        float b = dx / t - vRx;
        float c = dy / t - vRy;
        return b * b + c * c;
    };
    float den_lo = std::min(bc_sq(t_lo), bc_sq(t_hi));
    // Check zeros of b(T) = dx/vRx and c(T) = dy/vRy
    if (std::abs(vRx) > 1e-9f) {
        float tc = dx / vRx;
        if (tc >= t_lo && tc <= t_hi) {
            float c = dy / tc - vRy;
            den_lo = std::min(den_lo, c * c);
        }
    }
    if (std::abs(vRy) > 1e-9f) {
        float tc = dy / vRy;
        if (tc >= t_lo && tc <= t_hi) {
            float b = dx / tc - vRx;
            den_lo = std::min(den_lo, b * b);
        }
    }
    // Guard against degenerate case
    if (den_lo < 1e-12f) den_lo = 1e-12f;
    float l_theta = num_up / den_lo;

    // --- L_B and L_phi (sample N points, denser near t_lo where derivatives spike) ---
    // L_phi is evaluated analytically at each sample — the analytical formula is exact,
    // but the interval-based bound can under-estimate when B→0 near t_lo.
    // Sampling the analytical derivative directly gives a tight floor.
    float l_B        = 0.f;
    float l_phi_samp = 0.f;
    // Use 64 samples; cluster the first 16 near t_lo (where phi' spikes) and
    // the remaining 48 uniformly over the rest of the interval.
    const int N_lo = 16, N_rest = 48;
    // Near-boundary region: first 5% of interval
    float t_near = t_lo + 0.05f * (t_hi - t_lo);

    auto process_sample = [&](float t) {
        // L_B contribution
        float b = dx / t - vRx;
        float c = dy / t - vRy;
        float dtdT = ballistics_dtheta_dT(dx, dy, vRx, vRy, t);
        float term1 = dtdT * c - dx / (t * t);
        float term2 = dtdT * b + dy / (t * t);
        l_B = std::max(l_B, cfg.r_h / (t * t) + fast_sqrt(term1 * term1 + term2 * term2));

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
    // Evaluate B at midpoint for B0
    float t_mid = 0.5f * (t_lo + t_hi);
    BallisticsIntermediate s_mid = ballistics_intermediate(dx, dy, dz, vRx, vRy, t_mid, cfg);
    float B0  = s_mid.B;
    float delta_T = 0.5f * (t_hi - t_lo);
    float B_up  = std::abs(B0) + l_B * delta_T;
    float B_lo  = std::max(std::abs(B0) - l_B * delta_T, 0.f);

    // C(t) = dz/t + g/2*t; monotone if dz > 0 else has minimum at t = sqrt(2dz/g)
    float c_lo_t = dz / t_lo + 0.5f * cfg.g * t_lo;
    float c_hi_t = dz / t_hi + 0.5f * cfg.g * t_hi;
    float C_max  = std::max(std::abs(c_lo_t), std::abs(c_hi_t));
    float C_min;
    if (dz >= 0.f) {
        float tc = fast_sqrt(2.f * dz / cfg.g);
        if (tc >= t_lo && tc <= t_hi)
            C_min = fast_sqrt(2.f * cfg.g * dz);
        else
            C_min = std::min(std::abs(c_lo_t), std::abs(c_hi_t));
    } else {
        C_min = std::min(std::abs(c_lo_t), std::abs(c_hi_t));
    }

    float Cp_lo = -dz / (t_lo * t_lo) + 0.5f * cfg.g;
    float Cp_hi = -dz / (t_hi * t_hi) + 0.5f * cfg.g;
    float C_prime_max = std::max(std::abs(Cp_lo), std::abs(Cp_hi));

    // v² = B²+C²-a²; lower-bound v
    float a_max_sq = sq(cfg.r_h / t_lo);
    float v_sq_lo  = std::max(B_lo * B_lo + C_min * C_min - a_max_sq, 0.f);
    float v_lo_val = fast_sqrt(v_sq_lo);
    float v_up     = fast_sqrt(B_up * B_up + C_max * C_max);

    float l_v = 0.f;
    if (v_lo_val > 1e-6f) {
        l_v = (l_B * B_up + 0.25f * cfg.g * cfg.g * t_hi
               + (cfg.r_h * cfg.r_h + dz * dz) / (t_lo * t_lo * t_lo)) / v_lo_val;
    } else {
        l_v = 1e6f; // degenerate
    }

    // L_phi
    float denom_phi = B_lo * B_lo + C_min * C_min;
    float l_phi = 0.f;
    if (denom_phi > 1e-12f) {
        l_phi = (C_prime_max * B_up + l_B * C_max
                 + (cfg.r_h / (t_lo * t_lo)) * v_up
                 + l_v * (cfg.r_h / t_lo)) / denom_phi;
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
    // Upper bound on T: the trajectory that maximizes height.
    // At phi=45° approximately the maximum range case; use vMax at phi=pi/4.
    // Matches Kotlin tBounds: solve -g/2*T^2 + vMax/sqrt(2)*T - (dz + r_h/sqrt(2)) = 0
    float a_coef = cfg.g / 2.f;
    float b_coef = -bounds.v_exit_max / 1.41421356f;
    float c_coef =  dz - cfg.r_h / 1.41421356f;
    float disc   = b_coef * b_coef - 4.f * a_coef * c_coef;
    if (disc < 0.f) return {0.f, 0.f};  // target unreachable

    float T_upper_bound = (-b_coef + fast_sqrt(disc)) / (2.f * a_coef);
    if (T_upper_bound <= 0.f) return {0.f, 0.f};

    // Check if any feasible T exists in (0, T_upper_bound]
    auto is_feas = [&](float T) -> bool {
        if (T <= 0.f) return false;
        ShotParams p = ballistics_solve(0,0,0, dx,dy,dz, vRx,vRy, T, cfg,
                                        OmegaMapParams{});
        return ballistics_is_feasible(p, T, bounds, cfg);
    };

    // Binary search for t_max (upper edge of feasible window — the lob/speed limit)
    float lo = 0.f, hi = T_upper_bound;
    while (hi - lo > tol) {
        float m = 0.5f * (lo + hi);
        if (is_feas(m)) lo = m; else hi = m;
    }
    float t_max = lo;
    if (t_max <= 0.f) return {0.f, 0.f};

    // Binary search for t_min (lower edge — minimum flight time)
    lo = 0.f; hi = t_max;
    while (hi - lo > tol) {
        float m = 0.5f * (lo + hi);
        if (is_feas(m)) hi = m; else lo = m;
    }
    float t_min = hi;

    return {t_min, t_max};
}
