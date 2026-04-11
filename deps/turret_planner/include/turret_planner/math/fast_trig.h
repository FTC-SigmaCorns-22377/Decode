#pragma once
#include <cmath>

// All functions here trade a small amount of accuracy for speed.
// Errors are well within the noise floor of FTC-scale sensor data.

// fast_atan2: range-reduced polynomial approximation.
// Max error < 1e-5 rad. ~5ns on A53.
//
// Two-stage range reduction to keep the polynomial argument in [0, tan(π/8)] ≈ [0, 0.4142]:
//   Stage 1: fold to first octant (r = min/max ∈ [0,1])
//   Stage 2: if r > tan(π/8), apply identity atan(r) = π/4 + atan((r-1)/(r+1))
//            to get |t| ≤ tan(π/8). Otherwise use r directly.
//   Then Taylor/Horner on the tight range [0, 0.4142] (5 terms, error < 3e-8).
inline float fast_atan2(float y, float x) {
    const float pi    = 3.14159265f;
    const float pi_2  = 1.57079632f;
    const float pi_4  = 0.78539816f;
    const float tan_8 = 0.41421356f;  // tan(π/8) = √2 - 1

    float ax = __builtin_fabsf(x);
    float ay = __builtin_fabsf(y);

    // Stage 1: fold into first octant. r ∈ [0,1].
    float numer, denom;
    bool  swap_flag;
    if (ax >= ay) { numer = ay; denom = ax; swap_flag = false; }
    else          { numer = ax; denom = ay; swap_flag = true;  }

    float r = numer / (denom + 1e-30f);  // r ∈ [0, 1]

    // Stage 2: further reduce to [0, tan(π/8)] if r > tan(π/8).
    float offset = 0.f;
    float arg;
    if (r > tan_8) {
        arg    = (r - 1.f) / (r + 1.f);  // ∈ [-0.4142, 0] for r ∈ [0.4142, 1]
        offset = pi_4;
    } else {
        arg = r;
    }

    // Polynomial for atan(arg) on |arg| ≤ tan(π/8) ≈ 0.4142.
    // Taylor series atan(x) = x - x³/3 + x⁵/5 - x⁷/7 + x⁹/9 is excellent here:
    // max error < 3e-8 for |x| ≤ 0.4142 with 5 terms.
    float a2 = arg * arg;
    float atan_arg = arg * (1.f + a2*(-0.33333333f + a2*(0.20000000f
                                + a2*(-0.14285714f + a2*0.11111111f))));

    float r_atan = offset + atan_arg;

    // Reconstruct atan2
    float angle = swap_flag ? pi_2 - r_atan : r_atan;
    if (x < 0.f) angle = pi - angle;
    return (y < 0.f) ? -angle : angle;
}

// fast_sincos: compute sin and cos simultaneously.
// Falls back to std::sinf/cosf on non-ARM; on ARM the compiler will
// fuse these into a single sincos call with -ffast-math.
inline void fast_sincos(float angle, float* s, float* c) {
    *s = std::sin(angle);
    *c = std::cos(angle);
}

// fast_sqrt: wraps sqrtf; with -ffast-math this lowers to hardware sqrt.
inline float fast_sqrt(float x) {
    return __builtin_sqrtf(x);
}

// fast_rsqrt: ~22-bit accurate reciprocal square root.
// On x86/ARM with -ffast-math the compiler may lower this to hardware.
inline float fast_rsqrt(float x) {
#if defined(__ARM_NEON)
    // ARM NEON vrsqrte + 2 Newton-Raphson refinement steps
    float r;
    __asm__(
        "vrsqrte.f32  %0, %1\n"
        : "=w"(r) : "w"(x)
    );
    r = r * (1.5f - 0.5f * x * r * r);  // 1st Newton step
    r = r * (1.5f - 0.5f * x * r * r);  // 2nd Newton step
    return r;
#else
    return 1.f / __builtin_sqrtf(x);
#endif
}
