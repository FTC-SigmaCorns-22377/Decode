#pragma once
#include <cmath>

struct Vec3 {
    float x, y, z;

    static constexpr Vec3 zero() { return {0.f, 0.f, 0.f}; }

    constexpr Vec3 operator+(Vec3 o) const { return {x+o.x, y+o.y, z+o.z}; }
    constexpr Vec3 operator-(Vec3 o) const { return {x-o.x, y-o.y, z-o.z}; }
    constexpr Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }

    constexpr float dot(Vec3 o) const { return x*o.x + y*o.y + z*o.z; }
    float norm() const { return std::sqrt(x*x + y*y + z*z); }
    float norm_sq() const { return x*x + y*y + z*z; }
};

inline constexpr Vec3 operator*(float s, Vec3 v) { return v * s; }
