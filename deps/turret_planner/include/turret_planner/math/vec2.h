#pragma once
#include <cmath>

struct Vec2 {
    float x, y;

    static constexpr Vec2 zero() { return {0.f, 0.f}; }

    constexpr Vec2 operator+(Vec2 o) const { return {x+o.x, y+o.y}; }
    constexpr Vec2 operator-(Vec2 o) const { return {x-o.x, y-o.y}; }
    constexpr Vec2 operator*(float s) const { return {x*s, y*s}; }
    constexpr Vec2 operator/(float s) const { return {x/s, y/s}; }

    constexpr float dot(Vec2 o) const { return x*o.x + y*o.y; }
    constexpr float cross(Vec2 o) const { return x*o.y - y*o.x; }  // scalar z
    float norm() const { return std::sqrt(x*x + y*y); }
    float norm_sq() const { return x*x + y*y; }
};

inline constexpr Vec2 operator*(float s, Vec2 v) { return v * s; }
