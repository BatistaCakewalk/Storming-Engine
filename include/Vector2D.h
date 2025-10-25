#ifndef VECTOR2D_H
#define VECTOR2D_H

#pragma once
#include <cmath>

class Vector2D {
public:
    float x, y;

    // Constructors
    constexpr Vector2D(float x = 0.0f, float y = 0.0f) : x(x), y(y) {}

    // Arithmetic operators
    constexpr Vector2D operator+(const Vector2D& other) const { return {x + other.x, y + other.y}; }
    constexpr Vector2D operator-(const Vector2D& other) const { return {x - other.x, y - other.y}; }
    constexpr Vector2D operator*(float scalar) const { return {x * scalar, y * scalar}; }
    constexpr Vector2D operator/(float scalar) const { return {x / scalar, y / scalar}; }
    constexpr Vector2D operator-() const { return {-x, -y}; }


    Vector2D& operator+=(const Vector2D& other) { x += other.x; y += other.y; return *this; }
    Vector2D& operator-=(const Vector2D& other) { x -= other.x; y -= other.y; return *this; }
    Vector2D& operator*=(float scalar) { x *= scalar; y *= scalar; return *this; }
    Vector2D& operator/=(float scalar) { x /= scalar; y /= scalar; return *this; }

    // Magnitude
    float magnitude() const { return std::sqrt(x*x + y*y); }
    float magnitudeSquared() const { return x*x + y*y; }

    // Normalization
    Vector2D normalized() const {
        float mag = magnitude();
        if (mag == 0.0f) return {0.0f, 0.0f};
        return {x / mag, y / mag};
    }

    // Dot product
    static float dot(const Vector2D& a, const Vector2D& b) {
        return a.x * b.x + a.y * b.y;
    }

    // Utility
    static Vector2D zero() { return {0.0f, 0.0f}; }
};

#endif
