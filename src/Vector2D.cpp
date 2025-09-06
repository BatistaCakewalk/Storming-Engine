#include "Vector2D.h"
#include <cmath>

Vector2D::Vector2D(float x, float y) : x(x), y(y) {}

Vector2D Vector2D::operator+(const Vector2D& other) const {
    return Vector2D(x + other.x, y + other.y);
}

Vector2D Vector2D::operator-(const Vector2D& other) const {
    return Vector2D(x - other.x, y - other.y);
}

Vector2D Vector2D::operator*(float scalar) const {
    return Vector2D(x * scalar, y * scalar);
}

Vector2D& Vector2D::operator+=(const Vector2D& other) {
    x += other.x;
    y += other.y;
    return *this;
}

Vector2D& Vector2D::operator-=(const Vector2D& other) {
    x -= other.x;
    y -= other.y;
    return *this;
}

float Vector2D::magnitude() const {
    return std::sqrt(x*x + y*y);
}

Vector2D Vector2D::normalized() const {
    float mag = magnitude();
    if (mag == 0) return Vector2D(0,0);
    return Vector2D(x/mag, y/mag);
}
