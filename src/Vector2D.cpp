#include "Vector2D.h"
#include <cmath>

Vector2D::Vector2D(float x, float y) : x(x), y(y) {}

Vector2D Vector2D::operator+(const Vector2D& other) const {
    return {x + other.x, y + other.y};
}

Vector2D Vector2D::operator-(const Vector2D& other) const {
    return {x - other.x, y - other.y};
}

Vector2D Vector2D::operator*(float scalar) const {
    return {x * scalar, y * scalar};
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
    if (mag == 0) return {0,0};
    return {x/mag, y/mag};
}
