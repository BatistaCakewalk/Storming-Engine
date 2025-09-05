#ifndef VECTOR2D_H
#define VECTOR2D_H

#pragma once

class Vector2D {
public:
    float x, y;

    Vector2D(float x = 0, float y = 0);
    Vector2D operator+(const Vector2D& other) const;
    Vector2D operator*(float scalar) const;
    Vector2D& operator+=(const Vector2D& other);
};


#endif
