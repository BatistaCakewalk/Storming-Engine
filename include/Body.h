#ifndef BODY_H
#define BODY_H

#pragma once
#include "Vector2D.h"

enum class BodyType {
    Circle,
    Rectangle
};

struct AABB {
    Vector2D min;
    Vector2D max;
};

class Body {
public:
    Vector2D position;
    Vector2D velocity;
    float mass;
    BodyType type;
    float restitution = 0.6f; // bounce
    float friction = 0.8f;    // friction

    Body(Vector2D pos, float m = 1.0f, BodyType t = BodyType::Circle)
        : position(pos), velocity(0, 0), mass(m), type(t) {}

    virtual void applyForce(const Vector2D &force, float dt) = 0;
    virtual void update(float dt, float windowHeight) = 0;
    virtual AABB getAABB() const = 0;

    virtual ~Body() = default;
};

#endif
