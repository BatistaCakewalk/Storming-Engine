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
    float restitution; // bounce
    float friction;    // surface friction

    explicit Body(Vector2D pos, float m = 1.0f, BodyType t = BodyType::Circle,
                  float restitution = 0.6f, float friction = 0.8f)
        : position(pos), velocity(0,0), mass(m), type(t), restitution(restitution), friction(friction) {}

    virtual void applyForce(const Vector2D &force, float dt) noexcept = 0;
    virtual void update(float dt, float windowHeight) noexcept = 0;
    [[nodiscard]] virtual AABB getAABB() const noexcept = 0;

    virtual ~Body() = default;

    // Utility: Center of body (for collision calculations / debugging)
    [[nodiscard]] virtual Vector2D center() const noexcept = 0;
};

#endif
