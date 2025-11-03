#ifndef BODY_H
#define BODY_H

#pragma once
#include "Vector2D.h"
#include <utility>

enum class BodyType { Circle, Rectangle };

struct AABB {
    Vector2D min;
    Vector2D max;
};

class Body {
public:
    Vector2D position;
    Vector2D velocity{}; // <-- Add this line
    float mass;
    BodyType type;
    float restitution;
    float friction;

    Body(Vector2D pos, float m, BodyType t, float rest = 0.6f, float fric = 0.8f)
        : position(pos), mass(m), type(t), restitution(rest), friction(fric) {}

    virtual void applyForce(const Vector2D &force, float dt) noexcept = 0;
    virtual void update(float dt, float windowHeight) noexcept = 0;

    [[nodiscard]] virtual Vector2D center() const noexcept = 0;
    [[nodiscard]] virtual AABB getAABB() const noexcept = 0;

    virtual ~Body() = default;
};

#endif
