#ifndef CIRCLEBODY_H
#define CIRCLEBODY_H

#pragma once
#include "Body.h"

class CircleBody : public Body {
public:
    float radius;

    explicit CircleBody(Vector2D pos, float r = 25, float m = 1.0f,
                        float restitution = 0.6f, float friction = 0.8f)
        : Body(pos, m, BodyType::Circle, restitution, friction), radius(r) {}

    void applyForce(const Vector2D &force, float dt) noexcept override;
    void update(float dt, float windowHeight) noexcept override;

    [[nodiscard]] AABB getAABB() const noexcept override {
        return { position - Vector2D(radius, radius), position + Vector2D(radius, radius) };
    }

    [[nodiscard]] Vector2D center() const noexcept override {
        return position; // for circle, position is its center
    }
};

#endif
