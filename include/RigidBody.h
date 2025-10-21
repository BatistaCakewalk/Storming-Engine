#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#pragma once
#include "Body.h"

class RigidBody : public Body {
public:
    float width, height;

    explicit RigidBody(Vector2D pos, float w = 50, float h = 50, float m = 1.0f)
        : Body(pos, m, BodyType::Rectangle), width(w), height(h) {}

    void applyForce(const Vector2D &force, float dt) override;
    void update(float dt, float windowHeight) override;

    [[nodiscard]] AABB getAABB() const override {
        return { position, position + Vector2D(width, height) };
    }
};

#endif
