#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#pragma once
#include "Body.h"
#include <array>

class RigidBody : public Body {
public:
    float width, height;

    // Angular properties
    float angle = 0.0f;              // rotation in degrees
    float angularVelocity = 0.0f;    // angular speed in degrees/sec
    float inertia;                    // moment of inertia

    explicit RigidBody(Vector2D pos, float w = 50, float h = 50, float m = 1.0f,
                       float restitution = 0.6f, float friction = 0.8f);

    void applyForce(const Vector2D &force, float dt) noexcept override;
    void applyTorque(float torque, float dt) noexcept;
    void update(float dt, float windowHeight) noexcept override;

    [[nodiscard]] AABB getAABB() const noexcept override;
    [[nodiscard]] Vector2D center() const noexcept override;
};

#endif
