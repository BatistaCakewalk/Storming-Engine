#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#pragma once
#include "Body.h"

class RigidBody : public Body {
public:
    float width, height;

    // Angular properties
    float angle = 0.0f;              // rotation in degrees
    float angularVelocity = 0.0f;    // angular speed in degrees/sec
    float inertia;                    // moment of inertia

    explicit RigidBody(Vector2D pos, float w = 50, float h = 50, float m = 1.0f,
                       float restitution = 0.6f, float friction = 0.8f)
        : Body(pos, m, BodyType::Rectangle, restitution, friction),
          width(w), height(h) 
    {
        // Moment of inertia for rectangle about center
        inertia = (1.0f / 12.0f) * mass * (width*width + height*height);
    }

    void applyForce(const Vector2D &force, float dt) noexcept override;
    void applyTorque(float torque, float dt) noexcept; // new: torque application
    void update(float dt, float windowHeight) noexcept override;

    [[nodiscard]] AABB getAABB() const noexcept override {
        return { position, position + Vector2D(width, height) };
    }

    [[nodiscard]] Vector2D center() const noexcept override {
        return position + Vector2D(width / 2.0f, height / 2.0f);
    }
};

#endif
