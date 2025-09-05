//
// Created by Batista on 8/29/2025.
//

#include "RigidBody.h"

RigidBody::RigidBody(Vector2D pos, float m) : position(pos), mass(m), velocity(0, 0) {}

void RigidBody::applyForce(const Vector2D& force, float dt) {
    Vector2D acceleration = force * (1.0f / mass); // Speed Velocity
    velocity += acceleration * dt;
}

void RigidBody::update(float dt, float windowHeight) {
    position += velocity * dt;

    // Simple ground collision
    if (position.y > windowHeight - 50) { // 50 = box size
        position.y = windowHeight - 50;
        velocity.y *= -0.6f; // bounce with damping
    }
}

