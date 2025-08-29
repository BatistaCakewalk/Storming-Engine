//
// Created by Batista on 8/29/2025.
//

#include "RigidBody.h"

RigidBody::RigidBody(Vector2D pos, float mass)
    : position(pos), velocity(0,0), acceleration(0,0), mass(mass) {}

void RigidBody::applyForce(const Vector2D& force) {
    // F = m*a  -> a = F/m
    acceleration += force * (1.0f / mass);
}

void RigidBody::update(const float dt, const float floorY) {
    velocity += acceleration * dt;
    position += velocity * dt;

    // Simple floor collision with damping
    if (position.y < floorY) {
        position.y = floorY;
        velocity.y = -velocity.y * 0.5f; // bounce, 50% energy retained
    }

    acceleration = Vector2D(0, 0); // reset after integration
}
