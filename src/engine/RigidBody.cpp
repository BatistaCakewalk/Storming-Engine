//
// Created by Batista on 8/29/2025.
//

#include "RigidBody.h"

RigidBody::RigidBody(Vector2D pos, float mass)
    : position(pos), mass(mass), velocity(0,0), acceleration(0,0) {}

void RigidBody::applyForce(const Vector2D& force) {
    // F = m*a  -> a = F/m
    acceleration += force * (1.0f / mass);
}

void RigidBody::update(float dt) {
    velocity += acceleration * dt;
    position += velocity * dt;
    acceleration = Vector2D(0, 0); // reset after integration
}
