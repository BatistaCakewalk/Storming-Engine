#include "RigidBody.h"

RigidBody::RigidBody(Vector2D pos, float w, float h, float m)
    : Body(pos, m), width(w), height(h) {
}

void RigidBody::applyForce(const Vector2D &force, float dt) {
    Vector2D acceleration = force * (1.0f / mass);
    velocity += acceleration * dt;
}

void RigidBody::update(float dt, float windowHeight) {
    position += velocity * dt;

    // Simple ground collision
    if (position.y > windowHeight - height) {
        position.y = windowHeight - height;
        velocity.y *= -0.6f; // bounce
    }
}
