#include "RigidBody.h"

void RigidBody::applyForce(const Vector2D &force, float dt) noexcept {
    Vector2D acceleration = force * (1.0f / mass);
    velocity += acceleration * dt;
}

void RigidBody::update(float dt, float windowHeight) noexcept {
    position += velocity * dt;

    // Ground collision
    if (position.y > windowHeight - height) {
        position.y = windowHeight - height;
        velocity.y *= -restitution;
    }
}
