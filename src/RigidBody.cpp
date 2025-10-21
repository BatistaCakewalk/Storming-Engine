#include "RigidBody.h"

void RigidBody::applyForce(const Vector2D &force, float dt) {
    Vector2D acceleration = force * (1.0F / mass);
    velocity += acceleration * dt;
}

void RigidBody::update(float dt, float windowHeight) {
    position += velocity * dt;

    // Ground collision
    if (position.y > windowHeight - height) {
        position.y = windowHeight - height;
        velocity.y *= -restitution;
    }
}
