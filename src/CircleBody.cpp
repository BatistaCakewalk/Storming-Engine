#include "CircleBody.h"

void CircleBody::applyForce(const Vector2D &force, float dt) noexcept {
    Vector2D acceleration = force * (1.0f / mass);
    velocity += acceleration * dt;
}

void CircleBody::update(float dt, float windowHeight) noexcept {
    // Linear motion
    position += velocity * dt;

    // Simple ground collision
    if (position.y > windowHeight - radius) {
        position.y = windowHeight - radius;
        velocity.y *= -restitution;
    }
}



