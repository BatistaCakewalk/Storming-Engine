#include "CircleBody.h"

CircleBody::CircleBody(Vector2D pos, float r, float m)
    : Body(pos, m), radius(r) {}

void CircleBody::applyForce(const Vector2D& force, float dt) {
    Vector2D acceleration = force * (1.0f / mass);
    velocity += acceleration * dt;
}

void CircleBody::update(float dt, float windowHeight) {
    position += velocity * dt;

    // Ground collision
    if (position.y > windowHeight - radius) {
        position.y = windowHeight - radius;
        velocity.y *= -0.6f;
    }
}
