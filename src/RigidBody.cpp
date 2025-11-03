#include "RigidBody.h"
#include <algorithm>

RigidBody::RigidBody(Vector2D pos, float w, float h, float m, float restitution, float friction)
    : Body(pos, m, BodyType::Rectangle, restitution, friction),
      width(w), height(h)
{
    inertia = (1.0f / 12.0f) * mass * (width*width + height*height);
}

void RigidBody::applyForce(const Vector2D &force, float dt) noexcept {
    Vector2D acceleration = force * (1.0f / mass);
    velocity += acceleration * dt;
}

void RigidBody::applyTorque(float torque, float dt) noexcept {
    float angularAcceleration = torque / inertia;
    angularVelocity += angularAcceleration * dt;
}

void RigidBody::update(float dt, float windowHeight) noexcept {
    // Linear motion
    position += velocity * dt;

    // Angular motion
    angle += angularVelocity * dt;

    // Ground collision (simple)
    if (position.y > windowHeight - height) {
        position.y = windowHeight - height;
        velocity.y *= -restitution;

        // small torque on collision with ground based on velocity
        applyTorque(velocity.x * 0.1f, dt);
    }
}

AABB RigidBody::getAABB() const noexcept {
    return { position, position + Vector2D(width, height) };
}

Vector2D RigidBody::center() const noexcept {
    return position + Vector2D(width / 2.0f, height / 2.0f);
}
