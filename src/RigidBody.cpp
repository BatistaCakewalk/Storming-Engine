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
    // 1. Linear motion
    position += velocity * dt;

    // 2. Angular motion
    angle += angularVelocity * dt;

    // 3. Ground collision
    float bottomY = position.y + height;
    if (bottomY > windowHeight) {
        position.y = windowHeight - height;

        // Bounce in Y-axis
        velocity.y *= -restitution;

        // Small friction to slow X sliding
        velocity.x *= 0.85f;

        // Apply tiny torque based on off-center contact
        float contactOffsetX = (center().x - (position.x + width / 2.0f));
        float appliedTorque = contactOffsetX * -velocity.y * 0.02f; // reduced factor
        angularVelocity += appliedTorque / inertia;

        // Clamp angular velocity to avoid wild spins
        angularVelocity = std::clamp(angularVelocity, -5.0f, 5.0f);
    }

    // 4. Stabilize rectangle when nearly at rest
    if (bottomY >= windowHeight - 0.5f && std::abs(angularVelocity) < 0.1f) {
        angle = std::round(angle / 90.0f) * 90.0f; // snap to nearest right angle
        angularVelocity = 0.0f;
    }

    // 5. Small angular damping for stability
    angularVelocity *= 0.98f;
}




AABB RigidBody::getAABB() const noexcept {
    return { position, position + Vector2D(width, height) };
}

Vector2D RigidBody::center() const noexcept {
    return position + Vector2D(width / 2.0f, height / 2.0f);
}
