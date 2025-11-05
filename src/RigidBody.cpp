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

    // 3. Simple ground collision
    float bottomY = position.y + height;
    if (bottomY > windowHeight) {
        position.y = windowHeight - height;
        velocity.y *= -restitution;
        velocity.x *= 0.9f; // friction

        // Torque from contact
        float contactOffsetX = (position.x + width / 2.0f) - (position.x + width / 2.0f); 
        float appliedTorque = contactOffsetX * -velocity.y * 0.05f;
        angularVelocity += appliedTorque;

        // Angular damping
        angularVelocity *= 0.98f;
    }

    // 4. Gradual stabilization (instead of snapping)
    if (bottomY >= windowHeight - 0.5f) {
        float targetAngle = std::round(angle / 90.0f) * 90.0f;
        float angleDiff = targetAngle - angle;
        // Apply a small proportional torque to gradually bring the angle closer
        angularVelocity += angleDiff * 0.1f; // tweak factor for smooth wobble
        // Dampen the angular velocity slightly to prevent endless oscillation
        angularVelocity *= 0.95f;
    }
}





AABB RigidBody::getAABB() const noexcept {
    return { position, position + Vector2D(width, height) };
}

Vector2D RigidBody::center() const noexcept {
    return position + Vector2D(width / 2.0f, height / 2.0f);
}
