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
    // 1. Apply gravity
    velocity += Vector2D(0, 9.81f) * dt;
    position += velocity * dt;

    // 2. Update rotation
    angle += angularVelocity * dt;

    // 3. Ground collision
    float bottomY = position.y + height;
    if (bottomY > windowHeight) {
        position.y = windowHeight - height;

        // Bounce
        velocity.y *= -restitution;

        // Friction slows horizontal movement
        velocity.x *= 0.8f;

        // Torque from collision offset (corner hits)
        float contactOffsetX = (position.x + width / 2.0f) - (position.x + width / 2.0f);
        angularVelocity += contactOffsetX * -velocity.y * 0.05f;

        // Angular damping to prevent infinite spin
        angularVelocity *= 0.95f;
    }

    // 4. Stabilization if resting
    bool almostAtRest = std::abs(velocity.y) < 0.1f && bottomY >= windowHeight - 0.5f;
    if (almostAtRest) {
        // Snap to nearest 90°
        angle = std::round(angle / 90.0f) * 90.0f;
        angularVelocity = 0.0f;
        velocity.x *= 0.7f;  // friction slows horizontal movement
    }

    // 5. Clamp tiny velocities
    if (std::abs(velocity.x) < 0.01f) velocity.x = 0;
    if (std::abs(velocity.y) < 0.01f) velocity.y = 0;
}



AABB RigidBody::getAABB() const noexcept {
    return { position, position + Vector2D(width, height) };
}

Vector2D RigidBody::center() const noexcept {
    return position + Vector2D(width / 2.0f, height / 2.0f);
}
