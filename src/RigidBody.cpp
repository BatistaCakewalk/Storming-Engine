#include "RigidBody.h"
#include <algorithm>

void RigidBody::applyForce(const Vector2D &force, float dt) noexcept {
    Vector2D acceleration = force * (1.0f / mass);
    velocity += acceleration * dt;
}

void RigidBody::applyTorque(float torque, float dt) noexcept {
    // angular acceleration = torque / inertia
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
