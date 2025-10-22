#include "RigidBody.h"
#include <cmath>

RigidBody::RigidBody(Vector2D pos, float w, float h, float m, float restitution, float friction)
    : Body(pos, m, BodyType::Rectangle, restitution, friction),
      width(w), height(h)
{
    inertia = mass / 12.0f * (width*width + height*height);
}

void RigidBody::applyForce(const Vector2D &force, float dt) noexcept {
    Vector2D accel = force / mass;
    velocity += accel * dt;
}

void RigidBody::applyForce(const Vector2D &force, const Vector2D& contactPoint, float dt) noexcept {
    Vector2D accel = force / mass;
    velocity += accel * dt;

    Vector2D r = contactPoint - center();
    float torque = r.x * force.y - r.y * force.x;
    angularVelocity += torque / inertia * dt;
}

void RigidBody::update(float dt, float windowHeight) noexcept {
    position += velocity * dt;
    angle += angularVelocity * dt;

    // Angular damping
    angularVelocity *= 0.98f;
    if (std::abs(angularVelocity) < 0.01f) angularVelocity = 0;

    // Floor collision
    auto box = getAABB();
    float bottom = box.max.y;
    if (bottom > windowHeight) {
        float corr = bottom - windowHeight;
        position.y -= corr;
        velocity.y *= -restitution;
        velocity.x *= 0.99f;
        angularVelocity *= 0.8f;

        if (std::abs(velocity.x) > 0.1f)
            angularVelocity += velocity.x * 0.05f;
    }
}

AABB RigidBody::getAABB() const noexcept {
    auto corners = getCorners();
    Vector2D min = corners[0], max = corners[0];
    for (auto& c : corners) {
        if (c.x < min.x) min.x = c.x;
        if (c.y < min.y) min.y = c.y;
        if (c.x > max.x) max.x = c.x;
        if (c.y > max.y) max.y = c.y;
    }
    return { min, max };
}

Vector2D RigidBody::center() const noexcept {
    return position + Vector2D(width / 2, height / 2);
}

std::vector<Vector2D> RigidBody::getCorners() const noexcept {
    Vector2D half(width/2, height/2);
    Vector2D c = center();
    std::vector<Vector2D> local = {
        {-half.x,-half.y}, {half.x,-half.y}, {half.x,half.y}, {-half.x,half.y}
    };
    std::vector<Vector2D> corners;
    float cosA = std::cos(angle), sinA = std::sin(angle);
    for(auto& p : local)
        corners.push_back({c.x + p.x * cosA - p.y * sinA, c.y + p.x * sinA + p.y * cosA});
    return corners;
}
