#include "PhysicsWorld.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <unordered_map>

// ----------------- Broad-phase and Step -----------------
constexpr float CELL_SIZE = 100.0f;

PhysicsWorld::PhysicsWorld(Vector2D g) noexcept : gravity(g) {}

void PhysicsWorld::addBody(Body* body) noexcept {
    bodies.push_back(body);
}

void PhysicsWorld::step(float dt, float windowHeight) noexcept {
    for (Body* body : bodies) {
        body->applyForce(gravity * body->mass, dt);
        body->update(dt, windowHeight);
    }
    handleCollisions();
}

std::vector<std::pair<Body*, Body*>> PhysicsWorld::broadPhasePairs() noexcept {
    std::unordered_map<int, GridCell> grid;
    std::vector<std::pair<Body*, Body*>> pairs;
    auto hash = [](int x, int y) { return (x << 16) ^ y; };

    for (Body* body : bodies) {
        AABB box = body->getAABB();
        int minX = int(box.min.x / CELL_SIZE);
        int minY = int(box.min.y / CELL_SIZE);
        int maxX = int(box.max.x / CELL_SIZE);
        int maxY = int(box.max.y / CELL_SIZE);

        for (int gx = minX; gx <= maxX; ++gx) {
            for (int gy = minY; gy <= maxY; ++gy) {
                int h = hash(gx, gy);
                for (Body* other : grid[h].bodies) {
                    pairs.emplace_back(body, other);
                }
                grid[h].bodies.push_back(body);
            }
        }
    }

    return pairs;
}

// ----------------- Collision Dispatcher -----------------
void PhysicsWorld::handleCollisions() noexcept {
    auto pairs = broadPhasePairs();

    for (auto& p : pairs) {
        Body* a = p.first;
        Body* b = p.second;

        if (a->type == BodyType::Circle && b->type == BodyType::Circle)
            handleCircleCollision(dynamic_cast<CircleBody*>(a), dynamic_cast<CircleBody*>(b));
        else if (a->type == BodyType::Rectangle && b->type == BodyType::Rectangle)
            handleRectangleCollision(dynamic_cast<RigidBody*>(a), dynamic_cast<RigidBody*>(b));
        else {
            if (a->type == BodyType::Circle)
                handleCircleRectangle(dynamic_cast<CircleBody*>(a), dynamic_cast<RigidBody*>(b));
            else
                handleCircleRectangle(dynamic_cast<CircleBody*>(b), dynamic_cast<RigidBody*>(a));
        }
    }
}

// ----------------- Circle-Circle Collision -----------------
void PhysicsWorld::handleCircleCollision(CircleBody* a, CircleBody* b) noexcept {
    Vector2D diff = a->position - b->position;
    float dist = diff.magnitude();
    float minDist = a->radius + b->radius;

    if (dist == 0.0f) { diff = Vector2D(1,0); dist = 1.0f; }

    if (dist < minDist) {
        Vector2D normal = diff.normalized();
        float overlap = minDist - dist;

        a->position += normal * (overlap / 2.0f);
        b->position -= normal * (overlap / 2.0f);

        Vector2D relativeVel = a->velocity - b->velocity;
        float velAlongNormal = Vector2D::dot(relativeVel, normal);
        if (velAlongNormal > 0) return;

        float restitution = std::min(a->restitution, b->restitution);
        float j = -(1 + restitution) * velAlongNormal / (1 / a->mass + 1 / b->mass);
        Vector2D impulse = normal * j;

        a->velocity += impulse * (1 / a->mass);
        b->velocity -= impulse * (1 / b->mass);
    }
}

// ----------------- Circle-Rectangle Collision (with torque) -----------------
void PhysicsWorld::handleCircleRectangle(CircleBody* circle, RigidBody* rect) noexcept {
    Vector2D closestPoint = {
        std::max(rect->position.x, std::min(circle->position.x, rect->position.x + rect->width)),
        std::max(rect->position.y, std::min(circle->position.y, rect->position.y + rect->height))
    };

    Vector2D diff = circle->position - closestPoint;
    float dist = diff.magnitude();
    if (dist >= circle->radius) return;

    Vector2D normal = (dist == 0) ? Vector2D(0, -1) : diff.normalized();
    float penetration = circle->radius - dist;

    float totalMass = circle->mass + rect->mass;
    circle->position += normal * (penetration * (rect->mass / totalMass));
    rect->position -= normal * (penetration * (circle->mass / totalMass));

    Vector2D relativeVel = circle->velocity - rect->velocity;
    float velAlongNormal = Vector2D::dot(relativeVel, normal);
    if (velAlongNormal > 0) return;

    float restitution = std::min(circle->restitution, rect->restitution);
    float j = -(1 + restitution) * velAlongNormal / (1 / circle->mass + 1 / rect->mass);
    Vector2D impulse = normal * j;

    circle->velocity += impulse * (1 / circle->mass);
    rect->velocity -= impulse * (1 / rect->mass);

    // Torque for rotation
    Vector2D contactRel = closestPoint - rect->center();
    float torque = contactRel.x * impulse.y - contactRel.y * impulse.x;
    rect->angularVelocity += torque / rect->inertia;

    // Tangential friction
    Vector2D tangent(-normal.y, normal.x);
    float velAlongTangent = Vector2D::dot(relativeVel, tangent);
    Vector2D frictionImpulse = tangent * (-velAlongTangent * 0.4f / (1 / circle->mass + 1 / rect->mass));
    circle->velocity += Vector2D{-frictionImpulse.x, -frictionImpulse.y} * (1 / circle->mass);
    rect->velocity += frictionImpulse * (1 / rect->mass);

    // Angular damping
    rect->angularVelocity *= 0.98f;
}

// ----------------- Rectangle-Rectangle Collision (per-corner torque) -----------------
bool PhysicsWorld::overlapOnAxis(const std::vector<Vector2D>& aCorners,
                                 const std::vector<Vector2D>& bCorners,
                                 const Vector2D& axis, float& overlap, Vector2D& smallestAxis)
{
    float aMin = std::numeric_limits<float>::max(), aMax = -std::numeric_limits<float>::max();
    float bMin = std::numeric_limits<float>::max(), bMax = -std::numeric_limits<float>::max();

    for (auto& v : aCorners) { float proj = Vector2D::dot(v, axis); if (proj < aMin) aMin = proj; if (proj > aMax) aMax = proj; }
    for (auto& v : bCorners) { float proj = Vector2D::dot(v, axis); if (proj < bMin) bMin = proj; if (proj > bMax) bMax = proj; }

    if (aMax < bMin || bMax < aMin) return false;

    float o = std::min(aMax - bMin, bMax - aMin);
    if (o < overlap) {
        overlap = o;
        smallestAxis = axis;
    }
    return true;
}

void PhysicsWorld::handleRectangleCollision(RigidBody* a, RigidBody* b) noexcept {
    auto aCorners = a->getCorners();
    auto bCorners = b->getCorners();

    std::vector<Vector2D> axes = {
        (aCorners[1] - aCorners[0]).normalized(),
        (aCorners[3] - aCorners[0]).normalized(),
        (bCorners[1] - bCorners[0]).normalized(),
        (bCorners[3] - bCorners[0]).normalized()
    };

    float overlap = std::numeric_limits<float>::max();
    Vector2D smallestAxis;

    for (auto& axis : axes) {
        if (!overlapOnAxis(aCorners, bCorners, axis, overlap, smallestAxis)) return;
    }

    // Move bodies out of penetration
    Vector2D displacement = smallestAxis * (overlap / 2.0f);
    a->position -= displacement;
    b->position += displacement;

    // Per-corner contacts for torque
    std::vector<Vector2D> contacts;
    for (auto& corner : aCorners) {
        Vector2D bMin = b->getAABB().min;
        Vector2D bMax = b->getAABB().max;
        if (corner.x >= bMin.x && corner.x <= bMax.x && corner.y >= bMin.y && corner.y <= bMax.y) contacts.push_back(corner);
    }
    for (auto& corner : bCorners) {
        Vector2D aMin = a->getAABB().min;
        Vector2D aMax = a->getAABB().max;
        if (corner.x >= aMin.x && corner.x <= aMax.x && corner.y >= aMin.y && corner.y <= aMax.y) contacts.push_back(corner);
    }

    if (contacts.empty()) return;

    for (auto& contact : contacts) {
        Vector2D ra = contact - a->center();
        Vector2D rb = contact - b->center();

        Vector2D velA = a->velocity + Vector2D(-a->angularVelocity * ra.y, a->angularVelocity * ra.x);
        Vector2D velB = b->velocity + Vector2D(-b->angularVelocity * rb.y, b->angularVelocity * rb.x);

        Vector2D relativeVel = velB - velA;
        float velAlongNormal = Vector2D::dot(relativeVel, smallestAxis);
        if (velAlongNormal > 0) continue;

        float restitution = std::min(a->restitution, b->restitution);
        float j = -(1 + restitution) * velAlongNormal / (1 / a->mass + 1 / b->mass);
        Vector2D impulse = smallestAxis * j;

        a->velocity -= impulse * (1 / a->mass);
        b->velocity += impulse * (1 / b->mass);

        float torqueA = ra.x * impulse.y - ra.y * impulse.x;
        float torqueB = rb.x * impulse.y - rb.y * impulse.x;

        a->angularVelocity -= torqueA / a->inertia;
        b->angularVelocity += torqueB / b->inertia;

        // Tangential friction
        Vector2D tangent(-smallestAxis.y, smallestAxis.x);
        float relVelTangent = Vector2D::dot(relativeVel, tangent);
        Vector2D frictionImpulse = tangent * (-relVelTangent * std::min(a->friction, b->friction) / (1 / a->mass + 1 / b->mass));

        a->velocity += Vector2D{-frictionImpulse.x, -frictionImpulse.y} * (1 / a->mass);
        b->velocity += frictionImpulse * (1 / b->mass);
    }

    a->angularVelocity *= 0.98f;
    b->angularVelocity *= 0.98f;
}
