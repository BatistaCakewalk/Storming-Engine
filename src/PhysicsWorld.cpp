#include "PhysicsWorld.h"
#include <cmath>
#include <algorithm>

constexpr float CELL_SIZE = 100.0f;

PhysicsWorld::PhysicsWorld(Vector2D g) : gravity(g) {}

void PhysicsWorld::addBody(Body* body) {
    bodies.push_back(body);
}

void PhysicsWorld::step(float dt, float windowHeight) {
    // Apply forces and update
    for (auto* body : bodies) {
        body->applyForce(gravity * body->mass, dt);
        body->update(dt, windowHeight);
    }

    handleCollisions();
}

// ----------------- Broad-Phase -----------------
std::vector<std::pair<Body*, Body*>> PhysicsWorld::broadPhasePairs() {
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

// ----------------- Collision Handling -----------------
void PhysicsWorld::handleCollisions() {
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
void PhysicsWorld::handleCircleCollision(CircleBody* a, CircleBody* b) {
    Vector2D diff = a->position - b->position;
    float dist = diff.magnitude();
    float minDist = a->radius + b->radius;

    if (dist == 0.0f) {
        diff = Vector2D(1,0);
        dist = 1.0f;
    }

    if (dist < minDist) {
        Vector2D normal = diff.normalized();
        float overlap = minDist - dist;

        a->position += normal * (overlap / 2.0f);
        b->position -= normal * (overlap / 2.0f);

        Vector2D relativeVel = a->velocity - b->velocity;
        float velAlongNormal = relativeVel.x * normal.x + relativeVel.y * normal.y;
        if (velAlongNormal > 0) return;

        float restitution = std::min(a->restitution, b->restitution);
        float j = -(1 + restitution) * velAlongNormal / (1 / a->mass + 1 / b->mass);
        Vector2D impulse = normal * j;

        a->velocity += impulse * (1 / a->mass);
        b->velocity -= impulse * (1 / b->mass);
    }
}

// ----------------- Rectangle-Rectangle Collision -----------------
void PhysicsWorld::handleRectangleCollision(RigidBody* a, RigidBody* b) {
    Vector2D aMin = a->position;
    Vector2D aMax = a->position + Vector2D(a->width, a->height);
    Vector2D bMin = b->position;
    Vector2D bMax = b->position + Vector2D(b->width, b->height);

    if (aMax.x < bMin.x || aMin.x > bMax.x || aMax.y < bMin.y || aMin.y > bMax.y)
        return;

    float overlapX = std::min(aMax.x - bMin.x, bMax.x - aMin.x);
    float overlapY = std::min(aMax.y - bMin.y, bMax.y - aMin.y);

    if (overlapX < overlapY) {
        float dir = (a->position.x < b->position.x) ? -1.0f : 1.0f;
        a->position.x += dir * overlapX / 2;
        b->position.x -= dir * overlapX / 2;
        a->velocity.x *= -a->restitution;
        b->velocity.x *= -b->restitution;
    } else {
        float dir = (a->position.y < b->position.y) ? -1.0f : 1.0f;
        a->position.y += dir * overlapY / 2;
        b->position.y -= dir * overlapY / 2;
        a->velocity.y *= -a->restitution;
        b->velocity.y *= -b->restitution;
    }
}

// ----------------- Circle-Rectangle Collision (REWRITE) -----------------
void PhysicsWorld::handleCircleRectangle(CircleBody* circle, RigidBody* rect) {
    // Find the closest point on rectangle to circle
    Vector2D closestPoint = {
        std::max(rect->position.x, std::min(circle->position.x, rect->position.x + rect->width)),
        std::max(rect->position.y, std::min(circle->position.y, rect->position.y + rect->height))
    };

    Vector2D diff = circle->position - closestPoint;
    float dist = diff.magnitude();

    if (dist < circle->radius) {
        // Avoid division by zero
        Vector2D normal = (dist == 0) ? Vector2D(0, -1) : diff.normalized();
        float penetration = circle->radius - dist;

        // Positional correction
        float totalMass = circle->mass + rect->mass;
        circle->position += normal * (penetration * (rect->mass / totalMass));
        rect->position -= normal * (penetration * (circle->mass / totalMass));

        // Relative velocity
        Vector2D relativeVel = circle->velocity - rect->velocity;
        float velAlongNormal = relativeVel.x * normal.x + relativeVel.y * normal.y;

        if (velAlongNormal > 0) return; // already separating

        // Compute impulse scalar
        float restitution = std::min(circle->restitution, rect->restitution);
        float j = -(1 + restitution) * velAlongNormal / (1 / circle->mass + 1 / rect->mass);

        Vector2D impulse = normal * j;

        // Apply impulse
        circle->velocity += impulse * (1 / circle->mass);
        rect->velocity -= impulse * (1 / rect->mass);

        // Optional: simple friction along tangent
        Vector2D tangent = Vector2D(-normal.y, normal.x);
        float velAlongTangent = relativeVel.x * tangent.x + relativeVel.y * tangent.y;
        float friction = 0.4f; // tweakable
        Vector2D frictionImpulse = tangent * (-velAlongTangent * friction / (1 / circle->mass + 1 / rect->mass));

        circle->velocity += frictionImpulse * (1 / circle->mass);
        rect->velocity -= frictionImpulse * (1 / rect->mass);
    }
}
