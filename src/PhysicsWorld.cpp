#include "PhysicsWorld.h"
#include <algorithm>
#include <cmath>
#include <array>
#include <unordered_set>

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

// ---------------- Broad-phase collision ----------------
std::vector<std::pair<Body*, Body*>> PhysicsWorld::broadPhasePairs() noexcept {
    std::unordered_map<int, GridCell> grid;
    std::vector<std::pair<Body*, Body*>> pairs;
    auto hash = [](int x, int y) { return (x << 16) ^ y; };

    struct PairHash {
        size_t operator()(const std::pair<Body*, Body*>& p) const {
            return std::hash<Body*>{}(p.first) ^ std::hash<Body*>{}(p.second);
        }
    };
    std::unordered_set<std::pair<Body*, Body*>, PairHash> addedPairs;

    auto getBodyAABB = [](Body* body) -> AABB {
        if (body->type == BodyType::Rectangle) {
            RigidBody* r = dynamic_cast<RigidBody*>(body);
            Vector2D c = r->center();
            Vector2D he(r->width / 2.0f, r->height / 2.0f);
            float cosA = std::cos(r->angle * 3.14159265f / 180.0f);
            float sinA = std::sin(r->angle * 3.14159265f / 180.0f);

            std::array<Vector2D, 4> corners = {
                Vector2D(-he.x, -he.y), Vector2D(he.x, -he.y),
                Vector2D(he.x, he.y), Vector2D(-he.x, he.y)
            };

            for (auto& corner : corners) {
                float xNew = corner.x * cosA - corner.y * sinA;
                float yNew = corner.x * sinA + corner.y * cosA;
                corner.x = xNew + c.x;
                corner.y = yNew + c.y;
            }

            Vector2D min = corners[0];
            Vector2D max = corners[0];
            for (int i = 1; i < 4; ++i) {
                min.x = std::min(min.x, corners[i].x);
                min.y = std::min(min.y, corners[i].y);
                max.x = std::max(max.x, corners[i].x);
                max.y = std::max(max.y, corners[i].y);
            }
            return { min, max };
        }
        return body->getAABB();
    };

    for (Body* body : bodies) {
        AABB box = getBodyAABB(body);
        int minX = int(box.min.x / CELL_SIZE);
        int minY = int(box.min.y / CELL_SIZE);
        int maxX = int(box.max.x / CELL_SIZE);
        int maxY = int(box.max.y / CELL_SIZE);

        for (int gx = minX; gx <= maxX; ++gx) {
            for (int gy = minY; gy <= maxY; ++gy) {
                int h = hash(gx, gy);
                for (Body* other : grid[h].bodies) {
                    Body* first = std::min(body, other);
                    Body* second = std::max(body, other);
                    if (addedPairs.insert({ first, second }).second) {
                        pairs.emplace_back(first, second);
                    }
                }
                grid[h].bodies.push_back(body);
            }
        }
    }

    return pairs;
}

// ---------------- Collision dispatcher ----------------
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

        const float maxCorrection = 0.2f;
        float correctionAmount = std::min(overlap, maxCorrection);

        a->position += normal * (correctionAmount / 2.0f);
        b->position -= normal * (correctionAmount / 2.0f);


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

// ----------------- Rectangle-Rectangle Collision (SAT + Rotated) -----------------
void PhysicsWorld::handleRectangleCollision(RigidBody* a, RigidBody* b) noexcept {
    // 1. Compute corners of both rectangles in world space
    auto getCorners = [](RigidBody* r) -> std::array<Vector2D, 4> {
        Vector2D c = r->center();
        Vector2D he(r->width / 2.0f, r->height / 2.0f);
        float cosA = std::cos(r->angle * 3.14159265f / 180.0f);
        float sinA = std::sin(r->angle * 3.14159265f / 180.0f);

        std::array<Vector2D, 4> corners = {
            Vector2D(-he.x, -he.y), Vector2D(he.x, -he.y),
            Vector2D(he.x, he.y),   Vector2D(-he.x, he.y)
        };

        for (auto& corner : corners) {
            float xNew = corner.x * cosA - corner.y * sinA;
            float yNew = corner.x * sinA + corner.y * cosA;
            corner.x = xNew + c.x;
            corner.y = yNew + c.y;
        }
        return corners;
    };

    auto cornersA = getCorners(a);
    auto cornersB = getCorners(b);

    // 2. Build axes (normals) to test
    auto getAxes = [](const std::array<Vector2D, 4>& corners) -> std::array<Vector2D, 2> {
        std::array<Vector2D, 2> axes;
        axes[0] = (corners[1] - corners[0]).normalized();
        axes[0] = Vector2D(-axes[0].y, axes[0].x); // normal
        axes[1] = (corners[3] - corners[0]).normalized();
        axes[1] = Vector2D(-axes[1].y, axes[1].x);
        return axes;
    };

    std::array<Vector2D, 2> axesA = getAxes(cornersA);
    std::array<Vector2D, 2> axesB = getAxes(cornersB);

    std::vector<Vector2D> allAxes = { axesA[0], axesA[1], axesB[0], axesB[1] };

    // 3. SAT test
    float minOverlap = 1e20f;
    Vector2D smallestAxis;

    auto projectOntoAxis = [](const std::array<Vector2D, 4>& corners, const Vector2D& axis) {
        float minP = Vector2D::dot(corners[0], axis);
        float maxP = minP;
        for (int i = 1; i < 4; ++i) {
            float p = Vector2D::dot(corners[i], axis);
            if (p < minP) minP = p;
            if (p > maxP) maxP = p;
        }
        return std::make_pair(minP, maxP);
    };

    for (const auto& axis : allAxes) {
        auto [minA, maxA] = projectOntoAxis(cornersA, axis);
        auto [minB, maxB] = projectOntoAxis(cornersB, axis);

        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap <= 0.0f) return; // no collision
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;
        }
    }

    // 4. Positional correction
    Vector2D normal = smallestAxis;
    Vector2D delta = b->center() - a->center();
    if (Vector2D::dot(delta, normal) < 0) normal = -normal;

    
    const float maxCorrection = 0.2f; // max distance to correct per frame
    float correctionAmount = std::min(minOverlap, maxCorrection);

    float totalMass = a->mass + b->mass;
    a->position -= normal * (correctionAmount * (b->mass / totalMass));
    b->position += normal * (correctionAmount * (a->mass / totalMass));


    // 5. Relative velocity & impulse
    Vector2D contactPoint = (a->center() + b->center()) * 0.5f;
    Vector2D rA = contactPoint - a->center();
    Vector2D rB = contactPoint - b->center();

    Vector2D velA = a->velocity + Vector2D(-a->angularVelocity*rA.y, a->angularVelocity*rA.x);
    Vector2D velB = b->velocity + Vector2D(-b->angularVelocity*rB.y, b->angularVelocity*rB.x);
    Vector2D relativeVel = velA - velB;

    float velAlongNormal = Vector2D::dot(relativeVel, normal);
    if (velAlongNormal > 0) return;

    float restitution = std::min(a->restitution, b->restitution);
    float j = -(1 + restitution) * velAlongNormal / (1 / a->mass + 1 / b->mass);
    Vector2D impulse = normal * j;

    a->velocity += impulse * (1 / a->mass);
    b->velocity -= impulse * (1 / b->mass);

    applyTorque(a, impulse, contactPoint);
    applyTorque(b, -impulse, contactPoint);

    // 6. Optional friction
    Vector2D tangent(-normal.y, normal.x);
    float velAlongTangent = Vector2D::dot(relativeVel, tangent);
    float friction = 0.4f;
    Vector2D frictionImpulse = tangent * (-velAlongTangent * friction / (1 / a->mass + 1 / b->mass));
    a->velocity += frictionImpulse * (1 / a->mass);
    b->velocity -= frictionImpulse * (1 / b->mass);
}


// ----------------- Circle-Rectangle Collision -----------------
void PhysicsWorld::handleCircleRectangle(CircleBody* circle, RigidBody* rect) noexcept {
    // 1. Compute rectangle corners
    auto getCorners = [](RigidBody* r) -> std::array<Vector2D, 4> {
        Vector2D c = r->center();
        Vector2D he(r->width / 2.0f, r->height / 2.0f);
        float cosA = std::cos(r->angle * 3.14159265f / 180.0f);
        float sinA = std::sin(r->angle * 3.14159265f / 180.0f);

        std::array<Vector2D, 4> corners = {
            Vector2D(-he.x, -he.y), Vector2D(he.x, -he.y),
            Vector2D(he.x, he.y),   Vector2D(-he.x, he.y)
        };

        for (auto& corner : corners) {
            float xNew = corner.x * cosA - corner.y * sinA;
            float yNew = corner.x * sinA + corner.y * cosA;
            corner.x = xNew + c.x;
            corner.y = yNew + c.y;
        }
        return corners;
    };

    std::array<Vector2D, 4> corners = getCorners(rect);

    // 2. Find closest point on rectangle to circle
    auto closestPointOnRect = [&](const Vector2D& p) {
        Vector2D closest = corners[0];
        float minDist2 = (p - corners[0]).magnitudeSquared();

        // Test edges
        for (int i = 0; i < 4; ++i) {
            Vector2D a = corners[i];
            Vector2D b = corners[(i+1)%4];
            Vector2D ab = b - a;
            float t = Vector2D::dot(p - a, ab) / Vector2D::dot(ab, ab);
            t = std::clamp(t, 0.0f, 1.0f);
            Vector2D proj = a + ab * t;
            float dist2 = (p - proj).magnitudeSquared();
            if (dist2 < minDist2) {
                minDist2 = dist2;
                closest = proj;
            }
        }
        return closest;
    };

    Vector2D closestPoint = closestPointOnRect(circle->position);
    Vector2D diff = circle->position - closestPoint;
    float dist = diff.magnitude();

    if (dist < circle->radius) {
        // 3. Collision normal and penetration
        Vector2D normal = (dist == 0.0f) ? Vector2D(0, -1) : diff.normalized();
        float penetration = circle->radius - dist;

        const float maxCorrection = 0.2f;
        float correctionAmount = std::min(penetration, maxCorrection);

        float totalMass = circle->mass + rect->mass;
        circle->position += normal * (correctionAmount * (rect->mass / totalMass));
        rect->position -= normal * (correctionAmount * (circle->mass / totalMass));


        // 4. Relative velocity at contact
        Vector2D r = closestPoint - rect->center();
        Vector2D velCircle = circle->velocity;
        Vector2D velRect = rect->velocity + Vector2D(-rect->angularVelocity*r.y, rect->angularVelocity*r.x);
        Vector2D relativeVel = velCircle - velRect;

        float velAlongNormal = Vector2D::dot(relativeVel, normal);
        if (velAlongNormal > 0) return;

        // 5. Resolve impulse
        float restitution = std::min(circle->restitution, rect->restitution);
        float j = -(1 + restitution) * velAlongNormal / (1 / circle->mass + 1 / rect->mass);
        Vector2D impulse = normal * j;

        circle->velocity += impulse * (1 / circle->mass);
        rect->velocity -= impulse * (1 / rect->mass);
        applyTorque(rect, -impulse, closestPoint);

        // 6. Friction
        Vector2D tangent(-normal.y, normal.x);
        float velAlongTangent = Vector2D::dot(relativeVel, tangent);
        float friction = 0.4f;
        Vector2D frictionImpulse = tangent * (-velAlongTangent * friction / (1 / circle->mass + 1 / rect->mass));
        circle->velocity += frictionImpulse * (1 / circle->mass);
        rect->velocity -= frictionImpulse * (1 / rect->mass);
    }
}

// ----------------- Apply Torque -----------------
void PhysicsWorld::applyTorque(RigidBody* rect, const Vector2D& force, const Vector2D& contactPoint) noexcept {
    Vector2D r = contactPoint - rect->center();
    float torque = r.x * force.y - r.y * force.x; // 2D cross product (scalar)
    rect->applyTorque(torque, 1.0f); // dt multiplied inside applyTorque
}
