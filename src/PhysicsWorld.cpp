#include "PhysicsWorld.h"
#include <cmath>

PhysicsWorld::PhysicsWorld(Vector2D g) : gravity(g) {}

void PhysicsWorld::addBody(Body* body) {
    bodies.push_back(body);
}

void PhysicsWorld::step(float dt, float windowHeight) {
    for (auto* body : bodies) {
        body->applyForce(gravity * body->mass, dt);
        body->update(dt, windowHeight);
    }
    handleCollisions();
}

void PhysicsWorld::handleCircleCollision(CircleBody* a, CircleBody* b) {
    Vector2D diff = a->position - b->position;
    float dist = diff.magnitude();
    float minDist = a->radius + b->radius;

    if (dist < minDist && dist > 0) {
        Vector2D normal = diff.normalized();
        float overlap = minDist - dist;

        a->position += normal * (overlap / 2.0f);
        b->position -= normal * (overlap / 2.0f);

        // Simple velocity reflection
        a->velocity -= normal * (2 * (a->velocity.x*normal.x + a->velocity.y*normal.y));
        b->velocity -= normal * (2 * (b->velocity.x*normal.x + b->velocity.y*normal.y));
    }
}

void PhysicsWorld::handleCollisions() {
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i+1; j < bodies.size(); ++j) {
            auto* a = dynamic_cast<CircleBody*>(bodies[i]);
            auto* b = dynamic_cast<CircleBody*>(bodies[j]);
            if (a && b) {
                handleCircleCollision(a, b);
            }
        }
    }
}
