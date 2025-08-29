//
// Created by Batista on 8/29/2025.
//

#include "PhysicsWorld.h"

PhysicsWorld::PhysicsWorld(Vector2D gravity) : gravity(gravity) {}

void PhysicsWorld::addBody(RigidBody* body) {
    bodies.push_back(body);
}

void PhysicsWorld::step(float dt) {
    for (auto* body : bodies) {
        body->applyForce(gravity * body->mass); // gravity force
        body->update(dt);
    }
}
