//
// Created by Batista on 8/29/2025.
//

#include "PhysicsWorld.h"

PhysicsWorld::PhysicsWorld(Vector2D g) : gravity(g) {}

void PhysicsWorld::addBody(RigidBody* body) {
    bodies.push_back(body);
}

void PhysicsWorld::step(float dt, float windowHeight) {
    for (auto* body : bodies) {
        body->applyForce(gravity * body->mass, dt);
        body->update(dt, windowHeight);
    }
}
