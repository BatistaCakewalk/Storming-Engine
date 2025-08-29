//
// Created by Batista on 8/29/2025.
//

#include "PhysicsWorld.h"

PhysicsWorld::PhysicsWorld(Vector2D gravity) : gravity(gravity) {}

void PhysicsWorld::addBody(RigidBody* body) {
    bodies.push_back(body);
}

void PhysicsWorld::step(float dt) const {
    for (auto* body : bodies) {
        float floorY = 0.0f;
        body->applyForce(gravity * body->mass); // Grav force
        body->update(dt, floorY); // pass floorY for collision
    }
}
