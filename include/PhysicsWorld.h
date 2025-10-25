#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#pragma once
#include "RigidBody.h"
#include "CircleBody.h"
#include <vector>
#include <unordered_map>
#include <utility>

struct GridCell {
    std::vector<Body*> bodies;
};

class PhysicsWorld {
public:
    std::vector<Body*> bodies;
    Vector2D gravity;

    explicit PhysicsWorld(Vector2D g = Vector2D(0, 500.0f)) noexcept;

    void addBody(Body* body) noexcept;
    void step(float dt, float windowHeight) noexcept;

private:
    void handleCollisions() noexcept;

    static void handleCircleCollision(CircleBody* a, CircleBody* b) noexcept;
    static void handleRectangleCollision(RigidBody* a, RigidBody* b) noexcept;
    static void handleCircleRectangle(CircleBody* circle, RigidBody* rect) noexcept;

    // Helper to apply torque from a force applied at a point
    static void applyTorque(RigidBody* rect, const Vector2D& force, const Vector2D& contactPoint) noexcept;

    std::vector<std::pair<Body*, Body*>> broadPhasePairs() noexcept;

};

#endif
