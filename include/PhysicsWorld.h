#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#pragma once
#include "RigidBody.h"
#include "CircleBody.h"
#include <vector>
#include <unordered_map>

struct GridCell {
    std::vector<Body*> bodies;
};

class PhysicsWorld {
public:
    std::vector<Body*> bodies;
    Vector2D gravity;

    explicit PhysicsWorld(Vector2D g = Vector2D(0, 500.0f));

    void addBody(Body* body);
    void step(float dt, float windowHeight);

private:
    void handleCollisions();
    static void handleCircleCollision(CircleBody* a, CircleBody* b);
    static void handleRectangleCollision(RigidBody* a, RigidBody* b);
    static void handleCircleRectangle(CircleBody* circle, RigidBody* rect);

    std::vector<std::pair<Body*, Body*>> broadPhasePairs();
};

#endif
