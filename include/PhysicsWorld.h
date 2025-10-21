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

    PhysicsWorld(Vector2D g = Vector2D(0, 500.0f));

    void addBody(Body* body);
    void step(float dt, float windowHeight);

private:
    void handleCollisions();
    void handleCircleCollision(CircleBody* a, CircleBody* b);
    void handleRectangleCollision(RigidBody* a, RigidBody* b);
    void handleCircleRectangle(CircleBody* circle, RigidBody* rect);

    std::vector<std::pair<Body*, Body*>> broadPhasePairs();
};

#endif
