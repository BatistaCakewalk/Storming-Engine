#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#pragma once
#include "RigidBody.h"
#include "CircleBody.h"
#include <vector>

class PhysicsWorld {
public:
    std::vector<Body*> bodies;
    Vector2D gravity;

    PhysicsWorld(Vector2D g = Vector2D(0, 500.0f));

    void addBody(Body* body);
    void step(float dt, float windowHeight);

private:
    void handleCircleCollision(CircleBody* a, CircleBody* b);
    void handleCollisions();
};

#endif
