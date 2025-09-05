//
// Created by Batista on 8/29/2025.
//

#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#pragma once
#include "RigidBody.h"
#include <vector>

class PhysicsWorld {
public:
    std::vector<RigidBody*> bodies;
    Vector2D gravity;

    PhysicsWorld(Vector2D g = Vector2D(0, 500.0f)); // pixels/sec²
    void addBody(RigidBody* body);
    void step(float dt, float windowHeight);
};


#endif
