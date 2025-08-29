//
// Created by Batista on 8/29/2025.
//

#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#include <vector>
#include "RigidBody.h"

class PhysicsWorld {
public:
    std::vector<RigidBody*> bodies;
    Vector2D gravity;

    PhysicsWorld(Vector2D gravity = Vector2D(0, 9.8f));

    void addBody(RigidBody* body);
    void step(float dt);
};

#endif
