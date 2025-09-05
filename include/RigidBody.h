#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#pragma once
#include "Vector2D.h"

class RigidBody {
public:
    Vector2D position;
    Vector2D velocity;
    float mass;

    RigidBody(Vector2D pos, float m = 1.0f);
    void applyForce(const Vector2D& force, float dt);
    void update(float dt, float windowHeight);
};


#endif
