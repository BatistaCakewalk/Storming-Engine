#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Vector2D.h"

class RigidBody {
public:
    Vector2D position;
    Vector2D velocity;
    Vector2D acceleration;
    float mass;

    RigidBody(Vector2D pos = Vector2D(), float mass = 1.0f);

    void applyForce(const Vector2D& force);
    void update(float dt, float floorY = 0.0f); // Added for collision
};

#endif
