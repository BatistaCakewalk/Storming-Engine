#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#pragma once
#include "Body.h"

class RigidBody : public Body {
public:
    float width, height;

    RigidBody(Vector2D pos, float w = 50, float h = 50, float m = 1.0f);

    void applyForce(const Vector2D &force, float dt) override;

    void update(float dt, float windowHeight) override;
};

#endif
