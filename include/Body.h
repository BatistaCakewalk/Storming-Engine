//
// Created by Batista on 8/29/2025.
//

#ifndef BODY_H
#define BODY_H

#pragma once
#include "Vector2D.h"

class Body {
public:
    Vector2D position;
    Vector2D velocity;
    float mass;

    // Inline constructor
    Body(Vector2D pos, float m = 1.0f) : position(pos), velocity(0, 0), mass(m) {
    }

    // Pure virtual functions for derived bodies
    virtual void applyForce(const Vector2D &force, float dt) = 0;

    virtual void update(float dt, float windowHeight) = 0;

    virtual ~Body() = default;
};

#endif
