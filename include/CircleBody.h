#ifndef CIRCLEBODY_H
#define CIRCLEBODY_H

#pragma once
#include "Body.h"

class CircleBody : public Body {
public:
    float radius;

    CircleBody(Vector2D pos, float r = 25, float m = 1.0f);

    void applyForce(const Vector2D& force, float dt) override;
    void update(float dt, float windowHeight) override;
};
#endif
