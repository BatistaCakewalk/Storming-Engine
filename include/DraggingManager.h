#ifndef DRAGGINGMANAGER_H
#define DRAGGINGMANAGER_H

#pragma once
#include <SFML/Graphics.hpp>
#include "PhysicsWorld.h"
#include "Config.h"

class DraggingManager {
private:
    Body* draggedBody;
    Vector2D dragOffset;

    float stiffness;
    float damping;

public:
    explicit DraggingManager()
        : draggedBody(nullptr),
          dragOffset(OffsetDragging),
          stiffness(StiffnessDragging),
          damping(DampingDragging) {}

    // Call every frame to process input and apply dragging
    void update(sf::RenderWindow& window, PhysicsWorld& world, float dt) noexcept;

private:
    Body* getBodyUnderMouse(const PhysicsWorld& world, const Vector2D& mouse) const noexcept;
    void applyDrag(Body* body, const Vector2D& mouse, float dt) const noexcept;
};

#endif
