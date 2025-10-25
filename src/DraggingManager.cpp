#include "DraggingManager.h"
#include <algorithm>

// Check if mouse is over a body
Body* DraggingManager::getBodyUnderMouse(const PhysicsWorld& world, const Vector2D& mouse) const noexcept {
    for (auto* body : world.bodies) {
        if (body->type == BodyType::Circle) {
            auto* c = dynamic_cast<CircleBody*>(body);
            if ((c->position - mouse).magnitude() <= c->radius)
                return body;
        } else {
            auto* r = dynamic_cast<RigidBody*>(body);
            if (mouse.x >= r->position.x && mouse.x <= r->position.x + r->width &&
                mouse.y >= r->position.y && mouse.y <= r->position.y + r->height)
                return body;
        }
    }
    return nullptr;
}

// Apply spring-damper drag
void DraggingManager::applyDrag(Body* body, const Vector2D& mouse, float dt) const noexcept {
    Vector2D target = mouse + dragOffset;
    Vector2D displacement = target - body->position;
    Vector2D dampingForce = body->velocity * damping;
    Vector2D springForce = displacement * stiffness - dampingForce;
    body->applyForce(springForce, dt);
}

// Call this once per frame
void DraggingManager::update(sf::RenderWindow& window, PhysicsWorld& world, float dt) noexcept {
    if (!allowDragging) return;

    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
    Vector2D mouseVec(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));

    // Mouse press
    if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
        if (!draggedBody) {
            draggedBody = getBodyUnderMouse(world, mouseVec);
            if (draggedBody) {
                dragOffset = draggedBody->position - mouseVec;
            }
        }
    } else {
        draggedBody = nullptr; // release body
    }

    // Apply drag
    if (draggedBody) {
        applyDrag(draggedBody, mouseVec, dt);
    }
}
