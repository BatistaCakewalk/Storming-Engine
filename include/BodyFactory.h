#ifndef BODYFACTORY_H
#define BODYFACTORY_H

#pragma once
#include "PhysicsWorld.h"
#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>
#include <unordered_map>

class BodyFactory {
public:
    PhysicsWorld& world;

    // Smart pointers to manage SFML shapes
    std::vector<std::unique_ptr<sf::Shape>> shapes;
    std::unordered_map<Body*, sf::Shape*> bodyShapeMap;

    explicit BodyFactory(PhysicsWorld& w) : world(w) {}

    // Create a circle body and corresponding SFML shape
    Body* createCircle(const Vector2D& pos, float radius = 25, float mass = 1.0f,
                       float restitution = 0.6f, float friction = 0.8f,
                       sf::Color color = sf::Color::Cyan) 
    {
        auto* body = new CircleBody(pos, radius, mass, restitution, friction);
        world.addBody(body);

        auto shape = std::make_unique<sf::CircleShape>(radius);
        shape->setFillColor(color);
        shapes.push_back(std::move(shape));
        bodyShapeMap[body] = shapes.back().get();

        return body;
    }

    // Create a rectangle body and corresponding SFML shape
    Body* createRectangle(const Vector2D& pos, float width = 50, float height = 50, float mass = 1.0f,
                          float restitution = 0.6f, float friction = 0.8f,
                          sf::Color color = sf::Color::Red) 
    {
        auto* body = new RigidBody(pos, width, height, mass, restitution, friction);
        world.addBody(body);

        auto shape = std::make_unique<sf::RectangleShape>(sf::Vector2f(width, height));
        shape->setFillColor(color);
        shapes.push_back(std::move(shape));
        bodyShapeMap[body] = shapes.back().get();

        return body;
    }

    // Update all SFML shapes to match body positions
    void updateShapes() {
        for (auto& [body, shape] : bodyShapeMap) {
            if (body->type == BodyType::Circle) {
                auto* c = dynamic_cast<CircleBody*>(body);
                dynamic_cast<sf::CircleShape*>(shape)->setPosition(c->position.x - c->radius, c->position.y - c->radius);
            } else {
                auto* r = dynamic_cast<RigidBody*>(body);
                dynamic_cast<sf::RectangleShape*>(shape)->setPosition(r->position.x, r->position.y);
            }
        }
    }
};

#endif
