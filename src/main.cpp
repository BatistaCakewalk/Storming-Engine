#include <SFML/Graphics.hpp>
#include "PhysicsWorld.h"

int main() {
    const int windowWidth = 800;
    const int windowHeight = 600;

    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Storming Engine");

    PhysicsWorld world;
    RigidBody box(Vector2D(400, 50), 1.0f);
    world.addBody(&box);

    sf::RectangleShape shape(sf::Vector2f(50, 50));
    shape.setFillColor(sf::Color::Cyan);

    sf::Clock clock;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float dt = clock.restart().asSeconds();
        world.step(dt, windowHeight);

        shape.setPosition(box.position.x, box.position.y);

        window.clear(sf::Color::Black);
        window.draw(shape);
        window.display();
    }

    return 0;
}
