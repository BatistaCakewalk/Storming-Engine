#include <SFML/Graphics.hpp>
#include "PhysicsWorld.h"

int main() {
    const int windowWidth = 800;
    const int windowHeight = 600;

    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Storming Engine");

    PhysicsWorld world;

    CircleBody circle1(Vector2D(200, 50));
    CircleBody circle2(Vector2D(350, 100));
    

    world.addBody(&circle1);
    world.addBody(&circle2);

    sf::CircleShape shape1(circle1.radius);
    shape1.setFillColor(sf::Color::Cyan);
    sf::CircleShape shape2(circle2.radius);
    shape2.setFillColor(sf::Color::Green);
    

    sf::Clock clock;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float dt = clock.restart().asSeconds();
        world.step(dt, windowHeight);

        shape1.setPosition(circle1.position.x - circle1.radius, circle1.position.y - circle1.radius);
        shape2.setPosition(circle2.position.x - circle2.radius, circle2.position.y - circle2.radius);

        window.clear(sf::Color::Black);
        window.draw(shape1);
        window.draw(shape2);
        window.display();
    }

    return 0;
}
