#include <SFML/Graphics.hpp>
#include "PhysicsWorld.h"

int main() {
    constexpr int windowWidth = 800;
    constexpr int windowHeight = 600;
    constexpr float fixedDt = 1.0f / 60.0f; // fixed physics timestep

    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Storming Engine | BETA BRANCH");
    PhysicsWorld world;

    // Create some test bodies
    CircleBody circle1(Vector2D(200, 50));
    CircleBody circle2(Vector2D(350, 100));
    RigidBody rect1(Vector2D(500, 50), 60, 60);

    world.addBody(&circle1);
    world.addBody(&circle2);
    world.addBody(&rect1);

    // SFML shapes for rendering
    sf::CircleShape shape1(circle1.radius);
    shape1.setFillColor(sf::Color::Cyan);
    sf::CircleShape shape2(circle2.radius);
    shape2.setFillColor(sf::Color::Green);
    sf::RectangleShape shapeRect(sf::Vector2f(rect1.width, rect1.height));
    shapeRect.setFillColor(sf::Color::Red);

    sf::Clock clock;
    float accumulator = 0.0f;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Accumulate delta time
        float dt = clock.restart().asSeconds();
        accumulator += dt;

        // Fixed timestep loop
        while (accumulator >= fixedDt) {
            world.step(fixedDt, windowHeight);
            accumulator -= fixedDt;
        }

        // Update SFML shapes to match physics positions
        shape1.setPosition(circle1.position.x - circle1.radius, circle1.position.y - circle1.radius);
        shape2.setPosition(circle2.position.x - circle2.radius, circle2.position.y - circle2.radius);
        shapeRect.setPosition(rect1.position.x, rect1.position.y);

        // Render
        window.clear(sf::Color::Black);
        window.draw(shape1);
        window.draw(shape2);
        window.draw(shapeRect);
        window.display();
    }

    return 0;
}
