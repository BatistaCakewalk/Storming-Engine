#include <SFML/Graphics.hpp>
#include "PhysicsWorld.h"
#include "Config.h"
#include <string>
#include <filesystem>
#include <stdexcept>

namespace fs = std::filesystem;

int main() {
    constexpr int windowWidth = 800;
    constexpr int windowHeight = 600;
    constexpr float fixedDt = 1.0f / 60.0f;

    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Storming Engine | BETA BRANCH");

    // Apply FPS lock based on config
    if (!unlockFPS)
        window.setFramerateLimit(60);
    else
        window.setFramerateLimit(0);

    PhysicsWorld world;

    // Create test bodies
    CircleBody circle1(Vector2D(200, 50));
    CircleBody circle2(Vector2D(350, 100));
    RigidBody rect1(Vector2D(500, 50), 60, 60);

    world.addBody(&circle1);
    world.addBody(&circle2);
    world.addBody(&rect1);

    // SFML shapes
    sf::CircleShape shape1(circle1.radius);
    shape1.setFillColor(sf::Color::Cyan);
    sf::CircleShape shape2(circle2.radius);
    shape2.setFillColor(sf::Color::Green);
    sf::RectangleShape shapeRect(sf::Vector2f(rect1.width, rect1.height));
    shapeRect.setFillColor(sf::Color::Red);

    // Dragging state
    Body* draggedBody = nullptr;
    Vector2D dragOffset(0, 0);
    float dragStiffness = 1000.0f;
    float dragDamping = 20.0f;

    // FPS text
    sf::Font font;
    fs::path fontPath = fs::current_path() / "assets" / "Roboto-Bold.ttf";
    if (!font.loadFromFile(fontPath.string())) {
        throw std::runtime_error("Failed to load font from " + fontPath.string());
    }

    sf::Text fpsText;
    fpsText.setFont(font);
    fpsText.setCharacterSize(20);
    fpsText.setFillColor(sf::Color::White);
    fpsText.setPosition(10.f, 10.f);

    sf::Clock clock;      // for delta time
    sf::Clock fpsClock;   // for FPS display
    float accumulator = 0.0f;
    float fps = 0.0f;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            // Dragging
            if (allowDragging) {
                if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                    Vector2D mouseVec(mousePos.x, mousePos.y);

                    for (auto* body : world.bodies) {
                        if (body->type == BodyType::Circle) {
                            CircleBody* c = static_cast<CircleBody*>(body);
                            if ((c->position - mouseVec).magnitude() <= c->radius) {
                                draggedBody = body;
                                dragOffset = c->position - mouseVec;
                                break;
                            }
                        } else {
                            RigidBody* r = static_cast<RigidBody*>(body);
                            if (mouseVec.x >= r->position.x && mouseVec.x <= r->position.x + r->width &&
                                mouseVec.y >= r->position.y && mouseVec.y <= r->position.y + r->height) {
                                draggedBody = body;
                                dragOffset = r->position - mouseVec;
                                break;
                            }
                        }
                    }
                }

                if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
                    draggedBody = nullptr;
                }
            }
        }

        float dt = clock.restart().asSeconds();

        if (unlockFPS) {
            // variable timestep
            if (allowDragging && draggedBody) {
                sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                Vector2D mouseVec(mousePos.x, mousePos.y);
                Vector2D target = mouseVec + dragOffset;

                Vector2D displacement = target - draggedBody->position;
                Vector2D dampingForce = draggedBody->velocity * dragDamping;
                Vector2D springForce = displacement * dragStiffness - dampingForce;

                draggedBody->applyForce(springForce, dt);
            }
            world.step(dt, windowHeight);
        } else {
            // fixed timestep
            accumulator += dt;
            while (accumulator >= fixedDt) {
                if (allowDragging && draggedBody) {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                    Vector2D mouseVec(mousePos.x, mousePos.y);
                    Vector2D target = mouseVec + dragOffset;

                    Vector2D displacement = target - draggedBody->position;
                    Vector2D dampingForce = draggedBody->velocity * dragDamping;
                    Vector2D springForce = displacement * dragStiffness - dampingForce;

                    draggedBody->applyForce(springForce, fixedDt);
                }
                world.step(fixedDt, windowHeight);
                accumulator -= fixedDt;
            }
        }

        // update SFML shapes
        shape1.setPosition(circle1.position.x - circle1.radius, circle1.position.y - circle1.radius);
        shape2.setPosition(circle2.position.x - circle2.radius, circle2.position.y - circle2.radius);
        shapeRect.setPosition(rect1.position.x, rect1.position.y);

        // update FPS
        fps = 1.0f / fpsClock.restart().asSeconds();
        fpsText.setString("FPS: " + std::to_string(int(fps)));

        // render
        window.clear(sf::Color::Black);
        window.draw(shape1);
        window.draw(shape2);
        window.draw(shapeRect);

        if (showFPS) {
            window.draw(fpsText);   // draw FPS only if enabled
        }

        window.display();
    }

    return 0;
}
