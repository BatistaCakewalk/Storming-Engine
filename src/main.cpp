#include <SFML/Graphics.hpp>
#include "PhysicsWorld.h"
#include "Config.h"
#include <string>
#include <filesystem>
#include <stdexcept>
#include <unordered_map>

namespace fs = std::filesystem;

// Helper: Get body under mouse
Body* getBodyUnderMouse(const PhysicsWorld& world, const Vector2D& mouse) {
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

// Helper: Apply drag physics
void applyDrag(Body* body, const Vector2D& mouse, const Vector2D& dragOffset, float dt, float stiffness, float damping) {
    Vector2D target = mouse + dragOffset;
    Vector2D displacement = target - body->position;
    Vector2D dampingForce = body->velocity * damping;
    Vector2D springForce = displacement * stiffness - dampingForce;
    body->applyForce(springForce, dt);
}

int main() {
    constexpr int windowWidth = 800;
    constexpr int windowHeight = 600;
    constexpr float fixedDt = 1.0f / 60.0f;

    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Storming Engine | BETA BRANCH");

    // FPS limit
    window.setFramerateLimit(unlockFPS ? 0 : 60);

    PhysicsWorld world;

    // Create test bodies
    CircleBody circle1(Vector2D(200, 50));
    CircleBody circle2(Vector2D(350, 100));
    RigidBody rect1(Vector2D(500, 50), 60, 60);

    world.addBody(&circle1);
    world.addBody(&circle2);
    world.addBody(&rect1);

    // SFML shapes mapping
    std::unordered_map<Body*, sf::Shape*> shapeMap;

    sf::CircleShape* shape1 = new sf::CircleShape(circle1.radius);
    shape1->setFillColor(sf::Color::Cyan);
    shapeMap[&circle1] = shape1;

    sf::CircleShape* shape2 = new sf::CircleShape(circle2.radius);
    shape2->setFillColor(sf::Color::Green);
    shapeMap[&circle2] = shape2;

    sf::RectangleShape* shapeRect = new sf::RectangleShape(sf::Vector2f(rect1.width, rect1.height));
    shapeRect->setFillColor(sf::Color::Red);
    shapeMap[&rect1] = shapeRect;

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

    sf::Clock clock;      // delta time
    float accumulator = 0.0f;
    float fps = 0.0f;
    float fpsAccumulator = 0.0f;
    int fpsFrames = 0;

    while (window.isOpen()) {
        sf::Event event{};
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            if (allowDragging) {
                if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                    Vector2D mouseVec(sf::Mouse::getPosition(window).x, sf::Mouse::getPosition(window).y);
                    draggedBody = getBodyUnderMouse(world, mouseVec);
                    if (draggedBody) {
                        dragOffset = draggedBody->position - mouseVec;
                    }
                }

                if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
                    draggedBody = nullptr;
                }
            }
        }

        float dt = clock.restart().asSeconds();

        if (unlockFPS) {
            if (allowDragging && draggedBody) {
                Vector2D mouseVec(sf::Mouse::getPosition(window).x, sf::Mouse::getPosition(window).y);
                applyDrag(draggedBody, mouseVec, dragOffset, dt, dragStiffness, dragDamping);
            }
            world.step(dt, windowHeight);
        } else {
            accumulator += dt;
            while (accumulator >= fixedDt) {
                if (allowDragging && draggedBody) {
                    Vector2D mouseVec(sf::Mouse::getPosition(window).x, sf::Mouse::getPosition(window).y);
                    applyDrag(draggedBody, mouseVec, dragOffset, fixedDt, dragStiffness, dragDamping);
                }
                world.step(fixedDt, windowHeight);
                accumulator -= fixedDt;
            }
        }

        // Update SFML shapes positions
        for (auto& [body, shape] : shapeMap) {
            if (body->type == BodyType::Circle) {
                auto* c = dynamic_cast<CircleBody*>(body);
                dynamic_cast<sf::CircleShape*>(shape)->setPosition(c->position.x - c->radius, c->position.y - c->radius);
            } else {
                auto* r = dynamic_cast<RigidBody*>(body);
                dynamic_cast<sf::RectangleShape*>(shape)->setPosition(r->position.x, r->position.y);
            }
        }

        // Smooth FPS calculation
        fpsAccumulator += 1.0f / dt;
        fpsFrames++;
        if (fpsFrames >= 10) {
            fps = fpsAccumulator / fpsFrames;
            fpsFrames = 0;
            fpsAccumulator = 0.0f;
        }
        fpsText.setString("FPS: " + std::to_string(int(fps)));

        // Render
        window.clear(sf::Color::Black);
        for (auto& [_, shape] : shapeMap) {
            window.draw(*shape);
        }

        if (showFPS) {
            window.draw(fpsText);
        }

        window.display();
    }

    // Cleanup shapes
    for (auto& [_, shape] : shapeMap) delete shape;

    return 0;
}
