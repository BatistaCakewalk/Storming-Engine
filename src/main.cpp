#include <SFML/Graphics.hpp>
#include "PhysicsWorld.h"
#include "Config.h"
#include "DraggingManager.h"
#include <string>
#include <filesystem>
#include <stdexcept>
#include <unordered_map>

namespace fs = std::filesystem;

int main() {
    // Window settings
    int windowWidth = windowWidthSize;
    int windowHeight = windowHeightSize;

    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), windowName);

    // FPS limit
    window.setFramerateLimit(unlockFPS ? 0 : targetFPS);

    // Physics world
    PhysicsWorld world;

    // Create test bodies
    CircleBody circle1(Vector2D(200, 50));
    CircleBody circle2(Vector2D(350, 100));
    RigidBody rect1(Vector2D(500, 50), 60, 60);

    world.addBody(&circle1);
    world.addBody(&circle2);
    world.addBody(&rect1);
    

    // Map bodies to SFML shapes
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

    // Dragging manager
    DraggingManager draggingManager;

    // FPS text setup
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

    sf::Clock clock;
    float accumulator = 0.0f;
    float fps = 0.0f;
    float fpsAccumulator = 0.0f;
    int fpsFrames = 0;

    constexpr float fixedDt = 1.0f / 60.0f;

    // Main loop
    while (window.isOpen()) {
        sf::Event event{};
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float dt = clock.restart().asSeconds();

        // Update dragging and apply forces
        draggingManager.update(window, world, dt);

        // Step physics
        if (unlockFPS) {
            world.step(dt, windowHeight);
        } else {
            accumulator += dt;
            while (accumulator >= fixedDt) {
                world.step(fixedDt, windowHeight);
                accumulator -= fixedDt;
            }
        }

        // Update SFML shape positions
        for (auto& [body, shape] : shapeMap) {
            if (body->type == BodyType::Circle) {
                auto* c = dynamic_cast<CircleBody*>(body);
                dynamic_cast<sf::CircleShape*>(shape)->setPosition(c->position.x - c->radius, c->position.y - c->radius);
            } else {
                auto* r = dynamic_cast<RigidBody*>(body);
                sf::RectangleShape* rectShape = dynamic_cast<sf::RectangleShape*>(shape);
                rectShape->setSize(sf::Vector2f(r->width, r->height));
                rectShape->setOrigin(r->width / 2.0f, r->height / 2.0f); // rotate around center
                rectShape->setPosition(r->center().x, r->center().y);
                rectShape->setRotation(r->angle);
                
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
