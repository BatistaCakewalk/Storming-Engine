#include <iostream>
#include "PhysicsWorld.h"

int main() {
    PhysicsWorld world(Vector2D(0, -9.8f)); // gravity down

    RigidBody box(Vector2D(0, 100), 2.0f); // start at y = 100
    world.addBody(&box);

    for (int i = 0; i < 300; i++) {
        constexpr float dt = 0.016f;
        // 5 second simulation
        world.step(dt);
        std::cout << "Time: " << i*dt
                  << "s, Position: (" << box.position.x
                  << ", " << box.position.y << ")\n";
    }

    return 0;
}
