cmake_minimum_required(VERSION 3.15)
project(StormPhysics)

set(CMAKE_CXX_STANDARD 20)

include_directories(include)

# Point to SFML 2.6.2
set(SFML_DIR "C:/Libraries/SFML-2.6.2/lib/cmake/SFML")

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(StormPhysics
        src/main.cpp
        src/Vector2D.cpp
        src/RigidBody.cpp
        src/PhysicsWorld.cpp
)

target_link_libraries(StormPhysics sfml-graphics sfml-window sfml-system)
