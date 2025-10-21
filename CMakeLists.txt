cmake_minimum_required(VERSION 3.15)
project(StormPhysics)

set(CMAKE_CXX_STANDARD 20)

# Include your headers
include_directories(include)

# Point to SFML 2.6.2
set(SFML_DIR "C:/Libraries/SFML-2.6.2/lib/cmake/SFML")

# Find SFML components
find_package(SFML 2.6 COMPONENTS graphics window system REQUIRED)

# Executables
add_executable(StormPhysics
        src/main.cpp
        src/Vector2D.cpp
        src/RigidBody.cpp
        src/PhysicsWorld.cpp
        src/CircleBody.cpp
        include/Config.h
)


# Link SFML libraries
target_link_libraries(StormPhysics sfml-graphics sfml-window sfml-system)
