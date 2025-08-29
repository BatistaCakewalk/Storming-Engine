cmake_minimum_required(VERSION 3.15)
project(StormPhysics)

set(CMAKE_CXX_STANDARD 20)

# Add include/ so headers are found easily
include_directories(include)

# Build executable
add_executable(StormPhysics
        src/main.cpp
        src/Vector2D.cpp
        src/RigidBody.cpp
        src/PhysicsWorld.cpp
)
