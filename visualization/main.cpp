#include "renderer.h"
#include "physics/core/world.h"
#include <iostream>

using namespace PhysicsEngine;
using namespace PhysicsEngine::Visualization;

int main() {
    std::cout << "2D Physics Engine Visualization" << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << "Starting..." << std::endl;
    
    // Create the physics world
    World world;
    
    // Create and run the renderer
    Renderer renderer(1280, 720, "2D Physics Engine");
    renderer.setWorld(&world);
    renderer.run();
    
    std::cout << "Shutting down..." << std::endl;
    return 0;
}
