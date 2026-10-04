#include "renderer.h"
#include "physics/core/world.h"
#include <iostream>

using namespace PhysicsEngine;
using namespace PhysicsEngine::Visualization;

int main(int argc, char** argv) {
    try {
    const bool smoke = argc > 1 && std::string(argv[1]) == "--smoke-test";
    std::cout << "2D Physics Engine Visualization" << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << "Starting..." << std::endl;
    
    // Create the physics world
    World world;
    
    // Create and run the renderer
    Renderer renderer(1280, 720, "2D Physics Engine", !smoke);
    renderer.setWorld(&world);
    renderer.run(smoke ? 5 : 0, smoke && argc > 2 ? argv[2] : "");
    
    std::cout << "Shutting down..." << std::endl;
    return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
