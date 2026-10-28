#include "renderer.h"
#include "physics/core/world.h"
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace PhysicsEngine;
using namespace PhysicsEngine::Visualization;

// Usage: PhysicsVisualization [--scene N] [--smoke-test [capture.png]]
//        [--advance STEPS]
// --scene starts in scene N (1-8) and --advance simulates that many fixed steps
// before the first frame; both are meant for headless captures.
int main(int argc, char** argv) {
    try {
        bool smoke = false;
        std::string capturePath;
        std::size_t scene = 0;
        std::size_t advance = 0;
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--smoke-test") {
                smoke = true;
                if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) capturePath = argv[++i];
            } else if (argument == "--scene" && i + 1 < argc) {
                scene = static_cast<std::size_t>(std::stoul(argv[++i]));
                if (scene == 0) throw std::invalid_argument("Scene numbers start at 1.");
                --scene;
            } else if (argument == "--advance" && i + 1 < argc) {
                advance = static_cast<std::size_t>(std::stoul(argv[++i]));
            }
        }

        std::cout << "2D Physics Engine Visualization" << std::endl;
        std::cout << "================================" << std::endl;
        std::cout << "Starting..." << std::endl;

        // Create the physics world
        World world;

        // Create and run the renderer
        Renderer renderer(1280, 720, "2D Physics Engine", !smoke);
        renderer.setWorld(&world);
        renderer.setInitialScene(scene);
        renderer.setWarmupSteps(advance);
        renderer.run(smoke ? 5 : 0, capturePath);

        std::cout << "Shutting down..." << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
