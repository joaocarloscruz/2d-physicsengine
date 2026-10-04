#include "physics/physics.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
    try {
        const std::string prefix = argc > 1 ? argv[1] : "experiment";
        PhysicsEngine::SimulationConfig config;
        config.fixedTimeStep = 1.0f/120;
        config.enableLinearVelocityLimit = false;
        PhysicsEngine::World world(config);
        auto ball = std::make_shared<PhysicsEngine::RigidBody>(PhysicsEngine::Circle(0.2f),
            PhysicsEngine::Material{1, 0.8f}, PhysicsEngine::Vector2(0, 3));
        ball->SetCcdEnabled(true);
        world.addBody(ball);
        world.addBody(std::make_shared<PhysicsEngine::RigidBody>(PhysicsEngine::Polygon::MakeBox(10, 0.2f),
            PhysicsEngine::Material{1, 0.8f}, PhysicsEngine::Vector2(0, -0.1f), true));
        world.addUniversalForce(std::make_unique<PhysicsEngine::Gravity>(PhysicsEngine::Vector2(0, -9.81f)));
        std::ofstream csv(prefix+".csv"), json(prefix+".json");
        if (!csv || !json) throw std::runtime_error("Cannot open export files.");
        json << "[\n";
        for (int i=0; i<=240; ++i) {
            if (i) world.step();
            const double time = i*static_cast<double>(config.fixedTimeStep);
            std::string rows = PhysicsEngine::ExportWorldCsv(world, time);
            csv << (i ? rows.substr(rows.find('\n')+1) : rows);
            if (i) json << ",";
            json << PhysicsEngine::ExportWorldJson(world, time);
        }
        json << "]\n";
        if (!csv || !json) throw std::runtime_error("Failed to write exports.");
        std::cout << "Replayed 240 fixed steps; wrote " << prefix << ".csv and " << prefix << ".json\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
