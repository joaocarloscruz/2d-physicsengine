// Rigid-body step cost against body count. Each scene drops a grid of unit
// boxes and circles (one in three) into a walled bin, settles it for two
// seconds, then times 60 steps at 60 Hz. The best of five repeats is reported,
// since wall-clock timing on a busy machine only ever reads high.
//
// Usage: rigid_step_benchmark [--quick] [--legacy]
//   --quick   100 bodies, one repeat (the CTest smoke run)
//   --legacy  the legacy step order instead of speculative contacts
#include "physics/physics.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace PhysicsEngine;

namespace {
double TimeScene(int bodies, bool legacy, std::uint32_t& contacts) {
    SimulationConfig config;
    config.enableSpeculativeContacts = !legacy;
    World world(config);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    const float width = 2.0f * std::sqrt(float(bodies)) + 4;
    world.addBody(std::make_shared<RigidBody>(Polygon::MakeBox(width + 4, 2), Material{}, Vector2{0, -1}, true));
    world.addBody(std::make_shared<RigidBody>(Polygon::MakeBox(2, 200), Material{}, Vector2{-width / 2 - 1, 100}, true));
    world.addBody(std::make_shared<RigidBody>(Polygon::MakeBox(2, 200), Material{}, Vector2{width / 2 + 1, 100}, true));
    const int perRow = int(width / 1.1f) - 1;
    for (int i = 0; i < bodies; ++i) {
        const Vector2 position{-width / 2 + 1 + float(i % perRow) * 1.1f, 0.6f + float(i / perRow) * 1.1f};
        if (i % 3 == 0) world.addBody(std::make_shared<RigidBody>(Circle(0.5f), Material{}, position));
        else world.addBody(std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), Material{}, position));
    }
    for (int step = 0; step < 120; ++step) world.step(1.0f / 60);
    constexpr int steps = 60;
    const auto start = std::chrono::steady_clock::now();
    for (int step = 0; step < steps; ++step) world.step(1.0f / 60);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    contacts = world.getLastStepStatistics().resolvedContactCount;
    return std::chrono::duration<double, std::milli>(elapsed).count() / steps;
}
}

int main(int argc, char** argv) {
    bool quick = false, legacy = false;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--quick") quick = true;
        else if (argument == "--legacy") legacy = true;
        else {
            std::fprintf(stderr, "usage: rigid_step_benchmark [--quick] [--legacy]\n");
            return 2;
        }
    }
    const std::vector<int> counts = quick ? std::vector<int>{100} : std::vector<int>{100, 250, 500, 1000, 2000};
    const int repeats = quick ? 1 : 5;
    std::printf("%s step order, best of %d\n", legacy ? "legacy" : "speculative", repeats);
    for (int bodies : counts) {
        double best = 1e300;
        std::uint32_t contacts = 0;
        for (int repeat = 0; repeat < repeats; ++repeat) best = std::min(best, TimeScene(bodies, legacy, contacts));
        std::printf("%5d bodies: %8.3f ms/step  contacts=%u\n", bodies, best, contacts);
    }
    return 0;
}
