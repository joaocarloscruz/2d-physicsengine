#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace PhysicsEngine;

namespace {
constexpr int kSteps = 240;

// A pile of every rigid shape plus a jointed pendulum, so stepping exercises
// the polygon outline cache, the cos/sin memo and the solver's point cache.
std::unique_ptr<World> BuildScene() {
    auto world = std::make_unique<World>();
    world->addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    world->addBody(std::make_shared<RigidBody>(Polygon::MakeBox(20, 1), Material{}, Vector2{0, -0.5f}, true));
    const Polygon hexagon({{0.5f, 0}, {0.25f, 0.433f}, {-0.25f, 0.433f}, {-0.5f, 0}, {-0.25f, -0.433f}, {0.25f, -0.433f}});
    for (int i = 0; i < 40; ++i) {
        const Vector2 position{-6.0f + 0.9f * float(i % 12), 0.6f + 1.1f * float(i / 12)};
        std::shared_ptr<RigidBody> body;
        switch (i % 4) {
        case 0: body = std::make_shared<RigidBody>(Polygon::MakeBox(0.8f, 0.6f), Material{}, position); break;
        case 1: body = std::make_shared<RigidBody>(Circle(0.35f), Material{}, position); break;
        case 2: body = std::make_shared<RigidBody>(Capsule(0.25f, 0.2f), Material{}, position); break;
        default: body = std::make_shared<RigidBody>(hexagon, Material{}, position); break;
        }
        body->SetOrientation(0.3f * float(i));
        world->addBody(body);
    }
    auto pivot = std::make_shared<RigidBody>(Circle(0.1f), Material{}, Vector2{8, 6}, true);
    auto bob = std::make_shared<RigidBody>(Polygon::MakeBox(0.6f, 0.6f), Material{}, Vector2{9.5f, 6});
    world->addBody(pivot);
    world->addBody(bob);
    world->addJoint(std::make_shared<RevoluteJoint>(pivot, bob, Vector2{0, 0}, Vector2{-1.5f, 0}));
    return world;
}

// Every body's pose and velocity as raw bits, so the comparison is exact.
std::vector<std::uint32_t> Run(int steps) {
    auto world = BuildScene();
    for (int i = 0; i < steps; ++i) world->step();
    std::vector<std::uint32_t> state;
    for (const auto& body : world->getBodies()) {
        for (float value : {body->position.x, body->position.y, body->orientation,
                            body->velocity.x, body->velocity.y, body->angularVelocity}) {
            std::uint32_t bits;
            std::memcpy(&bits, &value, sizeof bits);
            state.push_back(bits);
        }
    }
    return state;
}
}

TEST_CASE("Separate worlds step concurrently with the same results as one thread", "[world][threads]") {
    const std::vector<std::uint32_t> reference = Run(kSteps);

    constexpr int threadCount = 4;
    std::vector<std::vector<std::uint32_t>> results(threadCount);
    std::vector<std::string> errors(threadCount);
    std::vector<std::thread> threads;
    for (int t = 0; t < threadCount; ++t) {
        threads.emplace_back([t, &results, &errors] {
            try {
                results[t] = Run(kSteps);
            } catch (const std::exception& error) {
                errors[t] = error.what();
            }
        });
    }
    for (auto& thread : threads) thread.join();

    for (int t = 0; t < threadCount; ++t) {
        INFO("thread " << t);
        REQUIRE(errors[t].empty());
        REQUIRE(results[t] == reference);
    }
}
