// The legacy step order (enableSpeculativeContacts = false) detects contacts
// once, after integrating positions, and then corrects positions on those
// contacts alone. A body squeezed by a heavy neighbour could be pushed into,
// and through, a static wall it was not touching when contacts were detected.
#include "catch_amalgamated.hpp"

#include "physics/physics.h"

#include <memory>

using namespace PhysicsEngine;

namespace {
struct Squeeze {
    World world;
    std::shared_ptr<RigidBody> floor, ball, box;
};

// A light ball hovering 2 cm above a 20 cm static floor, with a box 100 times
// as heavy overlapping it by `overlap` from above. No gravity: only position
// correction moves the bodies.
std::unique_ptr<Squeeze> MakeSqueeze(float overlap, float ballRadius = 0.1f) {
    auto scene = std::make_unique<Squeeze>();
    SimulationConfig config = scene->world.getSimulationConfig();
    config.enableSpeculativeContacts = false;
    scene->world.setSimulationConfig(config);
    scene->floor = std::make_shared<RigidBody>(Polygon::MakeBox(4, 0.2f), Material{}, Vector2{0, -0.1f}, true);
    const float ballY = 0.02f + ballRadius;
    scene->ball = std::make_shared<RigidBody>(Circle(ballRadius), Material{1, 0, 0.5f, 0.5f}, Vector2{0, ballY});
    scene->box = std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), Material{1, 0, 0.5f, 0.5f},
                                             Vector2{0, ballY + ballRadius + 0.5f - overlap});
    scene->box->SetMass(100 * scene->ball->GetMass());
    scene->world.addBody(scene->floor);
    scene->world.addBody(scene->ball);
    scene->world.addBody(scene->box);
    return scene;
}
}

TEST_CASE("Legacy position correction never pushes a body through a static wall it was not touching",
          "[world][legacy]") {
    for (const float overlap : {0.1f, 0.2f, 0.3f, 0.5f}) {
        for (const float radius : {0.05f, 0.1f, 0.2f}) {
            auto scene = MakeSqueeze(overlap, radius);
            float lowest = scene->ball->position.y;
            for (int step = 0; step < 120; ++step) {
                scene->world.step();
                lowest = std::min(lowest, scene->ball->position.y);
            }
            INFO("overlap " << overlap << ", radius " << radius << ", ball y " << scene->ball->position.y
                            << ", lowest " << lowest << ", box y " << scene->box->position.y);
            // The ball's centre stays above the floor's top face; it may rest
            // on the floor within the touching slop.
            CHECK(lowest > radius - 0.02f);
            CHECK(scene->ball->position.y > radius - 0.02f);
            // The box ends above the ball, not through it.
            CHECK(scene->box->position.y > scene->ball->position.y);
        }
    }
}
