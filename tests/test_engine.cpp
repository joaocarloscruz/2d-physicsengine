/*
#include "catch_amalgamated.hpp"
#include "../include/engine.h"
#include "../include/physics/core/forces/gravity.h"
#include <memory>

using namespace PhysicsEngine;

TEST_CASE("Engine Simulation", "[Engine]") {
    SECTION("A body with gravity applied falls correctly") {
        Engine engine;

        Circle circle(1.0f);
        Material material = {1.0f, 0.5f};
        auto body = std::make_shared<RigidBody>(&circle, material, Vector2(0, 10));

        engine.addBody(body);
        engine.addUniversalForce(std::make_unique<Gravity>(Vector2(0.0f, -9.8f)));

        const int max_frames = 500;
        const float dt = 0.016f;
        for (int i = 0; i < max_frames; ++i) {
            engine.step(dt);
        }

        const auto& bodies = engine.getBodies();
        const auto& final_pos = bodies[0]->GetPosition();

        // This value is the expected result after 500 frames of simulation
        // with dt=0.016 and g=-9.8, as observed from a previous run.
        // the final result should be approximately -303.6
        REQUIRE(final_pos.y == Catch::Approx(-303.6));
    }
}
*/