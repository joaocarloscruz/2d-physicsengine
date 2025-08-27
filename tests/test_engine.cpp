#include "catch_amalgamated.hpp"
#include "../include/physics_engine.h"
#include "../include/physics/core/forces/gravity.h"

TEST_CASE("Engine Simulation", "[Engine]") {
    SECTION("A body with gravity applied falls correctly") {
        PhysicsEngine engine;

        Circle circle(1.0f);
        RigidBody body(&circle, 1.0f, {0, 10});

        engine.addBody(&body);
        engine.addForce(&body, std::make_unique<Gravity>(Vector2(0.0f, -9.8f)));

        const int max_frames = 500;
        const float dt = 0.016f;
        for (int i = 0; i < max_frames; ++i) {
            engine.step(dt);
        }

        const auto& bodies = engine.getBodies();
        const auto& final_pos = bodies[0]->GetPosition();

        // This value is the expected result after 500 frames of simulation
        // with dt=0.016 and g=-9.8, as observed from a previous run.
        REQUIRE(final_pos.y == Catch::Approx(-304.226));
    }
}
