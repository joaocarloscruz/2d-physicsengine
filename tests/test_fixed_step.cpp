#include "catch_amalgamated.hpp"

#include "engine.h"
#include "physics/core/fixed_step_runner.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/core/world.h"

#include <limits>
#include <memory>
#include <vector>

using namespace PhysicsEngine;

namespace {
RigidBodyPtr AddMovingBody(World& world, Shape* shape, float speed = 1.0f) {
    Material material{1.0f, 0.0f};
    auto body = std::make_shared<RigidBody>(shape, material);
    body->SetVelocity(Vector2(speed, 0.0f));
    world.addBody(body);
    return body;
}
}

TEST_CASE("World performs one configured fixed step", "[fixed_step][World]") {
    SimulationConfig config;
    config.fixedTimeStep = 0.25f;
    World world(config);
    Circle shape(1.0f);
    const RigidBodyPtr body = AddMovingBody(world, &shape, 4.0f);

    world.step();

    REQUIRE(body->GetPosition().x == Catch::Approx(1.0f));
}

TEST_CASE("FixedStepRunner accumulates arbitrary elapsed time", "[fixed_step]") {
    SimulationConfig config;
    config.fixedTimeStep = 0.1f;
    World world(config);
    Circle shape(1.0f);
    const RigidBodyPtr body = AddMovingBody(world, &shape);
    FixedStepRunner runner(world);

    const FixedStepResult partial = runner.advance(0.04);
    REQUIRE(partial.stepsPerformed == 0);
    REQUIRE(partial.simulatedTime == Catch::Approx(0.0));
    REQUIRE(partial.remainingTime == Catch::Approx(0.04));
    REQUIRE(partial.interpolationAlpha == Catch::Approx(0.4));
    REQUIRE(body->GetPosition().x == Catch::Approx(0.0f));

    const FixedStepResult completed = runner.advance(0.06);
    REQUIRE(completed.stepsPerformed == 1);
    REQUIRE(completed.simulatedTime == Catch::Approx(0.1));
    REQUIRE(completed.remainingTime == Catch::Approx(0.0).margin(1e-8));
    REQUIRE(completed.interpolationAlpha == Catch::Approx(0.0).margin(1e-7));
    REQUIRE(body->GetPosition().x == Catch::Approx(0.1f));
}

TEST_CASE("FixedStepRunner bounds work without discarding backlog", "[fixed_step]") {
    SimulationConfig config;
    config.fixedTimeStep = 0.1f;
    config.maxSubstepsPerAdvance = 2;
    World world(config);
    Circle shape(1.0f);
    const RigidBodyPtr body = AddMovingBody(world, &shape);
    FixedStepRunner runner(world);

    const FixedStepResult first = runner.advance(0.5);
    REQUIRE(first.stepsPerformed == 2);
    REQUIRE(first.remainingTime == Catch::Approx(0.3));
    REQUIRE(runner.getTotalStepCount() == 2);

    const FixedStepResult second = runner.advance(0.0);
    REQUIRE(second.stepsPerformed == 2);
    REQUIRE(second.remainingTime == Catch::Approx(0.1));

    const FixedStepResult third = runner.advance(0.0);
    REQUIRE(third.stepsPerformed == 1);
    REQUIRE(third.remainingTime == Catch::Approx(0.0).margin(1e-7));
    REQUIRE(runner.getTotalStepCount() == 5);
    REQUIRE(body->GetPosition().x == Catch::Approx(0.5f));
}

TEST_CASE("Fixed stepping is invariant to elapsed-time subdivision", "[fixed_step][invariant]") {
    auto simulate = [](const std::vector<double>& frameTimes) {
        SimulationConfig config;
        config.fixedTimeStep = 0.01f;
        config.maxSubstepsPerAdvance = 100;
        World world(config);
        Circle shape(1.0f);
        const RigidBodyPtr body = AddMovingBody(world, &shape, 3.0f);
        FixedStepRunner runner(world);

        for (double frameTime : frameTimes) {
            runner.advance(frameTime);
        }
        return std::pair<float, std::uint64_t>{
            body->GetPosition().x,
            runner.getTotalStepCount()
        };
    };

    const auto singleFrame = simulate({0.6});
    const auto subdivided = simulate({0.07, 0.13, 0.11, 0.29});

    REQUIRE(singleFrame.second == 60);
    REQUIRE(subdivided.second == singleFrame.second);
    REQUIRE(subdivided.first == Catch::Approx(singleFrame.first).margin(1e-6f));
}

TEST_CASE("Fixed stepping rejects invalid time policy and elapsed time", "[fixed_step][validation]") {
    SimulationConfig config;

    SECTION("fixed timestep must be positive and finite") {
        config.fixedTimeStep = 0.0f;
        REQUIRE_THROWS_AS(config.Validate(), std::invalid_argument);
        config.fixedTimeStep = std::numeric_limits<float>::infinity();
        REQUIRE_THROWS_AS(config.Validate(), std::invalid_argument);
    }

    SECTION("maximum substeps must be positive") {
        config.maxSubstepsPerAdvance = 0;
        REQUIRE_THROWS_AS(config.Validate(), std::invalid_argument);
    }

    SECTION("elapsed time must be finite and non-negative") {
        World world;
        FixedStepRunner runner(world);
        REQUIRE_THROWS_AS(runner.advance(-0.01), std::invalid_argument);
        REQUIRE_THROWS_AS(
            runner.advance(std::numeric_limits<double>::infinity()),
            std::invalid_argument
        );
    }

    SECTION("elapsed-time accumulation must remain finite") {
        World world;
        FixedStepRunner runner(world);
        REQUIRE_NOTHROW(runner.advance(std::numeric_limits<double>::max()));
        REQUIRE_THROWS_AS(
            runner.advance(std::numeric_limits<double>::max()),
            std::overflow_error
        );
    }
}

TEST_CASE("Engine exposes fixed-step progress and reset", "[fixed_step][Engine]") {
    Engine engine;
    SimulationConfig config = engine.getSimulationConfig();
    config.fixedTimeStep = 0.1f;
    config.maxSubstepsPerAdvance = 1;
    engine.setSimulationConfig(config);

    const FixedStepResult result = engine.advance(0.25);
    REQUIRE(result.stepsPerformed == 1);
    REQUIRE(engine.getAccumulatedTime() == Catch::Approx(0.15));
    REQUIRE(engine.getTotalStepCount() == 1);

    engine.resetTiming();
    REQUIRE(engine.getAccumulatedTime() == Catch::Approx(0.0));
    REQUIRE(engine.getTotalStepCount() == 0);
}
