#include "catch_amalgamated.hpp"

#include "engine.h"
#include "physics/core/particles/particle_system.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/core/simulation_statistics.h"
#include "physics/core/world.h"

#include <limits>
#include <memory>

using namespace PhysicsEngine;

TEST_CASE("World reports work performed by one step", "[statistics][World]") {
    SimulationConfig config;
    config.solverIterations = 3;
    World world(config);
    Circle shape(1.0f);
    Material material{1.0f, 0.0f};
    auto first = std::make_shared<RigidBody>(
        &shape,
        material,
        Vector2(0.0f, 0.0f)
    );
    auto second = std::make_shared<RigidBody>(
        &shape,
        material,
        Vector2(1.5f, 0.0f)
    );
    auto staticBody = std::make_shared<RigidBody>(
        &shape,
        material,
        Vector2(20.0f, 0.0f),
        true
    );
    world.addBody(first);
    world.addBody(second);
    world.addBody(staticBody);

    auto particles = std::make_shared<ParticleSystem>();
    particles->addParticle(Vector2(0.0f, 0.0f));
    particles->addParticle(Vector2(1.0f, 0.0f));
    particles->addParticle(Vector2(2.0f, 0.0f));
    world.addParticleSystem(particles);

    world.step(0.0f);

    const SimulationStatistics& statistics = world.getLastStepStatistics();
    REQUIRE(statistics.integratedBodyCount == 2);
    REQUIRE(statistics.integratedParticleCount == 3);
    REQUIRE(statistics.broadPhaseCandidateCount == 1);
    REQUIRE(statistics.narrowPhaseCandidateCount == 1);
    REQUIRE(statistics.resolvedContactCount == 1);
    REQUIRE(statistics.solverIterationCount == 3);
    REQUIRE(statistics.activeContactCount == 1);
    REQUIRE(statistics.fluidIterationCount == 0);
}

TEST_CASE("World statistics reset at the start of each valid step", "[statistics][World]") {
    World world;
    Circle shape(1.0f);
    Material material{1.0f, 0.0f};
    world.addBody(std::make_shared<RigidBody>(&shape, material));
    world.step(0.0f);
    REQUIRE(world.getLastStepStatistics().integratedBodyCount == 1);

    world.clearBodies();
    world.step(0.0f);

    const SimulationStatistics& statistics = world.getLastStepStatistics();
    REQUIRE(statistics.integratedBodyCount == 0);
    REQUIRE(statistics.integratedParticleCount == 0);
    REQUIRE(statistics.broadPhaseCandidateCount == 0);
    REQUIRE(statistics.resolvedContactCount == 0);
    REQUIRE(statistics.solverIterationCount == 10);
    REQUIRE(statistics.activeContactCount == 0);
}

TEST_CASE("Invalid steps preserve the last completed statistics", "[statistics][validation]") {
    World world;
    Circle shape(1.0f);
    Material material{1.0f, 0.0f};
    world.addBody(std::make_shared<RigidBody>(&shape, material));
    world.step(0.0f);
    const SimulationStatistics completed = world.getLastStepStatistics();

    REQUIRE_THROWS_AS(
        world.step(std::numeric_limits<float>::quiet_NaN()),
        std::invalid_argument
    );
    REQUIRE(world.getLastStepStatistics().integratedBodyCount
        == completed.integratedBodyCount);
    REQUIRE(world.getLastStepStatistics().solverIterationCount
        == completed.solverIterationCount);
}

TEST_CASE("Engine exposes the last World step statistics", "[statistics][Engine]") {
    Engine engine;
    Circle shape(1.0f);
    Material material{1.0f, 0.0f};
    engine.addBody(std::make_shared<RigidBody>(&shape, material));

    engine.stepFixed();

    const SimulationStatistics statistics = engine.getLastStepStatistics();
    REQUIRE(statistics.integratedBodyCount == 1);
    REQUIRE(statistics.solverIterationCount == 10);
}
