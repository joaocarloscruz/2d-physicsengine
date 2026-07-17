#include "catch_amalgamated.hpp"
#include "../include/physics/core/collisions/broad_phase/ibroad_phase.h"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/core/shape.h"
#include "../include/physics/core/simulation_config.h"
#include "../include/physics/core/world.h"

#include <limits>
#include <memory>

using namespace PhysicsEngine;

namespace {
class CountingBroadPhase : public IBroadPhase {
public:
    std::vector<CollisionPair> FindPotentialCollisions(
        const std::vector<RigidBodyPtr>&
    ) override {
        ++calls;
        return {};
    }

    int calls = 0;
};
}

TEST_CASE("SimulationConfig preserves the current numerical defaults", "[simulation_config]") {
    SimulationConfig config;

    REQUIRE(config.fixedTimeStep == Catch::Approx(1.0f / 60.0f));
    REQUIRE(config.maxSubstepsPerAdvance == 8);
    REQUIRE(config.solverIterations == 10);
    REQUIRE(config.positionCorrectionFactor == Catch::Approx(0.8f));
    REQUIRE(config.penetrationSlop == Catch::Approx(0.005f));
    REQUIRE(config.warmStartFactor == Catch::Approx(0.8f));
    REQUIRE(config.enableLinearVelocityLimit);
    REQUIRE(config.maxLinearSpeed == Catch::Approx(200.0f));
    REQUIRE(config.enableAngularVelocityLimit);
    REQUIRE(config.maxAngularSpeed == Catch::Approx(30.0f));
    REQUIRE_NOTHROW(config.Validate());
}

TEST_CASE("SimulationConfig rejects invalid numerical settings", "[simulation_config][validation]") {
    SimulationConfig config;

    SECTION("solver iterations must be positive") {
        config.solverIterations = 0;
        REQUIRE_THROWS_AS(config.Validate(), std::invalid_argument);
    }

    SECTION("position correction factor must be normalized") {
        config.positionCorrectionFactor = 1.1f;
        REQUIRE_THROWS_AS(config.Validate(), std::invalid_argument);
    }

    SECTION("penetration slop must be finite and non-negative") {
        config.penetrationSlop = -0.01f;
        REQUIRE_THROWS_AS(config.Validate(), std::invalid_argument);
        config.penetrationSlop = std::numeric_limits<float>::infinity();
        REQUIRE_THROWS_AS(config.Validate(), std::invalid_argument);
    }

    SECTION("warm-start factor must be normalized") {
        config.warmStartFactor = -0.1f;
        REQUIRE_THROWS_AS(config.Validate(), std::invalid_argument);
    }

    SECTION("enabled velocity limits must be positive and finite") {
        config.maxLinearSpeed = 0.0f;
        REQUIRE_THROWS_AS(config.Validate(), std::invalid_argument);
        config.maxLinearSpeed = 200.0f;
        config.maxAngularSpeed = std::numeric_limits<float>::quiet_NaN();
        REQUIRE_THROWS_AS(config.Validate(), std::invalid_argument);
    }

    SECTION("disabled velocity limits ignore their numeric values") {
        config.enableLinearVelocityLimit = false;
        config.enableAngularVelocityLimit = false;
        config.maxLinearSpeed = 0.0f;
        config.maxAngularSpeed = 0.0f;
        REQUIRE_NOTHROW(config.Validate());
    }
}

TEST_CASE("World uses the configured solver iteration count", "[simulation_config][World]") {
    World world;
    SimulationConfig config = world.getSimulationConfig();
    config.solverIterations = 3;
    world.setSimulationConfig(config);

    auto broadPhase = std::make_unique<CountingBroadPhase>();
    CountingBroadPhase* observer = broadPhase.get();
    world.setBroadPhase(std::move(broadPhase));
    world.step(0.0f);

    REQUIRE(observer->calls == 3);
    REQUIRE(world.getSimulationConfig().solverIterations == 3);
}

TEST_CASE("World velocity limits can be configured or disabled", "[simulation_config][World]") {
    Circle shape(1.0f);
    Material material = {1.0f, 0.0f};

    SECTION("defaults retain the existing safety limits") {
        World world;
        auto body = std::make_shared<RigidBody>(&shape, material);
        body->SetVelocity(Vector2(500.0f, 0.0f));
        body->SetAngularVelocity(100.0f);
        world.addBody(body);
        world.step(0.0f);

        REQUIRE(body->GetVelocity().magnitude() == Catch::Approx(200.0f));
        REQUIRE(body->GetAngularVelocity() == Catch::Approx(30.0f));
    }

    SECTION("limits can be disabled explicitly") {
        World world;
        SimulationConfig config = world.getSimulationConfig();
        config.enableLinearVelocityLimit = false;
        config.enableAngularVelocityLimit = false;
        world.setSimulationConfig(config);

        auto body = std::make_shared<RigidBody>(&shape, material);
        body->SetVelocity(Vector2(500.0f, 0.0f));
        body->SetAngularVelocity(100.0f);
        world.addBody(body);
        world.step(0.0f);

        REQUIRE(body->GetVelocity().magnitude() == Catch::Approx(500.0f));
        REQUIRE(body->GetAngularVelocity() == Catch::Approx(100.0f));
    }
}

TEST_CASE("World uses configured penetration correction settings", "[simulation_config][World]") {
    Circle shape(1.0f);
    Material material = {1.0f, 0.0f};
    World world;
    SimulationConfig config = world.getSimulationConfig();
    config.positionCorrectionFactor = 0.0f;
    config.penetrationSlop = 0.0f;
    world.setSimulationConfig(config);

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
    world.addBody(first);
    world.addBody(second);
    world.step(0.0f);

    REQUIRE(first->GetPosition().x == Catch::Approx(0.0f));
    REQUIRE(second->GetPosition().x == Catch::Approx(1.5f));
    REQUIRE(world.getPersistentContactCount() == 1);
}

TEST_CASE("World rejects invalid configuration without replacing the current one", "[simulation_config][World]") {
    World world;
    SimulationConfig invalid = world.getSimulationConfig();
    invalid.solverIterations = 0;

    REQUIRE_THROWS_AS(world.setSimulationConfig(invalid), std::invalid_argument);
    REQUIRE(world.getSimulationConfig().solverIterations == 10);
}
