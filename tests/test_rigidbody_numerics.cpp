#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace PhysicsEngine;

TEST_CASE("Rigid load accumulation rejects overflow without changing loads", "[rigid-numerics]") {
    RigidBody body(Circle(1), Material{});
    const float large = std::numeric_limits<float>::max();
    body.ApplyForce({large, 2});
    REQUIRE_THROWS_AS(body.ApplyForce({large, 3}), std::overflow_error);
    REQUIRE(body.GetForce().x == large);
    REQUIRE(body.GetForce().y == 2);
    body.ApplyTorque(large);
    REQUIRE_THROWS_AS(body.ApplyTorque(large), std::overflow_error);
    REQUIRE(body.GetTorque() == large);
}

TEST_CASE("Rigid impulse stages linear and angular updates together", "[rigid-numerics]") {
    RigidBody body(Circle(1), Material{});
    body.SetMass(1);
    body.SetVelocity({2, 3});
    body.SetAngularVelocity(4);
    REQUIRE_THROWS_AS(body.ApplyImpulse({0, 1e30f}, {1e30f, 0}), std::overflow_error);
    REQUIRE(body.GetVelocity().x == 2);
    REQUIRE(body.GetVelocity().y == 3);
    REQUIRE(body.GetAngularVelocity() == 4);
    body.SetMass(0.01f);
    REQUIRE_THROWS_AS(body.ApplyImpulse({1e38f, 0}, {}), std::overflow_error);
    REQUIRE(body.GetVelocity().x == 2);
    REQUIRE(body.GetVelocity().y == 3);
    REQUIRE(body.GetAngularVelocity() == 4);
}

TEST_CASE("Rigid integration preserves state and loads on position overflow", "[rigid-numerics]") {
    RigidBody body(Circle(1), Material{}, {1, 2});
    const float large = std::numeric_limits<float>::max();
    body.SetVelocity({large, 3});
    body.SetOrientation(0.5f);
    body.SetAngularVelocity(4);
    body.ApplyForce({1, 2});
    body.ApplyTorque(3);
    REQUIRE_THROWS_AS(body.Integrate(2), std::overflow_error);
    REQUIRE(body.GetPosition().x == 1);
    REQUIRE(body.GetPosition().y == 2);
    REQUIRE(body.GetVelocity().x == large);
    REQUIRE(body.GetVelocity().y == 3);
    REQUIRE(body.GetOrientation() == 0.5f);
    REQUIRE(body.GetAngularVelocity() == 4);
    REQUIRE(body.GetForce().x == 1);
    REQUIRE(body.GetForce().y == 2);
    REQUIRE(body.GetTorque() == 3);
}

TEST_CASE("Rigid speed caps preserve direction for enormous finite speeds", "[rigid-numerics]") {
    RigidBody body(Circle(1), Material{});
    body.SetVelocity({3e30f, 4e30f});
    body.Integrate(1e-5f);
    REQUIRE(body.GetVelocity().x == Catch::Approx(120).epsilon(1e-6));
    REQUIRE(body.GetVelocity().y == Catch::Approx(160).epsilon(1e-6));
    REQUIRE(body.GetPosition().x == Catch::Approx(3e25).epsilon(1e-6));
    REQUIRE(body.GetPosition().y == Catch::Approx(4e25).epsilon(1e-6));
}

TEST_CASE("Rigid integration retains finite tiny-step results with large accelerations", "[rigid-numerics]") {
    RigidBody body(Circle(1), Material{});
    body.SetMass(1e-20f);
    body.ApplyForce({1e30f, 0});
    body.ApplyTorque(1e30f);
    const float dt = 1e-30f;
    const double expectedX = 0.5 * body.force.x * double(body.inverseMass) * dt * dt;
    const double expectedAngle = 0.5 * body.torque * double(body.inverseInertia) * dt * dt;
    body.Integrate(dt);
    REQUIRE(body.GetPosition().x == Catch::Approx(expectedX).epsilon(1e-6));
    REQUIRE(body.GetOrientation() == Catch::Approx(expectedAngle).epsilon(1e-6));
    REQUIRE(body.GetVelocity().x == 200);
    REQUIRE(body.GetAngularVelocity() == 30);
    REQUIRE(body.GetForce().x == 0);
    REQUIRE(body.GetTorque() == 0);
}

TEST_CASE("Rigid arithmetic rejects corrupted legacy fields when consumed", "[rigid-numerics]") {
    RigidBody body(Circle(1), Material{});
    const float nan = std::numeric_limits<float>::quiet_NaN();
    SECTION("force") {
        body.force.x = nan;
        REQUIRE_THROWS_AS(body.ApplyForce({1, 0}), std::invalid_argument);
        REQUIRE_THROWS_AS(body.Integrate(0.1f), std::invalid_argument);
    }
    SECTION("velocity") {
        body.velocity.y = nan;
        REQUIRE_THROWS_AS(body.ApplyImpulse({1, 0}, {}), std::invalid_argument);
        REQUIRE_THROWS_AS(body.Integrate(0.1f), std::invalid_argument);
    }
    SECTION("inverse mass") {
        body.inverseMass = -1;
        REQUIRE_THROWS_AS(body.ApplyImpulse({1, 0}, {}), std::invalid_argument);
        REQUIRE_THROWS_AS(body.Integrate(0.1f), std::invalid_argument);
    }
    SECTION("orientation") {
        body.orientation = nan;
        REQUIRE_THROWS_AS(body.Integrate(0.1f), std::invalid_argument);
    }
}

TEST_CASE("Rejected impulses do not wake sleepers but tiny nonzero loads do", "[rigid-numerics][sleep]") {
    SimulationConfig config;
    config.enableSleeping = true;
    config.sleepTimeThreshold = 0.01f;
    World world(config);
    auto body = std::make_shared<RigidBody>(Circle(1), Material{});
    world.addBody(body);
    world.step(0.02f);
    REQUIRE_FALSE(body->IsAwake());
    REQUIRE_THROWS_AS(body->ApplyImpulse({0, 1e30f}, {1e30f, 0}), std::overflow_error);
    REQUIRE_FALSE(body->IsAwake());
    REQUIRE(body->GetVelocity().x == 0);
    REQUIRE(body->GetVelocity().y == 0);
    SECTION("force") { body->ApplyForce({1e-30f, 0}); }
    SECTION("impulse") { body->ApplyImpulse({1e-30f, 0}, {}); }
    REQUIRE(body->IsAwake());
}
