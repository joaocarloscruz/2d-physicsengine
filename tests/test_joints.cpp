#include "catch_amalgamated.hpp"
#include "physics/core/world.h"
#include "physics/core/forces/gravity.h"
#include <limits>

using namespace PhysicsEngine;
namespace {
RigidBodyPtr Body(Vector2 p, bool fixed = false) {
    auto body = std::make_shared<RigidBody>(Circle(0.1f), Material{1, 0}, p, fixed);
    body->SetCollisionMaskBits(0);
    return body;
}
}

TEST_CASE("Distance joint shares axial momentum analytically", "[joints]") {
    World world;
    auto a = Body({0, 0}), b = Body({2, 0});
    a->SetMass(1); b->SetMass(3); a->SetVelocity({4, 0});
    world.addBody(a); world.addBody(b);
    world.addJoint(std::make_shared<DistanceJoint>(a, b, 2));
    world.step(0);
    REQUIRE(a->velocity.x == Catch::Approx(1));
    REQUIRE(b->velocity.x == Catch::Approx(1));
}

TEST_CASE("Joints maintain anchors in long running pendulums", "[joints]") {
    SimulationConfig config; config.solverIterations = 20;
    World world(config);
    auto a = Body({0, 0}, true), b = Body({1, -1});
    world.addBody(a); world.addBody(b);
    JointPtr joint;
    float target = 0;
    SECTION("distance") { target = std::sqrt(2.0f); joint = std::make_shared<DistanceJoint>(a, b, target); }
    SECTION("revolute") { joint = std::make_shared<RevoluteJoint>(a, b, Vector2{}, Vector2(-1, 1)); }
    world.addJoint(joint);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2(0, -9.81f)));
    float maxError = 0;
    for (int i=0; i<3000; ++i) {
        world.step(1.0f/120);
        maxError = std::max(maxError, std::abs((joint->getAnchorA()-joint->getAnchorB()).magnitude()-target));
        REQUIRE(std::isfinite(b->position.x));
    }
    REQUIRE(maxError < 0.006f);
    REQUIRE(a->position == Vector2(0, 0));
}

TEST_CASE("Connected islands sleep and wake together", "[sleep][joints]") {
    SimulationConfig config; config.enableSleeping = true; config.sleepTimeThreshold = 0.1f;
    World world(config);
    auto a = Body({0, 0}), b = Body({1, 0}), c = Body({10, 0});
    world.addBody(a); world.addBody(b); world.addBody(c);
    auto joint = std::make_shared<DistanceJoint>(a, b, 1);
    world.addJoint(joint);
    for (int i=0; i<20; ++i) world.step();
    REQUIRE_FALSE(a->IsAwake()); REQUIRE_FALSE(b->IsAwake()); REQUIRE_FALSE(c->IsAwake());
    REQUIRE(world.getLastStepStatistics().islandCount == 2);
    REQUIRE(world.getLastStepStatistics().integratedBodyCount == 0);
    REQUIRE(world.getLastStepStatistics().solvedConstraintCount == 0);
    a->ApplyForce({1, 0}); world.step();
    REQUIRE(a->IsAwake()); REQUIRE(b->IsAwake()); REQUIRE_FALSE(c->IsAwake());
    REQUIRE(world.getLastStepStatistics().solvedIslandCount == 1);
    world.removeBody(a);
    REQUIRE(world.getJoints().empty());
    config.enableSleeping = false; world.setSimulationConfig(config); world.step();
    REQUIRE(c->IsAwake());
}

TEST_CASE("Sleeping preserves a settled contact under gravity", "[sleep]") {
    Vector2 positions[2];
    for (int sleeping=0; sleeping<2; ++sleeping) {
        SimulationConfig config; config.enableSleeping = sleeping != 0;
        World world(config);
        auto ball = std::make_shared<RigidBody>(Circle(0.5f), Material{1, 0}, Vector2(0, 0.5f));
        auto floor = std::make_shared<RigidBody>(Polygon::MakeBox(10, 1), Material{1, 0}, Vector2(0, -0.5f), true);
        world.addBody(ball); world.addBody(floor);
        world.addUniversalForce(std::make_unique<Gravity>(Vector2(0, -9.81f)));
        for (int i=0; i<600; ++i) world.step();
        positions[sleeping] = ball->position;
        REQUIRE(ball->IsAwake() == (sleeping == 0));
        if (sleeping) REQUIRE(world.getLastStepStatistics().integratedBodyCount == 0);
    }
    REQUIRE((positions[0]-positions[1]).magnitude() < 0.006f);
}

TEST_CASE("Joint membership and invalid parameters are checked", "[joints][validation]") {
    World world;
    auto a = Body({0, 0}), b = Body({1, 0});
    REQUIRE_THROWS_AS(DistanceJoint(a, a, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(DistanceJoint(a, b, 0), std::invalid_argument);
    auto joint = std::make_shared<DistanceJoint>(a, b, 1);
    REQUIRE_THROWS_AS(world.addJoint(joint), std::invalid_argument);
    world.addBody(a); world.addBody(b); world.addJoint(joint); world.addJoint(joint);
    REQUIRE(world.getJoints().size() == 1);
    world.clearBodies(); REQUIRE(world.getJoints().empty());
}

TEST_CASE("Moving a static support wakes a settled body", "[sleep]") {
    SimulationConfig config; config.enableSleeping = true;
    World world(config);
    auto ball = std::make_shared<RigidBody>(Circle(0.5f), Material{1, 0}, Vector2(0, 0.5f));
    auto floor = std::make_shared<RigidBody>(Polygon::MakeBox(10, 1), Material{1, 0}, Vector2(0, -0.5f), true);
    world.addBody(ball); world.addBody(floor);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2(0, -9.81f)));
    for (int i=0; i<120; ++i) world.step();
    REQUIRE_FALSE(ball->IsAwake());
    floor->SetPosition({0, -5}); world.step();
    REQUIRE(ball->IsAwake());
    REQUIRE(ball->velocity.y < 0);
}

TEST_CASE("Revolute motors obey the torque budget across solver iterations", "[joints][motor]") {
    for (const int iterations : {1, 10, 40}) {
        SimulationConfig config; config.solverIterations = iterations;
        config.enableAngularVelocityLimit = false;
        World world(config);
        auto a = Body({}, true), b = Body({});
        b->SetMass(2);
        world.addBody(a); world.addBody(b);
        auto joint = std::make_shared<RevoluteJoint>(a, b);
        joint->setMotor(true, 100, 0.2f);
        world.addJoint(joint);
        world.step(0.1f);
        REQUIRE(b->angularVelocity == Catch::Approx(0.2 * 0.1 / b->inertia));
        REQUIRE(joint->getMotorTorque() == Catch::Approx(0.2));
        REQUIRE(a->angularVelocity == 0);
        world.step(0);
        REQUIRE(joint->getMotorTorque() == 0);
        REQUIRE(b->angularVelocity == Catch::Approx(0.2 * 0.1 / b->inertia));
    }
}

TEST_CASE("Motor acceleration scales with elapsed time", "[joints][motor]") {
    for (const int steps : {1, 2, 20}) {
        World world;
        auto a = Body({}, true), b = Body({});
        b->SetMass(1);
        world.addBody(a); world.addBody(b);
        auto joint = std::make_shared<RevoluteJoint>(a, b);
        joint->setMotor(true, -10, 0.01f);
        world.addJoint(joint);
        for (int i = 0; i < steps; ++i) world.step(0.5f / steps);
        REQUIRE(b->angularVelocity == Catch::Approx(-0.01 * 0.5 / b->inertia));
        REQUIRE(joint->getMotorTorque() == Catch::Approx(-0.01));
    }
}

TEST_CASE("Motor impulses conserve two-body angular momentum and brake without overshoot", "[joints][motor]") {
    World world;
    auto a = Body({}), b = Body({});
    a->SetMass(1); b->SetMass(3);
    world.addBody(a); world.addBody(b);
    auto joint = std::make_shared<RevoluteJoint>(a, b);
    joint->setMotor(true, 4, 100);
    world.addJoint(joint);
    world.step(0.01f);
    REQUIRE(a->angularVelocity == Catch::Approx(-3));
    REQUIRE(b->angularVelocity == Catch::Approx(1));
    REQUIRE(a->inertia * a->angularVelocity + b->inertia * b->angularVelocity == Catch::Approx(0).margin(1e-7));
    REQUIRE(std::abs(joint->getMotorTorque()) <= 100);
    joint->setMotor(true, 0, 100);
    world.step(0.01f);
    REQUIRE(a->angularVelocity == Catch::Approx(0).margin(1e-6));
    REQUIRE(b->angularVelocity == Catch::Approx(0).margin(1e-6));
    REQUIRE(joint->getMotorTorque() < 0);
}

TEST_CASE("Motor commands validate atomically and disabled motors leave rotation free", "[joints][motor][validation]") {
    World world;
    auto a = Body({}, true), b = Body({});
    world.addBody(a); world.addBody(b);
    auto joint = std::make_shared<RevoluteJoint>(a, b);
    world.addJoint(joint);
    REQUIRE_FALSE(joint->isMotorEnabled());
    joint->setMotor(true, 2, 3);
    REQUIRE_THROWS_AS(joint->setMotor(false, std::numeric_limits<float>::infinity(), 4), std::invalid_argument);
    REQUIRE_THROWS_AS(joint->setMotor(false, 4, -1), std::invalid_argument);
    REQUIRE_THROWS_AS(joint->setMotor(false, 4, std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
    REQUIRE(joint->isMotorEnabled());
    REQUIRE(joint->getMotorSpeed() == 2);
    REQUIRE(joint->getMaxMotorTorque() == 3);
    b->SetAngularVelocity(5);
    joint->setMotor(false, 2, 3);
    world.step(0.1f);
    REQUIRE(b->angularVelocity == 5);
    REQUIRE(joint->getMotorTorque() == 0);
    joint->setMotor(true, 2, 0);
    world.step(0.1f);
    REQUIRE(b->angularVelocity == 5);
}

TEST_CASE("A slow motor wakes its island and keeps driving below the sleep threshold", "[joints][motor][sleep]") {
    SimulationConfig config; config.enableSleeping = true; config.sleepTimeThreshold = 0.05f;
    World world(config);
    auto a = Body({}, true), b = Body({}), c = Body({1, 0});
    world.addBody(a); world.addBody(b); world.addBody(c);
    auto joint = std::make_shared<RevoluteJoint>(a, b);
    world.addJoint(joint);
    world.addJoint(std::make_shared<DistanceJoint>(b, c, 1));
    for (int i = 0; i < 30; ++i) world.step(0.01f);
    REQUIRE_FALSE(b->IsAwake()); REQUIRE_FALSE(c->IsAwake());
    joint->setMotor(true, 0.001f, 0.001f);
    REQUIRE(b->IsAwake());
    for (int i = 0; i < 30; ++i) world.step(0.01f);
    REQUIRE(b->IsAwake()); REQUIRE(c->IsAwake());
    REQUIRE(b->angularVelocity == Catch::Approx(0.001f));
    joint->setMotor(false, 0, 0);
    for (int i = 0; i < 30; ++i) world.step(0.01f);
    REQUIRE_FALSE(b->IsAwake()); REQUIRE_FALSE(c->IsAwake());
}

TEST_CASE("Angular stops hold against motors and allow travel back into range", "[joints][limits][motor]") {
    for (const float direction : {-1.0f, 1.0f}) {
        World world;
        auto a = Body({}, true), b = Body({});
        b->SetMass(1);
        world.addBody(a); world.addBody(b);
        auto joint = std::make_shared<RevoluteJoint>(a, b);
        joint->setLimits(true, -0.4f, 0.7f);
        joint->setMotor(true, direction * 2, 1);
        world.addJoint(joint);
        for (int i = 0; i < 600; ++i) {
            world.step(1.0f / 120);
            REQUIRE(joint->getAngle() >= -0.40001);
            REQUIRE(joint->getAngle() <= 0.70001);
            REQUIRE(std::abs(joint->getMotorTorque()) <= 1.00001);
        }
        REQUIRE(joint->getAngle() == Catch::Approx(direction < 0 ? -0.4 : 0.7).margin(1e-5));
        REQUIRE(b->angularVelocity == Catch::Approx(0).margin(1e-5));
        joint->setMotor(true, -direction, 1);
        for (int i = 0; i < 12; ++i) world.step(1.0f / 120);
        REQUIRE(b->angularVelocity == Catch::Approx(-direction));
        REQUIRE(joint->getAngle() > -0.4 + 0.05);
        REQUIRE(joint->getAngle() < 0.7 - 0.05);
    }
}

TEST_CASE("An equal-angle stop conserves angular momentum while locking relative rotation", "[joints][limits]") {
    World world;
    auto a = Body({}), b = Body({});
    a->SetMass(1); b->SetMass(3);
    a->SetAngularVelocity(4);
    world.addBody(a); world.addBody(b);
    auto joint = std::make_shared<RevoluteJoint>(a, b);
    joint->setLimits(true, 0.3f, 0.3f);
    world.addJoint(joint);
    world.step(0);
    REQUIRE(a->angularVelocity == Catch::Approx(1));
    REQUIRE(b->angularVelocity == Catch::Approx(1));
    REQUIRE(joint->getAngle() == Catch::Approx(0.3));
    const float momentum = a->inertia * 4;
    for (int i = 0; i < 1200; ++i) world.step(1.0f / 120);
    REQUIRE(joint->getAngle() == Catch::Approx(0.3).margin(1e-5));
    REQUIRE(a->inertia * a->angularVelocity + b->inertia * b->angularVelocity == Catch::Approx(momentum));
}

TEST_CASE("Joint limits use a relative reference across body angle wrapping", "[joints][limits]") {
    World world;
    auto a = Body({}), b = Body({});
    a->SetOrientation(2.9f); b->SetOrientation(-2.9f);
    a->SetAngularVelocity(2); b->SetAngularVelocity(2);
    world.addBody(a); world.addBody(b);
    auto joint = std::make_shared<RevoluteJoint>(a, b);
    joint->setLimits(true, -0.1f, 0.1f);
    world.addJoint(joint);
    REQUIRE(joint->getAngle() == Catch::Approx(0).margin(1e-7));
    for (int i = 0; i < 1200; ++i) {
        world.step(1.0f / 120);
        REQUIRE(std::abs(joint->getAngle()) < 1e-4);
    }
}

TEST_CASE("Limits preserve free motion inside their interval and validate before mutation", "[joints][limits][validation]") {
    World world;
    auto a = Body({}, true), b = Body({});
    world.addBody(a); world.addBody(b);
    auto joint = std::make_shared<RevoluteJoint>(a, b);
    world.addJoint(joint);
    REQUIRE_FALSE(joint->areLimitsEnabled());
    joint->setLimits(true, -1, 1);
    REQUIRE_THROWS_AS(joint->setLimits(false, 2, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(joint->setLimits(false, -4, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(joint->setLimits(false, -1, 4), std::invalid_argument);
    REQUIRE_THROWS_AS(joint->setLimits(false, 0, std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
    REQUIRE(joint->areLimitsEnabled());
    REQUIRE(joint->getLowerLimit() == -1);
    REQUIRE(joint->getUpperLimit() == 1);
    b->SetAngularVelocity(1);
    world.step(0);
    REQUIRE(b->angularVelocity == 1);
    world.step(0.1f);
    REQUIRE(b->angularVelocity == 1);
    REQUIRE(joint->getAngle() == Catch::Approx(0.1));
    b->SetOrientation(1.3f);
    world.step(0);
    REQUIRE(joint->getAngle() == Catch::Approx(1));
    REQUIRE(b->angularVelocity == 0);
    b->SetAngularVelocity(-1);
    world.step(0);
    REQUIRE(b->angularVelocity == -1);
    joint->setLimits(false, -1, 1);
    b->SetOrientation(2);
    world.step(0);
    REQUIRE(joint->getAngle() == Catch::Approx(2));
}

TEST_CASE("Limited off-center hinges maintain both anchors and angle", "[joints][limits]") {
    SimulationConfig config; config.solverIterations = 30;
    World world(config);
    auto a = Body({}, true), b = Body({1, 0});
    world.addBody(a); world.addBody(b);
    auto joint = std::make_shared<RevoluteJoint>(a, b, Vector2{}, Vector2(-1, 0));
    joint->setLimits(true, -0.5f, 0.5f);
    world.addJoint(joint);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2(0, -9.81f)));
    for (int i = 0; i < 1200; ++i) {
        world.step(1.0f / 120);
        REQUIRE(std::abs(joint->getAngle()) <= 0.505);
        REQUIRE((joint->getAnchorB() - joint->getAnchorA()).magnitude() < 0.005f);
    }
}

TEST_CASE("Editing a stop wakes a sleeping articulation", "[joints][limits][sleep]") {
    SimulationConfig config; config.enableSleeping = true; config.sleepTimeThreshold = 0.05f;
    World world(config);
    auto a = Body({}, true), b = Body({});
    world.addBody(a); world.addBody(b);
    auto joint = std::make_shared<RevoluteJoint>(a, b);
    world.addJoint(joint);
    for (int i = 0; i < 30; ++i) world.step(0.01f);
    REQUIRE_FALSE(b->IsAwake());
    joint->setLimits(true, 0.2f, 0.3f);
    REQUIRE(b->IsAwake());
    world.step(0.01f);
    REQUIRE(joint->getAngle() == Catch::Approx(0.2));
}
