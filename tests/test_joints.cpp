#include "catch_amalgamated.hpp"
#include "physics/core/world.h"
#include "physics/core/forces/gravity.h"

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
