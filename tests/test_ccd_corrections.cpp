// CCD after the velocity sweep: position corrections swept against moving
// bodies.
#include "catch_amalgamated.hpp"
#include "physics/physics.h"

#include <algorithm>
#include <cmath>
#include <memory>

using namespace PhysicsEngine;

namespace {
SimulationConfig Order(bool speculative) {
    SimulationConfig config;
    config.enableSpeculativeContacts = speculative;
    return config;
}

RigidBodyPtr Plank(Vector2 position, bool capsule, Vector2 velocity) {
    RigidBodyPtr plank = capsule
        ? std::make_shared<RigidBody>(Capsule(1000.0f, 0.01f), Material{}, position)
        : std::make_shared<RigidBody>(Polygon::MakeBox(0.02f, 2000.0f), Material{}, position);
    if (capsule) plank->SetOrientation(1.5707963267948966f);
    plank->SetMass(100000);
    plank->SetVelocity(velocity);
    return plank;
}

JointPtr Link(int kind, const RigidBodyPtr& driver, const RigidBodyPtr& body) {
    if (kind == 2) return std::make_shared<RevoluteJoint>(driver, body, Vector2{0, -0.5f}, Vector2{});
    auto rod = std::make_shared<DistanceJoint>(driver, body, 0.5f);
    if (kind == 1) rod->setLengthRange(0, 0.5f);
    return rod;
}
}

TEST_CASE("Joint position correction never pulls a CCD body through a moving plank", "[ccd][ccd-corrections][joint]") {
    // A heavy body that ignores everything drags a thin CCD shape on a joint
    // toward a thin dynamic plank. The plank moves along its length (so it
    // stays in the way) or sits still; it is a dynamic body, which the
    // static-only correction sweep did not see: the box crossed it in one
    // step and then lay beyond it.
    for (const bool speculative : {true, false})
    for (const bool capsulePlank : {false, true})
    for (const bool circleBullet : {false, true})
    for (int jointKind = 0; jointKind < 3; ++jointKind)
    for (const float speed : {5.f, 50.f, 200.f})
    for (const float plankSpeed : {0.f, 2.f}) {
        World world(Order(speculative));
        auto plank = Plank({0, 0}, capsulePlank, {0, plankSpeed});
        RigidBodyPtr bullet = circleBullet
            ? std::make_shared<RigidBody>(Circle(0.05f), Material{}, Vector2{-0.5f, 0})
            : std::make_shared<RigidBody>(Polygon::MakeBox(0.1f, 0.1f), Material{}, Vector2{-0.5f, 0});
        bullet->SetMass(0.25f);
        bullet->SetCcdEnabled(true);
        auto heavy = std::make_shared<RigidBody>(Polygon::MakeBox(0.2f, 0.2f), Material{}, Vector2{-0.5f, 0.5f});
        heavy->SetMass(100);
        heavy->SetCollisionMaskBits(0);
        heavy->SetVelocity({speed, 0});
        world.addBody(plank);
        world.addBody(bullet);
        world.addBody(heavy);
        world.addJoint(Link(jointKind, heavy, bullet));
        bool through = false;
        double reach = -1e9;
        for (int i = 0; i < 60; ++i) {
            world.step();
            REQUIRE(std::isfinite(bullet->position.x));
            through = through || bullet->position.x > plank->position.x;
            reach = std::max(reach, double(bullet->position.x) - plank->position.x);
        }
        INFO("speculative " << speculative << ", capsule plank " << capsulePlank << ", circle " << circleBullet
             << ", joint " << jointKind << ", speed " << speed << ", plank speed " << plankSpeed
             << ": farthest " << reach);
        CHECK_FALSE(through);
    }
}

TEST_CASE("Two bodies whose corrections close on each other are both cut at contact", "[ccd][ccd-corrections][joint]") {
    // The plank is pulled toward the CCD box by its own joint while the box
    // is pulled toward the plank: the sweep follows both corrections, so
    // neither can pass the other.
    for (const bool speculative : {true, false})
    for (const bool circleBullet : {false, true})
    for (const float speed : {5.f, 50.f, 200.f}) {
        World world(Order(speculative));
        auto plank = Plank({0.5f, 0}, false, {0, 0});
        RigidBodyPtr bullet = circleBullet
            ? std::make_shared<RigidBody>(Circle(0.05f), Material{}, Vector2{-0.5f, 0})
            : std::make_shared<RigidBody>(Polygon::MakeBox(0.1f, 0.1f), Material{}, Vector2{-0.5f, 0});
        bullet->SetMass(0.25f);
        bullet->SetCcdEnabled(true);
        auto left = std::make_shared<RigidBody>(Polygon::MakeBox(0.2f, 0.2f), Material{}, Vector2{-0.5f, 0.5f});
        auto right = std::make_shared<RigidBody>(Polygon::MakeBox(0.2f, 0.2f), Material{}, Vector2{0.5f, 0.5f});
        for (const auto& driver : {left, right}) {
            driver->SetMass(100);
            driver->SetCollisionMaskBits(0);
        }
        left->SetVelocity({speed, 0});
        right->SetVelocity({-speed, 0});
        world.addBody(plank);
        world.addBody(bullet);
        world.addBody(left);
        world.addBody(right);
        world.addJoint(Link(0, left, bullet));
        world.addJoint(Link(0, right, plank));
        bool crossed = false;
        for (int i = 0; i < 60; ++i) {
            world.step();
            REQUIRE(std::isfinite(bullet->position.x));
            REQUIRE(std::isfinite(plank->position.x));
            crossed = crossed || bullet->position.x > plank->position.x;
        }
        INFO("speculative " << speculative << ", circle " << circleBullet << ", speed " << speed);
        CHECK_FALSE(crossed);
    }
}
