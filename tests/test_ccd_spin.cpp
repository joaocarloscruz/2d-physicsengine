// The orientation of a CCD circle whose spin an impact changes.
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
}

TEST_CASE("A CCD circle's orientation follows the spin an impact gave it", "[ccd][ccd-spin]") {
    // A ball with some spin hits a wall at a known time and the wall's
    // friction changes its spin. Over the step its orientation is the old
    // spin until the impact and the new spin after it, as the ball's
    // position already was; it used to turn at the old spin throughout.
    constexpr float dt = 1.0f / 60.0f;
    for (const bool speculative : {true, false})
    for (const float spin : {0.f, 10.f, -8.f}) {
        World world(Order(speculative));
        Material grip;
        grip.restitution = 0;
        grip.staticFriction = grip.dynamicFriction = 2;
        auto wall = std::make_shared<RigidBody>(Polygon::MakeBox(0.1f, 100), grip, Vector2{1.05f, 0}, true);
        auto ball = std::make_shared<RigidBody>(Circle(0.1f), grip, Vector2{0, 0});
        ball->SetCcdEnabled(true);
        ball->SetVelocity({100, 2});
        ball->SetAngularVelocity(spin);
        world.addBody(wall);
        world.addBody(ball);
        world.step(dt);
        // The wall face is at x = 1 and the ball's surface reaches it after
        // travelling 0.9 m at 100 m/s.
        const double hitTime = 0.9 / 100;
        REQUIRE(hitTime < dt);
        const double expected = double(spin) * hitTime + double(ball->angularVelocity) * (dt - hitTime);
        INFO("speculative " << speculative << ", spin " << spin << ": spin after " << ball->angularVelocity
             << ", orientation " << ball->orientation << ", expected " << expected);
        REQUIRE(std::abs(ball->angularVelocity - spin) > 1);
        CHECK(std::abs(ball->orientation - expected) <= 2e-3 * std::max(1.0, std::abs(expected)));
    }
}

TEST_CASE("A CCD circle whose spin an impact leaves alone keeps its integrated orientation exactly",
          "[ccd][ccd-spin]") {
    // No friction and a head-on hit: the impulse acts through the centre, so
    // the spin is the same before and after the impact.
    constexpr float dt = 1.0f / 60.0f;
    for (const bool speculative : {true, false}) {
        float orientation[2] = {};
        for (const int wallCase : {0, 1}) {
            World world(Order(speculative));
            Material slick;
            slick.restitution = 0;
            slick.staticFriction = slick.dynamicFriction = 0;
            auto ball = std::make_shared<RigidBody>(Circle(0.1f), slick, Vector2{0, 0});
            ball->SetCcdEnabled(true);
            ball->SetVelocity({100, 0});
            ball->SetAngularVelocity(7);
            world.addBody(ball);
            if (wallCase == 1)
                world.addBody(std::make_shared<RigidBody>(Polygon::MakeBox(0.1f, 100), slick, Vector2{1.05f, 0}, true));
            world.step(dt);
            if (wallCase == 1) REQUIRE(ball->velocity.x < 1);
            REQUIRE(ball->angularVelocity == 7);
            orientation[wallCase] = ball->orientation;
        }
        INFO("speculative " << speculative);
        CHECK(orientation[1] == orientation[0]);
    }
}
