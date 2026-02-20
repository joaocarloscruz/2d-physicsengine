#include "catch_amalgamated.hpp"
#include "../include/physics/core/forces/gravity.h"
#include "../include/physics/core/forces/drag.h"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/math/vector2.h"
#include "../include/physics/core/shape.h"
#include <cmath>

using namespace PhysicsEngine;

TEST_CASE("Gravity Force", "[forces]") {
    Circle circle(1.0f);
    Material material = {1.0f, 0.5f};
    RigidBody body(&circle, material, {0, 0});
    body.SetMass(10.0f);

    SECTION("Gravity applies force correctly") {
        Gravity gravity({0, -9.8f});
        gravity.applyForce(&body);

        REQUIRE(body.GetForce().y == -98.0f);
    }

    SECTION("setGravity updates the gravity value") {
        Gravity gravity({0, -9.8f});
        gravity.setGravity({0, -1.62f}); // Moon gravity
        gravity.applyForce(&body);

        REQUIRE(body.GetForce().y == -16.2f);
    }
}

TEST_CASE("Drag Force", "[forces]") {
    Circle circle(1.0f);
    Material material = {1.0f, 0.5f};

    SECTION("Linear drag (k2=0) opposes motion and scales with speed") {
        RigidBody body(&circle, material, {0, 0});
        body.SetMass(1.0f);
        body.SetVelocity(Vector2(4.0f, 0.0f)); // moving right at speed 4

        // k1=2, k2=0 -> drag = -k1 * v = -(2 * 4) = -8 in x
        Drag drag(2.0f, 0.0f);
        drag.applyForce(&body);

        REQUIRE(body.GetForce().x == Catch::Approx(-8.0f));
        REQUIRE(body.GetForce().y == Catch::Approx(0.0f));
    }

    SECTION("Quadratic drag (k1=0) opposes motion and scales with speed squared") {
        RigidBody body(&circle, material, {0, 0});
        body.SetMass(1.0f);
        body.SetVelocity(Vector2(3.0f, 0.0f)); // speed = 3

        // k1=0, k2=2 -> drag magnitude = k2 * v^2 = 2 * 9 = 18, opposing direction (-x)
        Drag drag(0.0f, 2.0f);
        drag.applyForce(&body);

        REQUIRE(body.GetForce().x == Catch::Approx(-18.0f));
        REQUIRE(body.GetForce().y == Catch::Approx(0.0f));
    }

    SECTION("Combined drag applies both linear and quadratic terms") {
        RigidBody body(&circle, material, {0, 0});
        body.SetMass(1.0f);
        body.SetVelocity(Vector2(2.0f, 0.0f)); // speed = 2

        // k1=1, k2=1 -> drag = -(k1 * |v| + k2 * v^2) * direction
        //                     = -(1*2 + 1*4) * (1,0) = -6 in x
        Drag drag(1.0f, 1.0f);
        drag.applyForce(&body);

        REQUIRE(body.GetForce().x == Catch::Approx(-6.0f));
        REQUIRE(body.GetForce().y == Catch::Approx(0.0f));
    }

    SECTION("Drag on a body moving in Y direction opposes correctly") {
        RigidBody body(&circle, material, {0, 0});
        body.SetMass(1.0f);
        body.SetVelocity(Vector2(0.0f, -5.0f)); // moving down at speed 5

        // k1=1, k2=0 -> drag = -(1 * 5) * (0,-1) = (0, 5)
        Drag drag(1.0f, 0.0f);
        drag.applyForce(&body);

        REQUIRE(body.GetForce().x == Catch::Approx(0.0f));
        REQUIRE(body.GetForce().y == Catch::Approx(5.0f));
    }

    SECTION("Drag with zero velocity produces zero force") {
        RigidBody body(&circle, material, {0, 0});
        body.SetMass(1.0f);
        // velocity stays at (0,0)

        Drag drag(2.0f, 3.0f);
        drag.applyForce(&body);

        // No motion -> no drag force
        REQUIRE(body.GetForce().x == Catch::Approx(0.0f));
        REQUIRE(body.GetForce().y == Catch::Approx(0.0f));
    }
}
