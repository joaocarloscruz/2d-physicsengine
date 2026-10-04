#include "catch_amalgamated.hpp"
#include "../include/physics/core/forces/gravity.h"
#include "../include/physics/core/forces/drag.h"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/math/vector2.h"
#include "../include/physics/core/shape.h"
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace PhysicsEngine;

TEST_CASE("Drag preserves finite force and direction at large velocity scales", "[drag-numerics]") {
    for (const auto coefficients : {Vector2{0, 0}, Vector2{2, 0}, Vector2{0, 1e-30f}}) {
        RigidBody body(Circle(1), Material{});
        body.SetVelocity({3e30f, -4e30f});
        const double speed = std::hypot(double(body.velocity.x), double(body.velocity.y));
        const double gain = coefficients.x + coefficients.y * speed;
        Drag drag(coefficients.x, coefficients.y);
        REQUIRE_NOTHROW(drag.applyForce(&body));
        REQUIRE(body.GetForce().x == Catch::Approx(-gain * body.velocity.x).epsilon(1e-6));
        REQUIRE(body.GetForce().y == Catch::Approx(-gain * body.velocity.y).epsilon(1e-6));
        REQUIRE(double(body.force.x) * body.velocity.x + double(body.force.y) * body.velocity.y <= 0);
    }
}

TEST_CASE("Unrepresentable drag preserves pending loads", "[drag-numerics]") {
    RigidBody body(Circle(1), Material{});
    body.SetVelocity({1e30f, 0});
    body.ApplyForce({1, 2});
    Drag drag(0, 1);
    REQUIRE_THROWS_AS(drag.applyForce(&body), std::overflow_error);
    REQUIRE(body.GetForce().x == 1);
    REQUIRE(body.GetForce().y == 2);
}

TEST_CASE("Force generators reject missing targets and invalid consumed state", "[force-numerics]") {
    Drag drag(1, 0);
    Gravity gravity({0, -10});
    REQUIRE_THROWS_AS(drag.applyForce(nullptr), std::invalid_argument);
    REQUIRE_THROWS_AS(gravity.applyForce(nullptr), std::invalid_argument);
    RigidBody body(Circle(1), Material{});
    body.velocity.x = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(drag.applyForce(&body), std::invalid_argument);
    body.mass = -1;
    REQUIRE_THROWS_AS(gravity.applyForce(&body), std::invalid_argument);
    body.SetMass(1e38f);
    REQUIRE_THROWS_AS(gravity.applyForce(&body), std::overflow_error);
    REQUIRE(body.GetForce().x == 0);
    REQUIRE(body.GetForce().y == 0);
}

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
