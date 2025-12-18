#include "catch_amalgamated.hpp"
#include "../include/physics/core/forces/gravity.h"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/math/vector2.h"
#include "../include/physics/core/shape.h"

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
