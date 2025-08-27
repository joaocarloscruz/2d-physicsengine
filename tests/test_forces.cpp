#include "catch_amalgamated.hpp"
#include "../include/physics/core/forces/gravity.h"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/math/vector2.h"
#include "../include/physics/core/shape.h"

TEST_CASE("Gravity Force", "[forces]") {
    SECTION("Gravity applies force correctly") {
        Circle circle(1.0f);
        RigidBody body(&circle, 1.0f, {0, 0});
        body.SetMass(10.0f);

        Gravity gravity({0, -9.8f});
        gravity.applyForce(&body);

        REQUIRE(body.GetForce().y == -98.0f);
    }
}
