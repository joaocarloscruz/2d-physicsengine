#include "catch_amalgamated.hpp"

#include "physics/core/forces/drag.h"
#include "physics/core/forces/gravity.h"
#include "physics/core/material.h"
#include "physics/core/particles/particle.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/core/world.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace PhysicsEngine;

TEST_CASE("Circle radius must be positive and finite", "[validation][shape]") {
    REQUIRE_THROWS_AS(Circle(0.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(Circle(-1.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(Circle(std::numeric_limits<float>::infinity()), std::invalid_argument);
    REQUIRE_NOTHROW(Circle(0.001f));
}

TEST_CASE("Polygon must be non-degenerate and convex", "[validation][shape]") {
    REQUIRE_THROWS_AS(Polygon({}), std::invalid_argument);
    REQUIRE_THROWS_AS(
        Polygon({Vector2(0.0f, 0.0f), Vector2(1.0f, 0.0f)}),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(
        Polygon({
            Vector2(0.0f, 0.0f),
            Vector2(1.0f, 0.0f),
            Vector2(2.0f, 0.0f),
        }),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(
        Polygon({
            Vector2(0.0f, 0.0f),
            Vector2(2.0f, 0.0f),
            Vector2(1.0f, 0.5f),
            Vector2(2.0f, 2.0f),
            Vector2(0.0f, 2.0f),
        }),
        std::invalid_argument
    );
}

TEST_CASE("Box dimensions must be positive and finite", "[validation][shape]") {
    REQUIRE_THROWS_AS(Polygon::MakeBox(0.0f, 1.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(Polygon::MakeBox(1.0f, -1.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(
        Polygon::MakeBox(std::numeric_limits<float>::quiet_NaN(), 1.0f),
        std::invalid_argument
    );
}

TEST_CASE("RigidBody rejects non-physical materials", "[validation][RigidBody]") {
    Circle shape(1.0f);
    REQUIRE_THROWS_AS(
        RigidBody(&shape, Material{0.0f, 0.5f}),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(
        RigidBody(&shape, Material{1.0f, -0.1f}),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(
        RigidBody(&shape, Material{1.0f, 1.1f}),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(
        RigidBody(&shape, Material{1.0f, 0.5f, -0.1f, 0.2f}),
        std::invalid_argument
    );
}

TEST_CASE("Dynamic mass must be positive and finite", "[validation][RigidBody]") {
    Circle shape(1.0f);
    Material material{1.0f, 0.5f};
    RigidBody body(&shape, material);

    REQUIRE_THROWS_AS(body.SetMass(0.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(body.SetMass(-1.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(
        body.SetMass(std::numeric_limits<float>::quiet_NaN()),
        std::invalid_argument
    );
}

TEST_CASE("Static body remains infinite-mass when SetMass is called", "[validation][RigidBody]") {
    Circle shape(1.0f);
    Material material{1.0f, 0.5f};
    RigidBody body(&shape, material, Vector2(), true);

    body.SetMass(10.0f);

    REQUIRE(body.GetMass() == 0.0f);
    REQUIRE(body.GetInverseMass() == 0.0f);
    REQUIRE(body.GetInertia() == 0.0f);
    REQUIRE(body.GetInverseInertia() == 0.0f);
}

TEST_CASE("Simulation rejects negative and non-finite timesteps", "[validation][integration]") {
    Circle shape(1.0f);
    Material material{1.0f, 0.5f};
    RigidBody body(&shape, material);
    World world;

    REQUIRE_THROWS_AS(body.Integrate(-0.01f), std::invalid_argument);
    REQUIRE_THROWS_AS(
        body.Integrate(std::numeric_limits<float>::quiet_NaN()),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(world.step(-0.01f), std::invalid_argument);
    REQUIRE_THROWS_AS(
        world.step(std::numeric_limits<float>::infinity()),
        std::invalid_argument
    );
}

TEST_CASE("Force generators reject invalid coefficients", "[validation][forces]") {
    REQUIRE_THROWS_AS(Drag(-1.0f, 0.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(Drag(0.0f, -1.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(
        Drag(std::numeric_limits<float>::quiet_NaN(), 0.0f),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(
        Gravity(Vector2(0.0f, std::numeric_limits<float>::infinity())),
        std::invalid_argument
    );
}

TEST_CASE("Particle rejects invalid integration timesteps", "[validation][Particle]") {
    Particle particle;
    REQUIRE_THROWS_AS(particle.Integrate(-0.1f), std::invalid_argument);
    REQUIRE_THROWS_AS(
        particle.Integrate(std::numeric_limits<float>::infinity()),
        std::invalid_argument
    );
}
