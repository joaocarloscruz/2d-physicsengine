#include "catch_amalgamated.hpp"

#include "physics/core/forces/gravity.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/core/world.h"
#include "physics/math/matrix2x2.h"
#include "physics/math/vector2.h"

#include <array>
#include <cmath>
#include <memory>

using namespace PhysicsEngine;

TEST_CASE("Matrix inverse and composition preserve vectors", "[invariant][math]") {
    const Matrix2x2 matrix(3.0f, 2.0f, -1.0f, 4.0f);
    const Matrix2x2 inverse = matrix.inverse();
    const Matrix2x2 identity = matrix * inverse;
    const Vector2 vector(2.5f, -7.0f);
    const Vector2 result = identity * vector;

    REQUIRE(result.x == Catch::Approx(vector.x).margin(1e-5f));
    REQUIRE(result.y == Catch::Approx(vector.y).margin(1e-5f));
}

TEST_CASE("Singular matrix inverse is the zero matrix", "[invariant][math]") {
    const Matrix2x2 singular(1.0f, 2.0f, 2.0f, 4.0f);
    const Matrix2x2 inverse = singular.inverse();

    for (const auto& row : inverse.m) {
        for (float value : row) {
            REQUIRE(value == 0.0f);
        }
    }
}

TEST_CASE("Rotation round trips over representative angles", "[invariant][math]") {
    const Vector2 original(3.0f, -4.0f);
    const std::array<float, 7> angles{
        -3.0f, -1.0f, -0.1f, 0.0f, 0.25f, 1.5f, 3.0f,
    };

    for (float angle : angles) {
        const Vector2 rotated = Matrix2x2::rotation(angle) * original;
        const Vector2 restored = Matrix2x2::rotation(-angle) * rotated;
        CAPTURE(angle);
        REQUIRE(restored.x == Catch::Approx(original.x).margin(1e-4f));
        REQUIRE(restored.y == Catch::Approx(original.y).margin(1e-4f));
        REQUIRE(rotated.magnitude() == Catch::Approx(original.magnitude()).margin(1e-4f));
    }
}

TEST_CASE("Constant force follows the analytical trajectory", "[invariant][integration]") {
    Circle shape(1.0f);
    Material material{1.0f, 0.0f};
    RigidBody body(&shape, material, Vector2(2.0f, -1.0f));
    body.SetMass(2.0f);
    body.SetVelocity(Vector2(3.0f, 4.0f));
    body.ApplyForce(Vector2(4.0f, -8.0f));

    body.Integrate(0.5f);

    REQUIRE(body.GetPosition().x == Catch::Approx(3.75f));
    REQUIRE(body.GetPosition().y == Catch::Approx(0.5f));
    REQUIRE(body.GetVelocity().x == Catch::Approx(4.0f));
    REQUIRE(body.GetVelocity().y == Catch::Approx(2.0f));
    REQUIRE(body.GetForce().magnitudeSquared() == 0.0f);
}

TEST_CASE("Constant torque follows the analytical angular trajectory", "[invariant][integration]") {
    auto shape = Polygon::MakeBox(2.0f, 2.0f);
    Material material{1.0f, 0.0f};
    RigidBody body(&shape, material);
    body.SetMass(3.0f);
    body.SetAngularVelocity(1.0f);
    body.ApplyTorque(4.0f);
    const float angularAcceleration = 4.0f / body.GetInertia();

    body.Integrate(0.25f);

    const float expectedOrientation = 0.25f
        + 0.5f * angularAcceleration * 0.25f * 0.25f;
    REQUIRE(body.GetOrientation() == Catch::Approx(expectedOrientation));
    REQUIRE(body.GetAngularVelocity() == Catch::Approx(1.0f + angularAcceleration * 0.25f));
    REQUIRE(body.GetTorque() == 0.0f);
}

TEST_CASE("Velocity safety limits preserve direction", "[invariant][integration]") {
    Circle shape(1.0f);
    Material material{1.0f, 0.0f};
    RigidBody body(&shape, material);
    body.SetVelocity(Vector2(300.0f, 400.0f));
    body.SetAngularVelocity(-100.0f);

    body.Integrate(0.0f);

    REQUIRE(body.GetVelocity().magnitude() == Catch::Approx(200.0f));
    REQUIRE(body.GetVelocity().x / body.GetVelocity().y == Catch::Approx(0.75f));
    REQUIRE(body.GetAngularVelocity() == -30.0f);
}

TEST_CASE("World normalizes orientation to a canonical range", "[invariant][World]") {
    World world;
    auto shape = Polygon::MakeBox(1.0f, 1.0f);
    Material material{1.0f, 0.0f};
    auto body = std::make_shared<RigidBody>(&shape, material);
    body->SetOrientation(12.0f * static_cast<float>(M_PI));
    body->SetAngularVelocity(5.0f);
    world.addBody(body);

    world.step(0.1f);

    REQUIRE(body->GetOrientation() >= -static_cast<float>(M_PI));
    REQUIRE(body->GetOrientation() <= static_cast<float>(M_PI));
}

TEST_CASE("Free fall is invariant to timestep subdivision", "[invariant][integration]") {
    Circle shape(0.5f);
    Material material{1.0f, 0.0f};

    auto simulate = [&](float dt, int steps) {
        World world;
        auto body = std::make_shared<RigidBody>(&shape, material, Vector2(0.0f, 10.0f));
        world.addBody(body);
        world.addUniversalForce(std::make_unique<Gravity>(Vector2(0.0f, -9.81f)));
        for (int i = 0; i < steps; ++i) {
            world.step(dt);
        }
        return std::pair<Vector2, Vector2>{body->GetPosition(), body->GetVelocity()};
    };

    const auto coarse = simulate(1.0f / 30.0f, 60);
    const auto fine = simulate(1.0f / 240.0f, 480);

    REQUIRE(coarse.first.y == Catch::Approx(fine.first.y).margin(0.001f));
    REQUIRE(coarse.second.y == Catch::Approx(fine.second.y).margin(0.001f));
    REQUIRE(fine.first.y == Catch::Approx(10.0f - 0.5f * 9.81f * 4.0f).margin(0.001f));
}
