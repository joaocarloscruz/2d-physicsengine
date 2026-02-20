#include "catch_amalgamated.hpp"
#include "../include/physics/math/vector2.h"
#include "../include/physics/math/matrix2x2.h"
#include <cmath>

using namespace Catch;
using namespace PhysicsEngine;

TEST_CASE("Vector2 operations are correct", "[Vector2]") {
    Vector2 v1(2.0f, 3.0f);
    Vector2 v2(4.0f, 1.0f);

    SECTION("Addition") {
        Vector2 sum = v1 + v2;
        REQUIRE(sum.x == 6.0f);
        REQUIRE(sum.y == 4.0f);
    }

    SECTION("Subtraction") {
        Vector2 diff = v1 - v2;
        REQUIRE(diff.x == -2.0f);
        REQUIRE(diff.y == 2.0f);
    }

    SECTION("Magnitude") {
        REQUIRE(Approx(v1.magnitude()).epsilon(0.001f) == 3.60555f);
        REQUIRE(Approx(v1.magnitudeSquared()).epsilon(0.001f) == 13.0f);
    }

    SECTION("Normalization") {
        Vector2 normalized = v1.normalized();
        REQUIRE(Approx(normalized.x).epsilon(0.001f) == 0.5547f);
        REQUIRE(Approx(normalized.y).epsilon(0.001f) == 0.83205f);
        REQUIRE(Approx(normalized.magnitude()).epsilon(0.001f) == 1.0f);
    }

    SECTION("Scalar multiplication") {
        Vector2 scaled = v1 * 3.0f;
        REQUIRE(scaled.x == Approx(6.0f));
        REQUIRE(scaled.y == Approx(9.0f));
    }

    SECTION("Scalar division") {
        Vector2 divided = v1 / 2.0f;
        REQUIRE(divided.x == Approx(1.0f));
        REQUIRE(divided.y == Approx(1.5f));
    }

    SECTION("Equality operator") {
        Vector2 same(2.0f, 3.0f);
        Vector2 different(2.0f, 4.0f);
        REQUIRE(v1 == same);
        REQUIRE_FALSE(v1 == different);
    }

    SECTION("Dot product") {
        // v1=(2,3), v2=(4,1) -> 2*4 + 3*1 = 11
        float result = v1.dot(v2);
        REQUIRE(result == Approx(11.0f));
    }

    SECTION("Dot product with perpendicular vectors is zero") {
        Vector2 a(1.0f, 0.0f);
        Vector2 b(0.0f, 1.0f);
        REQUIRE(a.dot(b) == Approx(0.0f));
    }

    SECTION("Cross product (scalar)") {
        // v1 x v2 = v1.x*v2.y - v1.y*v2.x = 2*1 - 3*4 = 2 - 12 = -10
        float result = v1.cross(v2);
        REQUIRE(result == Approx(-10.0f));
    }

    SECTION("Cross product with parallel vectors is zero") {
        Vector2 a(2.0f, 0.0f);
        Vector2 b(5.0f, 0.0f);
        REQUIRE(a.cross(b) == Approx(0.0f));
    }

    SECTION("Static cross product: scalar x vector") {
        // scalar x v -> Vector2(-scalar*v.y, scalar*v.x)
        Vector2 v(1.0f, 0.0f);
        Vector2 result = Vector2::cross(1.0f, v);
        REQUIRE(result.x == Approx(0.0f));
        REQUIRE(result.y == Approx(1.0f));
    }

    SECTION("Static cross product: vector x scalar") {
        // v x scalar -> Vector2(scalar*v.y, -scalar*v.x)
        Vector2 v(1.0f, 0.0f);
        Vector2 result = Vector2::cross(v, 1.0f);
        REQUIRE(result.x == Approx(0.0f));
        REQUIRE(result.y == Approx(-1.0f));
    }

    SECTION("Zero vector magnitude is zero") {
        Vector2 zero(0.0f, 0.0f);
        REQUIRE(zero.magnitude() == Approx(0.0f));
        REQUIRE(zero.magnitudeSquared() == Approx(0.0f));
    }
}

TEST_CASE("Matrix2x2 operations are correct", "[Matrix2x2]") {
    Matrix2x2 m1(1.0f, 2.0f, 3.0f, 4.0f);
    Vector2 v1(5.0f, 6.0f);

    SECTION("Vector Multiplication") {
        Vector2 result = m1 * v1;
        REQUIRE(result.x == 17.0f);
        REQUIRE(result.y == 39.0f);
    }

    SECTION("Determinant and Inverse") {
        REQUIRE(m1.determinant() == -2.0f);
        Matrix2x2 inv = m1.inverse();
        
        Matrix2x2 expected_inv( -2.0f, 1.0f, 1.5f, -0.5f );
        REQUIRE(Approx(inv.m[0][0]) == expected_inv.m[0][0]);
        REQUIRE(Approx(inv.m[0][1]) == expected_inv.m[0][1]);
        REQUIRE(Approx(inv.m[1][0]) == expected_inv.m[1][0]);
        REQUIRE(Approx(inv.m[1][1]) == expected_inv.m[1][1]);
    }

    SECTION("Rotation factory: 0 degrees is identity") {
        Matrix2x2 rot = Matrix2x2::rotation(0.0f);
        Vector2 v(1.0f, 0.0f);
        Vector2 result = rot * v;
        REQUIRE(result.x == Approx(1.0f).margin(1e-5f));
        REQUIRE(result.y == Approx(0.0f).margin(1e-5f));
    }

    SECTION("Rotation factory: 90 degrees rotates (1,0) to (0,1)") {
        float angle = static_cast<float>(M_PI) / 2.0f;
        Matrix2x2 rot = Matrix2x2::rotation(angle);
        Vector2 v(1.0f, 0.0f);
        Vector2 result = rot * v;
        REQUIRE(result.x == Approx(0.0f).margin(1e-5f));
        REQUIRE(result.y == Approx(1.0f).margin(1e-5f));
    }

    SECTION("Rotation factory: 180 degrees rotates (1,0) to (-1,0)") {
        float angle = static_cast<float>(M_PI);
        Matrix2x2 rot = Matrix2x2::rotation(angle);
        Vector2 v(1.0f, 0.0f);
        Vector2 result = rot * v;
        REQUIRE(result.x == Approx(-1.0f).margin(1e-5f));
        REQUIRE(result.y == Approx(0.0f).margin(1e-5f));
    }

    SECTION("Rotation preserves vector magnitude") {
        float angle = 1.23456f; // arbitrary angle
        Matrix2x2 rot = Matrix2x2::rotation(angle);
        Vector2 v(3.0f, 4.0f); // magnitude 5
        Vector2 result = rot * v;
        REQUIRE(result.magnitude() == Approx(v.magnitude()).margin(1e-4f));
    }
}