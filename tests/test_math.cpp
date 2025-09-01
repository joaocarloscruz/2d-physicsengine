#include "catch_amalgamated.hpp"
#include "../include/physics/math/vector2.h"
#include "../include/physics/math/matrix2x2.h"

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
        Vector2 result_v = inv * v1;
        
        Matrix2x2 expected_inv( -2.0f, 1.0f, 1.5f, -0.5f );
        REQUIRE(Approx(inv.m[0][0]) == expected_inv.m[0][0]);
        REQUIRE(Approx(inv.m[0][1]) == expected_inv.m[0][1]);
        REQUIRE(Approx(inv.m[1][0]) == expected_inv.m[1][0]);
        REQUIRE(Approx(inv.m[1][1]) == expected_inv.m[1][1]);
    }
}