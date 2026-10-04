#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace PhysicsEngine;

TEST_CASE("Polygon centroid matches analytical triangles and asymmetric trapezoids", "[centroid]") {
    std::vector<Vector2> triangle{{-2, 0}, {2, 0}, {0, 5}};
    for (int winding = 0; winding < 2; ++winding) {
        Polygon p(triangle);
        REQUIRE(p.GetCentroid().x == Catch::Approx(0).epsilon(0).margin(1e-7));
        REQUIRE(p.GetCentroid().y == Catch::Approx(5.0 / 3).epsilon(1e-7));
        const auto centered = p.Recentered();
        REQUIRE(centered.GetCentroid().x == Catch::Approx(0).epsilon(0).margin(1e-7));
        REQUIRE(centered.GetCentroid().y == Catch::Approx(0).epsilon(0).margin(1e-7));
        REQUIRE(centered.GetArea() == Catch::Approx(10).epsilon(1e-7));
        REQUIRE(centered.GetInertia(1) == Catch::Approx(37.0 / 18).epsilon(2e-7));
        REQUIRE(p.getVertices() == triangle);
        std::reverse(triangle.begin(), triangle.end());
    }
    const Polygon trapezoid({{0, 0}, {4, 0}, {3, 2}, {1, 2}});
    REQUIRE(trapezoid.GetCentroid().x == 2);
    REQUIRE(trapezoid.GetCentroid().y == Catch::Approx(8.0 / 9).epsilon(1e-7));
}

TEST_CASE("Recentered inertia obeys the parallel-axis theorem", "[centroid]") {
    const Polygon p({{5, -1}, {7, -1}, {7, 1}, {5, 1}});
    const auto c = p.GetCentroid();
    const auto centered = p.Recentered();
    REQUIRE(c.x == 6);
    REQUIRE(c.y == 0);
    for (const float mass : {0.1f, 1.0f, 10.0f}) {
        REQUIRE(centered.GetInertia(mass) == Catch::Approx(mass * 2.0 / 3).epsilon(1e-7));
        REQUIRE(p.GetInertia(mass) == Catch::Approx(centered.GetInertia(mass)
            + double(mass) * (double(c.x) * c.x + double(c.y) * c.y)).epsilon(1e-7));
    }
    REQUIRE(centered.GetArea() == p.GetArea());
}

TEST_CASE("Polygon centroid uses local moments across coordinate scales", "[centroid]") {
    for (const float scale : {1e-18f, 1.0f, 1e18f}) {
        const Polygon p({{2 * scale, -4 * scale}, {6 * scale, -4 * scale},
                         {6 * scale, -2 * scale}, {2 * scale, -2 * scale}});
        REQUIRE(p.GetCentroid().x == Catch::Approx(4 * double(scale)).epsilon(2e-7));
        REQUIRE(p.GetCentroid().y == Catch::Approx(-3 * double(scale)).epsilon(2e-7));
        const auto centered = p.Recentered();
        REQUIRE(std::abs(double(centered.GetCentroid().x)) <= 4e-7 * scale);
        REQUIRE(std::abs(double(centered.GetCentroid().y)) <= 4e-7 * scale);
    }
    const Polygon translated({{1e8f, 1e8f}, {1e8f + 32, 1e8f},
                              {1e8f + 32, 1e8f + 16}, {1e8f, 1e8f + 16}});
    REQUIRE(translated.GetCentroid().x == 1e8f + 16);
    REQUIRE(translated.GetCentroid().y == 1e8f + 8);
    REQUIRE(translated.Recentered().GetArea() == 512);
}

TEST_CASE("Recentering preserves transformed geometry when the origin is shifted", "[centroid]") {
    const Polygon p({{1, 1}, {5, 1}, {3, 4}});
    const Polygon centered = p.Recentered();
    const auto c = p.GetCentroid();
    const Vector2 origin{4, -2};
    const double angle = 0.7, cosine = std::cos(angle), sine = std::sin(angle);
    const double newX = origin.x + cosine * c.x - sine * c.y;
    const double newY = origin.y + sine * c.x + cosine * c.y;
    for (std::size_t i = 0; i < p.getVertices().size(); ++i) {
        const auto a = p.getVertices()[i], b = centered.getVertices()[i];
        REQUIRE(origin.x + cosine * a.x - sine * a.y
            == Catch::Approx(newX + cosine * b.x - sine * b.y).epsilon(0).margin(1e-6));
        REQUIRE(origin.y + sine * a.x + cosine * a.y
            == Catch::Approx(newY + sine * b.x + cosine * b.y).epsilon(0).margin(1e-6));
    }
    RigidBody body(centered, Material{});
    body.SetMass(1);
    body.ApplyImpulse({2, 0}, {});
    REQUIRE(body.GetVelocity().x == 2);
    REQUIRE(body.GetAngularVelocity() == 0);
}

TEST_CASE("Recentering rejects unrepresentable offsets without altering the outline", "[centroid]") {
    const float maximum = std::numeric_limits<float>::max();
    const std::vector<Vector2> vertices{{-maximum, 0}, {maximum, -1}, {maximum, 1}};
    const Polygon p(vertices);
    REQUIRE(std::isfinite(p.GetCentroid().x));
    REQUIRE_THROWS_AS(p.Recentered(), std::overflow_error);
    REQUIRE(p.getVertices() == vertices);
}
