#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include "physics/core/collisions/broad_phase/uniform_grid.h"
#include <cmath>
#include <memory>

using namespace PhysicsEngine;
using Catch::Approx;

namespace {
RigidBodyPtr Body(const Shape& shape, Vector2 position, bool fixed = false, Material material = {1, 0}) {
    return std::make_shared<RigidBody>(shape, material, position, fixed);
}
}

TEST_CASE("Margin queries report separated contacts with negative penetration", "[speculative][narrow]") {
    SECTION("circle pair") {
        auto a = Body(Circle(0.5f), {0, 0}), b = Body(Circle(0.5f), {1.25f, 0});
        REQUIRE_FALSE(CheckCollision(a.get(), b.get()).hasCollision);
        REQUIRE_FALSE(CheckCollision(a.get(), b.get(), 0.25f).hasCollision); // gap == margin
        const auto m = CheckCollision(a.get(), b.get(), 0.5f);
        REQUIRE(m.hasCollision);
        REQUIRE(m.penetration == Approx(-0.25f));
        REQUIRE(m.contacts[0].penetration == Approx(-0.25f));
        REQUIRE(m.normal.x == Approx(1));
    }
    SECTION("circle and polygon") {
        auto box = Body(Polygon::MakeBox(2, 2), {0, 0}, true), ball = Body(Circle(0.5f), {0, 1.75f});
        const auto m = CheckCollision(ball.get(), box.get(), 0.5f);
        REQUIRE(m.hasCollision);
        REQUIRE(m.penetration == Approx(-0.25f));
        REQUIRE(m.normal.y == Approx(-1)); // From the ball (A) toward the box (B).
        REQUIRE(m.contacts[0].position.y == Approx(1.25f)); // On the ball's real surface.
    }
    SECTION("parallel polygon faces keep two points") {
        auto floor = Body(Polygon::MakeBox(4, 1), {0, -0.5f}, true), box = Body(Polygon::MakeBox(1, 1), {0, 0.6f});
        const auto m = CheckCollision(floor.get(), box.get(), 0.2f);
        REQUIRE(m.hasCollision);
        REQUIRE(m.contactCount == 2);
        REQUIRE(m.contacts[0].penetration == Approx(-0.1f));
        REQUIRE(m.contacts[1].penetration == Approx(-0.1f));
        REQUIRE(m.normal.y == Approx(1));
    }
    SECTION("corner to corner uses the true gap") {
        // Face separation along either axis is 0.1, but the corners are
        // 0.1 * sqrt(2) apart along the diagonal.
        auto a = Body(Polygon::MakeBox(1, 1), {0, 0}), b = Body(Polygon::MakeBox(1, 1), {1.1f, 1.1f});
        REQUIRE_FALSE(CheckCollision(a.get(), b.get(), 0.14f).hasCollision);
        const auto m = CheckCollision(a.get(), b.get(), 0.2f);
        REQUIRE(m.hasCollision);
        REQUIRE(m.contactCount == 1);
        REQUIRE(m.penetration == Approx(-0.1 * std::sqrt(2.0)).margin(1e-6));
        REQUIRE(m.normal.x == Approx(std::sqrt(0.5)).margin(1e-6));
        REQUIRE(m.normal.y == Approx(std::sqrt(0.5)).margin(1e-6));
        REQUIRE(m.contacts[0].position.x == Approx(0.55f));
    }
    SECTION("capsule contacts sit midway between the real surfaces") {
        auto ball = Body(Circle(0.5f), {0, 1}), capsule = Body(Capsule(2, 0.25f), {0, 0}, true);
        REQUIRE_FALSE(CheckCollision(ball.get(), capsule.get()).hasCollision);
        const auto m = CheckCollision(ball.get(), capsule.get(), 0.5f);
        REQUIRE(m.hasCollision);
        REQUIRE(m.penetration == Approx(-0.25f));
        REQUIRE(m.contacts[0].position.y == Approx(0.375f));
    }
}

TEST_CASE("Broad phases grow bounds by each body's margin", "[speculative][BroadPhase]") {
    auto a = Body(Circle(0.5f), {0, 0}), b = Body(Circle(0.5f), {1.5f, 0});
    const std::vector<RigidBodyPtr> bodies{a, b};
    SweepAndPrune sap;
    UniformGrid grid(1.0f);
    for (IBroadPhase* phase : {static_cast<IBroadPhase*>(&sap), static_cast<IBroadPhase*>(&grid)}) {
        REQUIRE(phase->FindPotentialCollisions(bodies).empty());
        REQUIRE(phase->FindSpeculativeCollisions(bodies, {0.2f, 0.2f}).empty());
        REQUIRE(phase->FindSpeculativeCollisions(bodies, {0.3f, 0.3f}).size() == 1);
        REQUIRE_THROWS_AS(phase->FindSpeculativeCollisions(bodies, {0.3f}), std::invalid_argument);
    }
}
