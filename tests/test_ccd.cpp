#include "catch_amalgamated.hpp"
#include "physics/core/world.h"
#include "physics/core/collisions/continuous_collision.h"
#include <algorithm>

using namespace PhysicsEngine;

TEST_CASE("Swept circles report analytical time of impact", "[ccd]") {
    const auto hit = SweepCircleCircle({-5, 0}, {10, 0}, 1, {5, 0}, {-10, 0}, 1);
    REQUIRE(hit.hit);
    REQUIRE(hit.fraction == Catch::Approx(0.4f));
    REQUIRE(hit.normal.x == Catch::Approx(1));
    REQUIRE_FALSE(SweepCircleCircle({0, 0}, {-10, 0}, 1, {5, 0}, {}, 1).hit);
    REQUIRE_FALSE(SweepCircleCircle({0, 0}, {}, 1, {5, 0}, {}, 1).hit);
    REQUIRE(SweepCircleCircle({0, 0}, {}, 1, {1, 0}, {}, 1).fraction == 0);
}

TEST_CASE("Circle polygon sweep handles faces corners and either winding", "[ccd]") {
    auto vertices = Polygon::MakeBox(2, 2).getVertices();
    for (int winding=0; winding<2; ++winding) {
        const auto face = SweepCirclePolygon({-5, 0}, {10, 0}, 0.5f, vertices);
        REQUIRE(face.hit);
        REQUIRE(face.fraction == Catch::Approx(0.35f));
        const auto corner = SweepCirclePolygon({-5, -5}, {10, 10}, 1, vertices);
        REQUIRE(corner.hit);
        REQUIRE(corner.fraction == Catch::Approx((4-std::sqrt(0.5f))/10));
        REQUIRE_FALSE(SweepCirclePolygon({-5, 3}, {10, 0}, 0.5f, vertices).hit);
        REQUIRE(SweepCirclePolygon({0, 0}, {}, 0.5f, vertices).fraction == 0);
        std::reverse(vertices.begin(), vertices.end());
    }
}

TEST_CASE("Fast circles bounce off thin walls without tunneling", "[ccd]") {
    SimulationConfig config;
    config.enableLinearVelocityLimit = false;
    World world(config);
    auto ball = std::make_shared<RigidBody>(Circle(0.1f), Material{1, 1, 0, 0}, Vector2(-5, 0));
    auto wall = std::make_shared<RigidBody>(Polygon::MakeBox(0.1f, 10), Material{1, 1, 0, 0}, Vector2{}, true);
    ball->SetVelocity({100, 0});
    world.addBody(ball); world.addBody(wall);
    SECTION("enabled") {
        ball->SetCcdEnabled(true);
        world.step(0.1f);
        REQUIRE(ball->velocity.x == Catch::Approx(-100));
        REQUIRE(ball->position.x == Catch::Approx(-5.3f).margin(0.001f));
        REQUIRE(world.getLastStepStatistics().ccdImpactCount == 1);
    }
    SECTION("disabled") {
        world.step(0.1f);
        REQUIRE(ball->position.x == Catch::Approx(5));
        REQUIRE(world.getLastStepStatistics().ccdImpactCount == 0);
    }
    SECTION("filtered") {
        ball->SetCcdEnabled(true); wall->SetCollisionMaskBits(0);
        world.step(0.1f);
        REQUIRE(ball->position.x == Catch::Approx(5));
    }
}

TEST_CASE("CCD transfers momentum between moving circles", "[ccd]") {
    SimulationConfig config; config.enableLinearVelocityLimit = false;
    World world(config);
    auto a = std::make_shared<RigidBody>(Circle(1), Material{1, 1, 0, 0}, Vector2(-5, 0));
    auto b = std::make_shared<RigidBody>(Circle(1), Material{1, 1, 0, 0}, Vector2(5, 0));
    a->SetVelocity({100, 0}); b->SetVelocity({-100, 0}); a->SetCcdEnabled(true);
    world.addBody(a); world.addBody(b); world.step(0.1f);
    REQUIRE(a->velocity.x == Catch::Approx(-100));
    REQUIRE(b->velocity.x == Catch::Approx(100));
    REQUIRE(a->position.x == Catch::Approx(-7).margin(0.001f));
    REQUIRE(b->position.x == Catch::Approx(7).margin(0.001f));
}

TEST_CASE("CCD handles repeated impacts and reports budget exhaustion", "[ccd]") {
    SimulationConfig config; config.enableLinearVelocityLimit = false;
    SECTION("enough budget") { config.maximumCcdImpacts = 32; }
    SECTION("limited budget") { config.maximumCcdImpacts = 1; }
    World world(config);
    auto ball = std::make_shared<RigidBody>(Circle(0.1f), Material{1, 1, 0, 0});
    ball->SetVelocity({100, 0}); ball->SetCcdEnabled(true);
    world.addBody(ball);
    for (float x : {-1.0f, 1.0f}) world.addBody(std::make_shared<RigidBody>(
        Polygon::MakeBox(0.1f, 10), Material{1, 1, 0, 0}, Vector2(x, 0), true));
    world.step(0.1f);
    REQUIRE(std::abs(ball->position.x) <= 0.851f);
    REQUIRE(world.getLastStepStatistics().ccdIterationLimitReached == (config.maximumCcdImpacts == 1));
    if (config.maximumCcdImpacts > 1) REQUIRE(world.getLastStepStatistics().ccdImpactCount > 1);
}
