#include "catch_amalgamated.hpp"

#include "physics/core/collisions/collision_dispatcher.h"
#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/collisions/narrow_phase/collision_circle_circle.h"
#include "physics/core/collisions/narrow_phase/collision_circle_polygon.h"
#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"
#include "physics/core/material.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"

#include <cmath>

using namespace PhysicsEngine;

TEST_CASE("Concentric circles report their full overlap", "[collision][property]") {
    Circle firstShape(2.0f);
    Circle secondShape(3.0f);
    Material material{1.0f, 0.0f};
    RigidBody first(&firstShape, material);
    RigidBody second(&secondShape, material);

    const CollisionManifold manifold = CollisionCircleCircle(&first, &second);

    REQUIRE(manifold.hasCollision);
    REQUIRE(manifold.penetration == Catch::Approx(5.0f));
    REQUIRE(manifold.normal.magnitude() == Catch::Approx(1.0f));
}

TEST_CASE("Circle collision is symmetric", "[collision][property]") {
    Circle firstShape(2.0f);
    Circle secondShape(1.0f);
    Material material{1.0f, 0.0f};
    RigidBody first(&firstShape, material, Vector2(-0.5f, 1.0f));
    RigidBody second(&secondShape, material, Vector2(1.0f, 1.5f));

    const CollisionManifold forward = CollisionCircleCircle(&first, &second);
    const CollisionManifold reverse = CollisionCircleCircle(&second, &first);

    REQUIRE(forward.hasCollision == reverse.hasCollision);
    REQUIRE(forward.penetration == Catch::Approx(reverse.penetration));
    REQUIRE(forward.normal.x == Catch::Approx(-reverse.normal.x));
    REQUIRE(forward.normal.y == Catch::Approx(-reverse.normal.y));
    REQUIRE(forward.normal.magnitude() == Catch::Approx(1.0f));
}

TEST_CASE("Separated and exactly touching circles have no penetration", "[collision][boundary]") {
    Circle shape(1.0f);
    Material material{1.0f, 0.0f};
    RigidBody first(&shape, material, Vector2(0.0f, 0.0f));
    RigidBody touching(&shape, material, Vector2(2.0f, 0.0f));
    RigidBody separated(&shape, material, Vector2(2.001f, 0.0f));

    REQUIRE_FALSE(CollisionCircleCircle(&first, &touching).hasCollision);
    REQUIRE_FALSE(CollisionCircleCircle(&first, &separated).hasCollision);
}

TEST_CASE("Polygon collision is symmetric for rotated boxes", "[collision][property]") {
    auto firstShape = Polygon::MakeBox(3.0f, 1.0f);
    auto secondShape = Polygon::MakeBox(2.0f, 2.0f);
    Material material{1.0f, 0.0f};
    RigidBody first(&firstShape, material, Vector2(0.0f, 0.0f));
    RigidBody second(&secondShape, material, Vector2(1.0f, 0.2f));
    first.SetOrientation(0.35f);
    second.SetOrientation(-0.2f);

    const CollisionManifold forward = CollisionPolygonPolygon(&first, &second);
    const CollisionManifold reverse = CollisionPolygonPolygon(&second, &first);

    REQUIRE(forward.hasCollision);
    REQUIRE(reverse.hasCollision);
    REQUIRE(forward.penetration == Catch::Approx(reverse.penetration).margin(1e-4f));
    REQUIRE(forward.normal.x == Catch::Approx(-reverse.normal.x).margin(1e-4f));
    REQUIRE(forward.normal.y == Catch::Approx(-reverse.normal.y).margin(1e-4f));
}

TEST_CASE("Separated rotated boxes do not collide", "[collision][boundary]") {
    auto shape = Polygon::MakeBox(2.0f, 1.0f);
    Material material{1.0f, 0.0f};
    RigidBody first(&shape, material, Vector2(-2.0f, 0.0f));
    RigidBody second(&shape, material, Vector2(2.0f, 0.0f));
    first.SetOrientation(0.7f);
    second.SetOrientation(-0.7f);

    REQUIRE_FALSE(CollisionPolygonPolygon(&first, &second).hasCollision);
}

TEST_CASE("Exactly touching polygons have no penetration", "[collision][boundary]") {
    auto shape = Polygon::MakeBox(2.0f, 2.0f);
    Material material{1.0f, 0.0f};
    RigidBody first(&shape, material, Vector2(0.0f, 0.0f));
    RigidBody second(&shape, material, Vector2(2.0f, 0.0f));

    REQUIRE_FALSE(CollisionPolygonPolygon(&first, &second).hasCollision);
}

TEST_CASE("Exactly touching circle and polygon have no penetration", "[collision][boundary]") {
    Circle circleShape(0.5f);
    auto boxShape = Polygon::MakeBox(2.0f, 2.0f);
    Material material{1.0f, 0.0f};
    RigidBody circle(&circleShape, material, Vector2(1.5f, 0.0f));
    RigidBody box(&boxShape, material);

    REQUIRE_FALSE(CollisionCirclePolygon(&circle, &box).hasCollision);
}

TEST_CASE("Circle-polygon dispatch is symmetric", "[collision][property]") {
    Circle circleShape(0.75f);
    auto boxShape = Polygon::MakeBox(2.0f, 2.0f);
    Material material{1.0f, 0.0f};
    RigidBody circle(&circleShape, material, Vector2(1.2f, 0.2f));
    RigidBody box(&boxShape, material, Vector2(0.0f, 0.0f));
    box.SetOrientation(0.25f);

    const CollisionManifold forward = CheckCollision(&circle, &box);
    const CollisionManifold reverse = CheckCollision(&box, &circle);

    REQUIRE(forward.hasCollision);
    REQUIRE(reverse.hasCollision);
    REQUIRE(forward.A == &circle);
    REQUIRE(forward.B == &box);
    REQUIRE(reverse.A == &box);
    REQUIRE(reverse.B == &circle);
    REQUIRE(forward.penetration == Catch::Approx(reverse.penetration).margin(1e-4f));
    REQUIRE(forward.normal.x == Catch::Approx(-reverse.normal.x).margin(1e-4f));
    REQUIRE(forward.normal.y == Catch::Approx(-reverse.normal.y).margin(1e-4f));
}

TEST_CASE("Circle fully inside a box produces a finite manifold", "[collision][boundary]") {
    Circle circleShape(0.5f);
    auto boxShape = Polygon::MakeBox(4.0f, 4.0f);
    Material material{1.0f, 0.0f};
    RigidBody circle(&circleShape, material, Vector2(0.25f, -0.1f));
    RigidBody box(&boxShape, material);

    const CollisionManifold manifold = CollisionCirclePolygon(&circle, &box);

    REQUIRE(manifold.hasCollision);
    REQUIRE(std::isfinite(manifold.penetration));
    REQUIRE(manifold.penetration == Catch::Approx(2.25f).margin(1e-4f));
    REQUIRE(manifold.normal.magnitude() == Catch::Approx(1.0f).margin(1e-4f));
}

TEST_CASE("Nested polygons report minimum separating translation", "[collision][boundary]") {
    auto innerShape = Polygon::MakeBox(1.0f, 1.0f);
    auto outerShape = Polygon::MakeBox(4.0f, 4.0f);
    Material material{1.0f, 0.0f};
    RigidBody inner(&innerShape, material, Vector2(1.0f, 0.0f));
    RigidBody outer(&outerShape, material);

    const CollisionManifold manifold = CollisionPolygonPolygon(&inner, &outer);

    REQUIRE(manifold.hasCollision);
    REQUIRE(manifold.penetration == Catch::Approx(1.5f).margin(1e-4f));
    REQUIRE(manifold.normal.x < -0.9f);
}

TEST_CASE("Elastic collision with unequal masses conserves momentum and energy", "[collision][resolver][property]") {
    Circle shape(1.0f);
    Material material{1.0f, 1.0f, 0.0f, 0.0f};
    RigidBody first(&shape, material, Vector2(0.0f, 0.0f));
    RigidBody second(&shape, material, Vector2(1.5f, 0.0f));
    first.SetMass(2.0f);
    second.SetMass(5.0f);
    first.SetVelocity(Vector2(4.0f, 0.0f));
    second.SetVelocity(Vector2(-1.0f, 0.0f));
    const float momentumBefore = 2.0f * 4.0f + 5.0f * -1.0f;
    const float energyBefore = 0.5f * 2.0f * 16.0f + 0.5f * 5.0f;

    CollisionResolver::Resolve(CollisionCircleCircle(&first, &second));

    const float momentumAfter = 2.0f * first.GetVelocity().x + 5.0f * second.GetVelocity().x;
    const float energyAfter = 0.5f * 2.0f * first.GetVelocity().magnitudeSquared()
        + 0.5f * 5.0f * second.GetVelocity().magnitudeSquared();
    REQUIRE(momentumAfter == Catch::Approx(momentumBefore).margin(1e-4f));
    REQUIRE(energyAfter == Catch::Approx(energyBefore).margin(1e-3f));
}

TEST_CASE("Separating bodies receive no collision impulse", "[collision][resolver][boundary]") {
    Circle shape(1.0f);
    Material material{1.0f, 1.0f};
    RigidBody first(&shape, material, Vector2(0.0f, 0.0f));
    RigidBody second(&shape, material, Vector2(1.5f, 0.0f));
    first.SetVelocity(Vector2(-2.0f, 0.0f));
    second.SetVelocity(Vector2(2.0f, 0.0f));

    CollisionResolver::Resolve(CollisionCircleCircle(&first, &second));

    REQUIRE(first.GetVelocity().x == -2.0f);
    REQUIRE(second.GetVelocity().x == 2.0f);
}

TEST_CASE("Friction cannot increase tangential speed", "[collision][resolver][friction]") {
    auto floorShape = Polygon::MakeBox(10.0f, 1.0f);
    auto boxShape = Polygon::MakeBox(1.0f, 1.0f);
    Material material{1.0f, 0.0f, 0.8f, 0.6f};
    RigidBody floor(&floorShape, material, Vector2(0.0f, -0.5f), true);
    RigidBody box(&boxShape, material, Vector2(0.0f, 0.49f));
    box.SetVelocity(Vector2(5.0f, -1.0f));
    const float speedBefore = std::abs(box.GetVelocity().x);

    CollisionResolver::Resolve(CheckCollision(&floor, &box));

    REQUIRE(std::abs(box.GetVelocity().x) <= speedBefore);
    REQUIRE(box.GetVelocity().x >= 0.0f);
}
