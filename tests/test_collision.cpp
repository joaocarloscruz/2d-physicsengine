#include "catch_amalgamated.hpp"
#include "../include/physics/core/world.h"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/core/shape.h"
#include "../include/physics/math/vector2.h"
#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"
#include <memory>

using namespace Catch;
using namespace PhysicsEngine;

TEST_CASE("Polygon-Polygon SAT Intersection", "[collision]") {
    // Create two 2x2 squares
    auto polyShape = Polygon::MakeBox(2.0f, 2.0f); // Use the static helper
    Material mat = {1.0f, 0.5f}; // Density 1, Restitution 0.5

    SECTION("Detects overlap correctly") {
        RigidBody a(&polyShape, mat, Vector2(0, 0));
        RigidBody b(&polyShape, mat, Vector2(1.5f, 0)); // Overlapping by 0.5

        CollisionManifold manifold = PhysicsEngine::CollisionPolygonPolygon(&a, &b);

        REQUIRE(manifold.hasCollision == true);
        REQUIRE(manifold.penetration == Catch::Approx(0.5f));
        
        // Normal should point from A to B (so approx (1, 0))
        REQUIRE(manifold.normal.x == Catch::Approx(1.0f));
        REQUIRE(manifold.normal.y == Catch::Approx(0.0f));
    }

    SECTION("Calculates valid contact point") {
        RigidBody a(&polyShape, mat, Vector2(0, 0));
        RigidBody b(&polyShape, mat, Vector2(1.5f, 0));
        CollisionManifold manifold = PhysicsEngine::CollisionPolygonPolygon(&a, &b);
    
        // The contact point should be the deepest point of the incident body.
        // Incident is B (left edge). Deepest point is (-1, 0) relative to B, which is x=0.5 in World.
    
        // Just check if the point is reasonable (between the shapes)
        REQUIRE(manifold.contactPoint.x == Catch::Approx(0.5f).margin(0.1f));
    }

    SECTION("Detects separation correctly") {
        RigidBody a(&polyShape, mat, Vector2(0, 0));
        RigidBody b(&polyShape, mat, Vector2(2.1f, 0)); // Gap of 0.1

        CollisionManifold manifold = PhysicsEngine::CollisionPolygonPolygon(&a, &b);

        REQUIRE(manifold.hasCollision == false);
    }

    SECTION("Normal flips correctly based on relative position") {
        RigidBody a(&polyShape, mat, Vector2(0, 0));
        RigidBody b(&polyShape, mat, Vector2(-1.5f, 0)); // B is to the LEFT now

        CollisionManifold manifold = PhysicsEngine::CollisionPolygonPolygon(&a, &b);

        REQUIRE(manifold.hasCollision == true);
        // Normal should point from A to B (so approx (-1, 0))
        REQUIRE(manifold.normal.x == Catch::Approx(-1.0f));
    }
}

/*
TEST_CASE("Circle-Circle Collision", "[collision]") {
    SECTION("Two circles colliding head-on") {
        World world;
        Circle circleShape(1.0f);
        Material material = {1.0f, 0.5f};

        auto bodyA = std::make_shared<RigidBody>(&circleShape, material, Vector2(0.0f, 0.0f));
        bodyA->SetVelocity(Vector2(1.0f, 0.0f));

        auto bodyB = std::make_shared<RigidBody>(&circleShape, material, Vector2(1.5f, 0.0f));
        bodyB->SetVelocity(Vector2(-1.0f, 0.0f));

        world.addBody(bodyA);
        world.addBody(bodyB);

        // Let the simulation run for a short time
        world.step(0.1f);

        // Assertions to check the result of the collision
        // For a head-on collision of equal mass, they should reverse velocity
        REQUIRE(bodyA->GetVelocity().x < 0.0f);
        REQUIRE(bodyB->GetVelocity().x > 0.0f);

        // They should also have moved apart
        REQUIRE(bodyB->GetPosition().x > bodyA->GetPosition().x);
    }
}

TEST_CASE("Rectangle-Rectangle Collision", "[collision]") {
    SECTION("Two rectangles colliding head-on") {
        World world;
        auto rectangleShape = Polygon::MakeBox(1.0f, 1.0f);
        Material material = {1.0f, 0.5f};

        auto bodyA = std::make_shared<RigidBody>(&rectangleShape, material, Vector2(0.0f, 0.0f));
        bodyA->SetVelocity(Vector2(1.0f, 0.0f));

        auto bodyB = std::make_shared<RigidBody>(&rectangleShape, material, Vector2(0.5f, 0.0f));
        bodyB->SetVelocity(Vector2(-1.0f, 0.0f));

        world.addBody(bodyA);
        world.addBody(bodyB);

        // Let the simulation run for a short time
        world.step(0.1f);

        // Assertions to check the result of the collision
        // For a head-on collision of equal mass, they should reverse velocity
        REQUIRE(bodyA->GetVelocity().x < 0.0f);
        REQUIRE(bodyB->GetVelocity().x > 0.0f);

        // They should also have moved apart
        REQUIRE(bodyB->GetPosition().x > bodyA->GetPosition().x);
    }
}
*/