#include "catch_amalgamated.hpp"
#include "../include/physics/core/world.h"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/core/shape.h"
#include "../include/physics/math/vector2.h"
#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"
#include "physics/core/collisions/narrow_phase/collision_circle_polygon.h"
#include "physics/core/collisions/collision_dispatcher.h"
#include "physics/core/forces/gravity.h"
#include <memory>
#include <iostream>

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

TEST_CASE("Polygon-Polygon Collision", "[collision]") {
    SECTION("Two polygons colliding head-on") {
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

    SECTION("Off-center collision causes rotation") {
        World world;
        auto rectangleShape = Polygon::MakeBox(1.0f, 1.0f);
        Material material = {1.0f, 0.5f};

        auto bodyA = std::make_shared<RigidBody>(&rectangleShape, material, Vector2(0.0f, 0.0f));
        auto bodyB = std::make_shared<RigidBody>(&rectangleShape, material, Vector2(0.5f, 0.4f));
        bodyB->SetVelocity(Vector2(-1.0f, 0.0f));

        // Manually check narrow phase first to be sure
        CollisionManifold manifold = PhysicsEngine::CollisionPolygonPolygon(bodyA.get(), bodyB.get());
        REQUIRE(manifold.hasCollision == true);
        REQUIRE(std::abs(manifold.normal.x) > 0.9f); // Should be horizontal-ish

        world.addBody(bodyA);
        world.addBody(bodyB);

        world.step(0.01f);

        // At least one body should have rotating (the one hit off-center)
        REQUIRE(std::abs(bodyA->GetAngularVelocity()) + std::abs(bodyB->GetAngularVelocity()) > 0.01f);
    }
}

TEST_CASE("Circle-Polygon Collision Detection", "[collision]") {
    SECTION("Circle directly above polygon detects collision") {
        auto groundShape = Polygon::MakeBox(10.0f, 1.0f);
        Circle circleShape(0.5f);
        Material mat = {1.0f, 0.5f};

        // Ground centered at (0, 0), top face at y=0.5
        // Circle at (0, 0.8), bottom at y=0.3 - overlapping by 0.2
        RigidBody ground(&groundShape, mat, Vector2(0.0f, 0.0f), true);
        RigidBody circle(&circleShape, mat, Vector2(0.0f, 0.8f));

        CollisionManifold manifold = CollisionCirclePolygon(&circle, &ground);
        
        INFO("hasCollision: " << manifold.hasCollision);
        INFO("penetration: " << manifold.penetration);
        INFO("normal: (" << manifold.normal.x << ", " << manifold.normal.y << ")");
        INFO("contactPoint: (" << manifold.contactPoint.x << ", " << manifold.contactPoint.y << ")");
        
        REQUIRE(manifold.hasCollision == true);
        REQUIRE(manifold.penetration == Catch::Approx(0.2f).margin(0.05f));
    }

    SECTION("Circle-Polygon normal direction is correct") {
        auto groundShape = Polygon::MakeBox(10.0f, 1.0f);
        Circle circleShape(0.5f);
        Material mat = {1.0f, 0.5f};

        // Circle above polygon
        RigidBody ground(&groundShape, mat, Vector2(0.0f, 0.0f), true);
        RigidBody circle(&circleShape, mat, Vector2(0.0f, 0.8f));

        CollisionManifold manifold = CollisionCirclePolygon(&circle, &ground);
        
        INFO("normal: (" << manifold.normal.x << ", " << manifold.normal.y << ")");
        REQUIRE(manifold.hasCollision == true);
        
        // Normal should point from circle (A) toward polygon (B), i.e., downward
        REQUIRE(manifold.normal.y < -0.9f);
    }

    SECTION("Circle does not fall through ground with gravity") {
        World world;
        auto groundShape = Polygon::MakeBox(20.0f, 1.0f);
        Circle circleShape(0.5f);
        Material mat = {1.0f, 0.3f};

        auto ground = std::make_shared<RigidBody>(&groundShape, mat, Vector2(0.0f, -8.0f), true);
        auto circle = std::make_shared<RigidBody>(&circleShape, mat, Vector2(0.0f, -6.0f));

        world.addBody(ground);
        world.addBody(circle);
        world.addUniversalForce(std::make_unique<Gravity>(Vector2(0.0f, -9.81f)));

        float initialY = circle->GetPosition().y;
        float groundTopY = -8.0f + 0.5f; // ground center + half-height

        // Run simulation for 5 seconds (300 frames at 1/60)
        for (int i = 0; i < 300; ++i) {
            world.step(1.0f / 60.0f);
        }

        float finalY = circle->GetPosition().y;
        float circleBottomY = finalY - 0.5f; // center - radius

        INFO("Initial Y: " << initialY);
        INFO("Final Y: " << finalY);
        INFO("Circle bottom Y: " << circleBottomY);
        INFO("Ground top Y: " << groundTopY);
        
        // Circle bottom should be AT or ABOVE the ground surface, not below it
        REQUIRE(circleBottomY >= groundTopY - 0.5f); // Allow some penetration tolerance
    }

    SECTION("Circle-Polygon via dispatcher") {
        auto groundShape = Polygon::MakeBox(10.0f, 1.0f);
        Circle circleShape(0.5f);
        Material mat = {1.0f, 0.5f};

        RigidBody ground(&groundShape, mat, Vector2(0.0f, 0.0f), true);
        RigidBody circle(&circleShape, mat, Vector2(0.0f, 0.8f));

        // Test via the dispatcher (which handles type ordering)
        CollisionManifold manifold = CheckCollision(&circle, &ground);
        
        INFO("Via dispatcher - hasCollision: " << manifold.hasCollision);
        INFO("Via dispatcher - penetration: " << manifold.penetration);
        INFO("Via dispatcher - normal: (" << manifold.normal.x << ", " << manifold.normal.y << ")");
        
        REQUIRE(manifold.hasCollision == true);

        // Also test with reversed order
        CollisionManifold manifold2 = CheckCollision(&ground, &circle);
        
        INFO("Reversed - hasCollision: " << manifold2.hasCollision);
        INFO("Reversed - penetration: " << manifold2.penetration);
        INFO("Reversed - normal: (" << manifold2.normal.x << ", " << manifold2.normal.y << ")");
        
        REQUIRE(manifold2.hasCollision == true);
    }
}
