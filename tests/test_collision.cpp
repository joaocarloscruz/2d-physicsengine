#include "catch_amalgamated.hpp"
#include "../include/physics/core/world.h"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/core/shape.h"
#include "../include/physics/math/vector2.h"
#include <memory>

using namespace Catch;
using namespace PhysicsEngine;

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
        Rectangle rectangleShape(1.0f, 1.0f);
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