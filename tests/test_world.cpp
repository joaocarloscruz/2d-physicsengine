#include "catch_amalgamated.hpp"
#include "../include/physics/core/world.h"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/core/shape.h"
#include "../include/physics/math/vector2.h"
#include "../include/physics/core/forces/gravity.h"
#include <memory>

using namespace Catch;
using namespace PhysicsEngine;

TEST_CASE("World operations are correct", "[World]") {
    World world;
    Circle circle(1.0f);
    Material material = {1.0f, 0.5f};

    SECTION("Add and Remove Body") {
        auto body = std::make_shared<RigidBody>(&circle, material, Vector2(0.0f, 0.0f));
        world.addBody(body);
        REQUIRE(world.getBodies().size() == 1);
        REQUIRE(world.getBodies()[0] == body);

        world.removeBody(body);
        REQUIRE(world.getBodies().empty());
    }

    SECTION("Step") {
        auto body = std::make_shared<RigidBody>(&circle, material, Vector2(0.0f, 0.0f));
        body->SetVelocity(Vector2(1.0f, 2.0f));
        world.addBody(body);

        world.step(0.1f);

        REQUIRE(body->GetPosition().x == Approx(0.1f));
        REQUIRE(body->GetPosition().y == Approx(0.2f));

        auto staticBody = std::make_shared<RigidBody>(&circle, material, Vector2(5.0f, 5.0f), true);
        world.addBody(staticBody);
        world.step(0.1f);

        REQUIRE(staticBody->GetPosition().x == Approx(5.0f));
        REQUIRE(staticBody->GetPosition().y == Approx(5.0f));
    }

    SECTION("Force") {
        auto body = std::make_shared<RigidBody>(&circle, material, Vector2(0.0f, 0.0f));
        world.addBody(body);

        auto gravity = std::make_unique<Gravity>(Vector2(0.0f, -9.8f));
        world.addUniversalForce(std::move(gravity));

        world.step(0.1f);

        REQUIRE(body->GetVelocity().y == Approx(-0.98f));
        REQUIRE(body->GetPosition().y == Approx(-0.049f));
    }

    SECTION("Broad Phase"){
        auto bodyA = std::make_shared<RigidBody>(&circle, material, Vector2(0.0f, 0.0f));
        auto bodyB = std::make_shared<RigidBody>(&circle, material, Vector2(1.5f, 0.0f));
        auto bodyC = std::make_shared<RigidBody>(&circle, material, Vector2(5.0f, 5.0f));

        world.addBody(bodyA);
        world.addBody(bodyB);
        world.addBody(bodyC);

        AABB aabbA = bodyA->GetAABB();
        AABB aabbB = bodyB->GetAABB();
        AABB aabbC = bodyC->GetAABB();

        REQUIRE(aabbA.IsOverlapping(aabbB) == true);
        REQUIRE(aabbA.IsOverlapping(aabbC) == false);
        REQUIRE(aabbB.IsOverlapping(aabbC) == false);
    }

    SECTION("Broad Phase finds correct pairs in step()") {
        World world;
        Circle circleA(1.0f);
        Circle circleB(1.0f);
        Circle circleC(1.0f);

        auto bodyA = std::make_shared<RigidBody>(&circleA, material, Vector2(0.0f, 0.0f));
        auto bodyB = std::make_shared<RigidBody>(&circleB, material, Vector2(1.5f, 0.0f));
        auto bodyC = std::make_shared<RigidBody>(&circleC, material, Vector2(5.0f, 5.0f));

        world.addBody(bodyA);
        world.addBody(bodyB);
        world.addBody(bodyC);

        world.step(0.0f); // delta time at 0 so bodies are static.

        
        REQUIRE(world.getPotentialCollisions().size() == 1);
        
        const auto& pairs = world.getPotentialCollisions();
        bool correctPair = (pairs[0].first == bodyA && pairs[0].second == bodyB) || (pairs[0].first == bodyB && pairs[0].second == bodyA);
        REQUIRE(correctPair);
    }

    SECTION("Two rectangles overlapping in AABB") {
        World world;
        auto rect1 = Polygon::MakeBox(2.0f, 2.0f);
        auto rect2 = Polygon::MakeBox(2.0f, 2.0f);

        // Create two bodies that should definitely overlap
        auto bodyA = std::make_shared<RigidBody>(&rect1, material, Vector2(0.0f, 0.0f));
        auto bodyB = std::make_shared<RigidBody>(&rect2, material, Vector2(1.5f, 0.0f));

        world.addBody(bodyA);
        world.addBody(bodyB);

        // Call step() to trigger the broad phase
        world.step(0.0f); 

        // Get the potential collisions via your new getter
        REQUIRE(world.getPotentialCollisions().size() == 1);

        // Verify the pair is correct
        const auto& pairs = world.getPotentialCollisions();
        bool correctPairFound = false;
        if (!pairs.empty()) {
            if ((pairs[0].first == bodyA && pairs[0].second == bodyB) ||
                (pairs[0].first == bodyB && pairs[0].second == bodyA)) {
                correctPairFound = true;
            }
        }
        REQUIRE(correctPairFound);
    }
}