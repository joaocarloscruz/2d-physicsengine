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

TEST_CASE("World persists active contact impulses", "[World][warm_start]") {
    World world;
    auto floorShape = Polygon::MakeBox(10.0f, 1.0f);
    Circle circleShape(0.5f);
    Material material = {1.0f, 0.0f, 0.6f, 0.4f};

    auto floor = std::make_shared<RigidBody>(
        &floorShape,
        material,
        Vector2(0.0f, -0.5f),
        true
    );
    auto circle = std::make_shared<RigidBody>(
        &circleShape,
        material,
        Vector2(0.0f, 0.49f)
    );
    circle->SetVelocity(Vector2(0.0f, -1.0f));

    world.addBody(floor);
    world.addBody(circle);
    world.step(1.0f / 60.0f);

    REQUIRE(world.getPersistentContactCount() == 1);

    world.removeBody(circle);
    REQUIRE(world.getPersistentContactCount() == 0);
}

#include <functional>
#include "physics/core/fixed_step_runner.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
namespace {
struct CallbackForce : IForceGenerator {
    std::function<void()> callback;
    explicit CallbackForce(std::function<void()> value) : callback(std::move(value)) {}
    void applyForce(RigidBody*) override { callback(); }
};
}

TEST_CASE("Force callbacks cannot mutate world collections during stepping", "[review][World]") {
    World world;
    auto body = std::make_shared<RigidBody>(Circle(1), Material{});
    world.addBody(body);
    auto extra = std::make_shared<RigidBody>(Circle(1), Material{}, Vector2(10, 0));
    world.addForce(body, std::make_unique<CallbackForce>([&] {
        REQUIRE_THROWS_AS(world.addBody(extra), std::logic_error);
    }));
    world.step(0.01f);
    REQUIRE(world.getBodies().size() == 1);
    world.addBody(extra);
    REQUIRE(world.getBodies().size() == 2);
}

TEST_CASE("World mutation guards cover all structural operations and recover from exceptions", "[review][World]") {
    World world;
    auto a = std::make_shared<RigidBody>(Circle(1), Material{});
    auto b = std::make_shared<RigidBody>(Circle(1), Material{}, Vector2(5, 0));
    world.addBody(a); world.addBody(b);
    auto joint = std::make_shared<DistanceJoint>(a, b, 5);
    world.addJoint(joint);
    auto system = std::make_shared<ParticleSystem>(); world.addParticleSystem(system);
    bool fail = true;
    world.addForce(a, std::make_unique<CallbackForce>([&] {
        REQUIRE_THROWS_AS(world.removeBody(a), std::logic_error);
        REQUIRE_THROWS_AS(world.clearBodies(), std::logic_error);
        REQUIRE_THROWS_AS(world.addForce(a, std::make_unique<CallbackForce>([] {})), std::logic_error);
        REQUIRE_THROWS_AS(world.addUniversalForce(std::make_unique<CallbackForce>([] {})), std::logic_error);
        REQUIRE_THROWS_AS(world.addParticleSystem(system), std::logic_error);
        REQUIRE_THROWS_AS(world.removeParticleSystem(system), std::logic_error);
        REQUIRE_THROWS_AS(world.clearParticleSystems(), std::logic_error);
        REQUIRE_THROWS_AS(world.addJoint(joint), std::logic_error);
        REQUIRE_THROWS_AS(world.removeJoint(joint), std::logic_error);
        REQUIRE_THROWS_AS(world.setBroadPhase(std::make_unique<SweepAndPrune>()), std::logic_error);
        REQUIRE_THROWS_AS(world.setSimulationConfig(SimulationConfig{}), std::logic_error);
        if (fail) throw std::runtime_error("force callback failed");
    }));
    FixedStepRunner runner(world);
    REQUIRE_THROWS_AS(runner.advance(0.02), std::runtime_error);
    fail = false;
    REQUIRE_NOTHROW(runner.reset());
    REQUIRE_NOTHROW(world.step(0.01f));
    REQUIRE_NOTHROW(world.clearBodies());
    REQUIRE(world.getBodies().empty());
}
