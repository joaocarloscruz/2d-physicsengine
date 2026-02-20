#include "catch_amalgamated.hpp"
#include "../include/physics/core/world.h"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/core/shape.h"
#include "../include/physics/math/vector2.h"
#include "../include/physics/core/collisions/collision_listener.h"
#include "../include/physics/core/collisions/collision_manifold.h"
#include <memory>

using namespace PhysicsEngine;

// -----------------------------------------------------------------------
// Test helper: a concrete listener that counts calls and stores manifolds
// -----------------------------------------------------------------------
class TestListener : public ICollisionListener {
public:
    int callCount = 0;
    std::vector<CollisionManifold> received;

    void onCollision(const CollisionManifold& manifold) override {
        ++callCount;
        received.push_back(manifold);
    }
};

// -----------------------------------------------------------------------
// Tests
// -----------------------------------------------------------------------

TEST_CASE("Listener is called when a collision occurs", "[collision_listener]") {
    World world;
    Circle circleShape(1.0f);
    Material material = {1.0f, 0.5f};

    // Two overlapping circles
    auto bodyA = std::make_shared<RigidBody>(&circleShape, material, Vector2(0.0f, 0.0f));
    auto bodyB = std::make_shared<RigidBody>(&circleShape, material, Vector2(1.5f, 0.0f));
    world.addBody(bodyA);
    world.addBody(bodyB);

    TestListener listener;
    world.addCollisionListener(&listener);

    world.step(1.0f / 60.0f);

    REQUIRE(listener.callCount >= 1);
}

TEST_CASE("Listener is NOT called when no collision occurs", "[collision_listener]") {
    World world;
    Circle circleShape(1.0f);
    Material material = {1.0f, 0.5f};

    // Two bodies far apart — no collision
    auto bodyA = std::make_shared<RigidBody>(&circleShape, material, Vector2(0.0f, 0.0f));
    auto bodyB = std::make_shared<RigidBody>(&circleShape, material, Vector2(10.0f, 0.0f));
    world.addBody(bodyA);
    world.addBody(bodyB);

    TestListener listener;
    world.addCollisionListener(&listener);

    world.step(1.0f / 60.0f);

    REQUIRE(listener.callCount == 0);
}

TEST_CASE("Listener receives correct body references in manifold", "[collision_listener]") {
    World world;
    Circle circleShape(1.0f);
    Material material = {1.0f, 0.5f};

    auto bodyA = std::make_shared<RigidBody>(&circleShape, material, Vector2(0.0f, 0.0f));
    auto bodyB = std::make_shared<RigidBody>(&circleShape, material, Vector2(1.5f, 0.0f));
    world.addBody(bodyA);
    world.addBody(bodyB);

    TestListener listener;
    world.addCollisionListener(&listener);

    world.step(1.0f / 60.0f);

    REQUIRE(listener.callCount >= 1);
    const CollisionManifold& m = listener.received[0];

    // Both bodies in the manifold must be the ones we added
    bool bodiesCorrect = (m.A == bodyA.get() && m.B == bodyB.get()) ||
                         (m.A == bodyB.get() && m.B == bodyA.get());
    REQUIRE(bodiesCorrect);
}

TEST_CASE("Listener fires once per collision per step, not once per solver iteration", "[collision_listener]") {
    // The world runs 10 solver iterations internally.
    // Listeners should only be called once per step for each collision pair.
    World world;
    Circle circleShape(1.0f);
    Material material = {1.0f, 0.5f};

    auto bodyA = std::make_shared<RigidBody>(&circleShape, material, Vector2(0.0f, 0.0f));
    auto bodyB = std::make_shared<RigidBody>(&circleShape, material, Vector2(1.5f, 0.0f));
    world.addBody(bodyA);
    world.addBody(bodyB);

    TestListener listener;
    world.addCollisionListener(&listener);

    world.step(1.0f / 60.0f);

    // Must be exactly 1 — the 10 solver iterations must not fire it 10 times
    REQUIRE(listener.callCount == 1);
}

TEST_CASE("Listener fires once per step over multiple steps", "[collision_listener]") {
    World world;
    Circle circleShape(1.0f);
    Material material = {1.0f, 0.5f};

    auto bodyA = std::make_shared<RigidBody>(&circleShape, material, Vector2(0.0f, 0.0f));
    auto bodyB = std::make_shared<RigidBody>(&circleShape, material, Vector2(1.5f, 0.0f));
    world.addBody(bodyA);
    world.addBody(bodyB);

    TestListener listener;
    world.addCollisionListener(&listener);

    world.step(1.0f / 60.0f);
    world.step(1.0f / 60.0f);
    world.step(1.0f / 60.0f);

    // 3 steps, bodies may still be overlapping on each — at most 3 callbacks
    // (and at least 1 since they definitely collide on the first step)
    REQUIRE(listener.callCount >= 1);
    REQUIRE(listener.callCount <= 3);
}

TEST_CASE("Removed listener is no longer called", "[collision_listener]") {
    World world;
    Circle circleShape(1.0f);
    Material material = {1.0f, 0.5f};

    auto bodyA = std::make_shared<RigidBody>(&circleShape, material, Vector2(0.0f, 0.0f));
    auto bodyB = std::make_shared<RigidBody>(&circleShape, material, Vector2(1.5f, 0.0f));
    world.addBody(bodyA);
    world.addBody(bodyB);

    TestListener listener;
    world.addCollisionListener(&listener);

    // Remove before stepping
    world.removeCollisionListener(&listener);

    world.step(1.0f / 60.0f);

    REQUIRE(listener.callCount == 0);
}

TEST_CASE("Multiple listeners all receive the same collision event", "[collision_listener]") {
    World world;
    Circle circleShape(1.0f);
    Material material = {1.0f, 0.5f};

    auto bodyA = std::make_shared<RigidBody>(&circleShape, material, Vector2(0.0f, 0.0f));
    auto bodyB = std::make_shared<RigidBody>(&circleShape, material, Vector2(1.5f, 0.0f));
    world.addBody(bodyA);
    world.addBody(bodyB);

    TestListener listenerA;
    TestListener listenerB;
    world.addCollisionListener(&listenerA);
    world.addCollisionListener(&listenerB);

    world.step(1.0f / 60.0f);

    // Both listeners must have received the event
    REQUIRE(listenerA.callCount >= 1);
    REQUIRE(listenerB.callCount >= 1);
    REQUIRE(listenerA.callCount == listenerB.callCount);
}

TEST_CASE("Manifold passed to listener has valid collision data", "[collision_listener]") {
    World world;
    Circle circleShape(1.0f);
    Material material = {1.0f, 0.5f};

    auto bodyA = std::make_shared<RigidBody>(&circleShape, material, Vector2(0.0f, 0.0f));
    auto bodyB = std::make_shared<RigidBody>(&circleShape, material, Vector2(1.5f, 0.0f));
    world.addBody(bodyA);
    world.addBody(bodyB);

    TestListener listener;
    world.addCollisionListener(&listener);

    world.step(1.0f / 60.0f);

    REQUIRE(listener.callCount >= 1);
    const CollisionManifold& m = listener.received[0];

    REQUIRE(m.hasCollision == true);
    REQUIRE(m.penetration > 0.0f);
    // Normal must be a unit vector (or close to it)
    float normalMag = m.normal.magnitude();
    REQUIRE(normalMag == Catch::Approx(1.0f).margin(0.01f));
    // Both body pointers must be non-null
    REQUIRE(m.A != nullptr);
    REQUIRE(m.B != nullptr);
}
