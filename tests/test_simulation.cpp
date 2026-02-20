#include "catch_amalgamated.hpp"
#include "../include/physics/core/world.h"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/core/shape.h"
#include "../include/physics/math/vector2.h"
#include "../include/physics/core/forces/gravity.h"
#include <memory>
#include <cmath>

using namespace PhysicsEngine;

// Helper: total linear momentum of all bodies in the world
static Vector2 totalMomentum(const std::vector<std::shared_ptr<RigidBody>>& bodies) {
    Vector2 p(0.0f, 0.0f);
    for (const auto& b : bodies) {
        if (!b->IsStatic()) {
            p = p + b->GetVelocity() * b->GetMass();
        }
    }
    return p;
}

// Helper: total kinetic energy of all non-static bodies
static float totalKineticEnergy(const std::vector<std::shared_ptr<RigidBody>>& bodies) {
    float ke = 0.0f;
    for (const auto& b : bodies) {
        if (!b->IsStatic()) {
            float v2 = b->GetVelocity().magnitudeSquared();
            float w2 = b->GetAngularVelocity() * b->GetAngularVelocity();
            ke += 0.5f * b->GetMass() * v2 + 0.5f * b->GetInertia() * w2;
        }
    }
    return ke;
}

TEST_CASE("Linear momentum is conserved in isolated circle-circle collision", "[simulation]") {
    // No external forces, two equal-mass circles collide head-on.
    // Total momentum must be the same before and after the collision.
    World world;
    Circle circleShape(1.0f);
    Material material = {1.0f, 0.5f}; // restitution = 0.5

    auto bodyA = std::make_shared<RigidBody>(&circleShape, material, Vector2(-2.0f, 0.0f));
    auto bodyB = std::make_shared<RigidBody>(&circleShape, material, Vector2(2.0f, 0.0f));

    bodyA->SetMass(2.0f);
    bodyB->SetMass(2.0f);
    bodyA->SetVelocity(Vector2(3.0f, 0.0f));
    bodyB->SetVelocity(Vector2(-3.0f, 0.0f));

    world.addBody(bodyA);
    world.addBody(bodyB);

    Vector2 pBefore = totalMomentum(world.getBodies());

    // Step until collision is well resolved
    for (int i = 0; i < 10; ++i) {
        world.step(1.0f / 60.0f);
    }

    Vector2 pAfter = totalMomentum(world.getBodies());

    INFO("Momentum before: (" << pBefore.x << ", " << pBefore.y << ")");
    INFO("Momentum after:  (" << pAfter.x  << ", " << pAfter.y  << ")");

    REQUIRE(pAfter.x == Catch::Approx(pBefore.x).margin(0.01f));
    REQUIRE(pAfter.y == Catch::Approx(pBefore.y).margin(0.01f));
}

TEST_CASE("Elastic collision: moving circle hits stationary circle of equal mass", "[simulation]") {
    // With restitution = 1, the moving body stops and the stationary one moves forward
    World world;
    Circle circleShape(1.0f);
    Material material = {1.0f, 1.0f}; // perfectly elastic

    auto bodyA = std::make_shared<RigidBody>(&circleShape, material, Vector2(-2.0f, 0.0f));
    auto bodyB = std::make_shared<RigidBody>(&circleShape, material, Vector2(0.5f, 0.0f));

    bodyA->SetMass(1.0f);
    bodyB->SetMass(1.0f);
    bodyA->SetVelocity(Vector2(4.0f, 0.0f));
    // bodyB is stationary

    world.addBody(bodyA);
    world.addBody(bodyB);

    for (int i = 0; i < 10; ++i) {
        world.step(1.0f / 60.0f);
    }

    // After elastic equal-mass 1D collision:
    // A should have near-zero x-velocity, B should have moved in +x
    INFO("bodyA velocity: (" << bodyA->GetVelocity().x << ", " << bodyA->GetVelocity().y << ")");
    INFO("bodyB velocity: (" << bodyB->GetVelocity().x << ", " << bodyB->GetVelocity().y << ")");

    REQUIRE(bodyB->GetVelocity().x > 0.0f);          // B moves forward
    REQUIRE(bodyB->GetPosition().x > bodyA->GetPosition().x); // B is further right
}

TEST_CASE("Static body is unmoved by a dynamic body collision", "[simulation]") {
    World world;
    Circle circleShape(1.0f);
    auto groundShape = Polygon::MakeBox(10.0f, 1.0f);
    Material material = {1.0f, 0.5f};

    auto ground = std::make_shared<RigidBody>(&groundShape, material, Vector2(0.0f, -5.0f), true);
    auto ball   = std::make_shared<RigidBody>(&circleShape, material, Vector2(0.0f, -3.0f));

    ball->SetMass(1.0f);
    ball->SetVelocity(Vector2(0.0f, -5.0f)); // falling

    world.addBody(ground);
    world.addBody(ball);

    for (int i = 0; i < 30; ++i) {
        world.step(1.0f / 60.0f);
    }

    // Ground must never move
    REQUIRE(ground->GetPosition().x == Catch::Approx(0.0f));
    REQUIRE(ground->GetPosition().y == Catch::Approx(-5.0f));
    REQUIRE(ground->GetVelocity().x  == Catch::Approx(0.0f));
    REQUIRE(ground->GetVelocity().y  == Catch::Approx(0.0f));
}

TEST_CASE("Circle does not tunnel through a static floor under gravity", "[simulation]") {
    // The classic no-tunneling test: circle dropped from height onto a wide floor.
    World world;
    Circle circleShape(0.5f); // radius = 0.5
    auto groundShape = Polygon::MakeBox(20.0f, 1.0f);
    Material mat = {1.0f, 0.3f};

    float groundCenterY = -8.0f;
    float groundTopY    = groundCenterY + 0.5f; // = -7.5

    auto ground = std::make_shared<RigidBody>(&groundShape, mat, Vector2(0.0f, groundCenterY), true);
    auto circle = std::make_shared<RigidBody>(&circleShape, mat, Vector2(0.0f, -4.0f));

    world.addBody(ground);
    world.addBody(circle);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2(0.0f, -9.81f)));

    // Simulate 4 seconds (240 frames)
    for (int i = 0; i < 240; ++i) {
        world.step(1.0f / 60.0f);
    }

    float circleBottomY = circle->GetPosition().y - 0.5f;

    INFO("Ground top Y:     " << groundTopY);
    INFO("Circle bottom Y:  " << circleBottomY);
    INFO("Circle center Y:  " << circle->GetPosition().y);

    // Circle bottom must not be significantly below the floor (allow small tolerance)
    REQUIRE(circleBottomY >= groundTopY - 0.3f);
}

TEST_CASE("Two overlapping bodies separate after one step", "[simulation]") {
    // If two circles start overlapping, the collision response must push them apart.
    World world;
    Circle circleShape(1.0f);
    Material material = {1.0f, 0.5f};

    // Place circles so they overlap by 0.5
    auto bodyA = std::make_shared<RigidBody>(&circleShape, material, Vector2(0.0f, 0.0f));
    auto bodyB = std::make_shared<RigidBody>(&circleShape, material, Vector2(1.5f, 0.0f));

    bodyA->SetMass(1.0f);
    bodyB->SetMass(1.0f);

    world.addBody(bodyA);
    world.addBody(bodyB);

    float distBefore = (bodyB->GetPosition() - bodyA->GetPosition()).magnitude();

    world.step(1.0f / 60.0f);

    float distAfter = (bodyB->GetPosition() - bodyA->GetPosition()).magnitude();

    INFO("Distance before step: " << distBefore);
    INFO("Distance after step:  " << distAfter);

    // After resolution they should be farther apart (or at least not more overlapping)
    REQUIRE(distAfter >= distBefore - 0.01f);
    // And the distance should approach or exceed 2.0 (sum of radii = no overlap)
    REQUIRE(distAfter >= 1.9f);
}

TEST_CASE("Circle resting on floor under gravity stays at floor surface", "[simulation]") {
    // After 5 seconds, a circle sitting on a floor under gravity should be
    // resting at the surface, not sinking and not bouncing wildly.
    World world;
    Circle circleShape(0.5f);
    auto groundShape = Polygon::MakeBox(20.0f, 1.0f);
    Material mat = {1.0f, 0.1f}; // low restitution so it settles quickly

    float groundCenterY = -5.0f;
    float expectedRestY = groundCenterY + 0.5f + 0.5f; // ground top + radius = -4.0

    auto ground = std::make_shared<RigidBody>(&groundShape, mat, Vector2(0.0f, groundCenterY), true);
    auto circle = std::make_shared<RigidBody>(&circleShape, mat, Vector2(0.0f, -3.0f));

    world.addBody(ground);
    world.addBody(circle);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2(0.0f, -9.81f)));

    for (int i = 0; i < 300; ++i) { // 5 seconds
        world.step(1.0f / 60.0f);
    }

    float finalY = circle->GetPosition().y;
    INFO("Expected rest Y: " << expectedRestY);
    INFO("Final Y:         " << finalY);

    // The circle should be near the expected rest position (within 0.5 units)
    REQUIRE(finalY == Catch::Approx(expectedRestY).margin(0.5f));
    // And it should not have sunk through the floor
    REQUIRE(circle->GetPosition().y - 0.5f >= groundCenterY + 0.5f - 0.3f);
}

TEST_CASE("Box sliding on frictionless floor maintains velocity", "[simulation][friction]") {
    World world;
    auto boxShape = Polygon::MakeBox(1.0f, 1.0f);
    auto groundShape = Polygon::MakeBox(20.0f, 1.0f);
    
    // Frictionless material
    Material zeroFriction = {1.0f, 0.0f, 0.0f, 0.0f};

    auto ground = std::make_shared<RigidBody>(&groundShape, zeroFriction, Vector2(0.0f, -0.5f), true);
    auto box = std::make_shared<RigidBody>(&boxShape, zeroFriction, Vector2(0.0f, 0.51f));

    box->SetVelocity(Vector2(5.0f, 0.0f));
    
    world.addBody(ground);
    world.addBody(box);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2(0.0f, -9.81f)));

    for (int i = 0; i < 60; ++i) { // 1 second
        world.step(1.0f / 60.0f);
    }

    REQUIRE(box->GetVelocity().x == Catch::Approx(5.0f).margin(0.01f));
}

TEST_CASE("Box sliding on floor with friction slows down", "[simulation][friction]") {
    World world;
    auto boxShape = Polygon::MakeBox(1.0f, 1.0f);
    auto groundShape = Polygon::MakeBox(20.0f, 1.0f);
    
    // High friction material
    Material highFriction = {1.0f, 0.0f, 0.8f, 0.6f};

    auto ground = std::make_shared<RigidBody>(&groundShape, highFriction, Vector2(0.0f, -0.5f), true);
    auto box = std::make_shared<RigidBody>(&boxShape, highFriction, Vector2(0.0f, 0.51f));

    box->SetVelocity(Vector2(5.0f, 0.0f));
    
    world.addBody(ground);
    world.addBody(box);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2(0.0f, -9.81f)));

    for (int i = 0; i < 60; ++i) { // 1 second
        world.step(1.0f / 60.0f);
    }

    // Since friction is applied, velocity should be strictly less than 5.0
    // With dynamic friction 0.6 and gravity 9.81, deceleration is ~5.88 m/s^2.
    // Over 1s, it should come to a complete stop (x velocity approaches 0).
    REQUIRE(box->GetVelocity().x < 1.0f);
}
