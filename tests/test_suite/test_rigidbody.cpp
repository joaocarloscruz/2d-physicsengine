#include "catch_amalgamated.hpp"
#include "../../include/physics/core/shape.h"
#include "../../include/physics/core/rigidbody.h"
#include <cmath>

using namespace Catch;

TEST_CASE("Shape operations are correct", "[Shape]") {
    Circle circle(5.0f);
    Rectangle rectangle(10.0f, 2.0f);
    Triangle triangle(5.0f, 4.0f);

    SECTION("Circle Area is correct"){
        REQUIRE(circle.GetArea() == Approx(M_PI * 25.0f));
    }

    SECTION("Rectangle Area is correct"){
        REQUIRE(rectangle.GetArea() == Approx(20.0f));
    }

    SECTION("Triangle Area is correct"){
        REQUIRE(triangle.GetArea() == Approx(10.0f));
    }

    // Inertia with mass as 1
    SECTION("Circle Inertia is correct"){
        REQUIRE(circle.GetInertia(1.0f) == Approx(12.5f));
    }

    SECTION("Rectangle Inertia is correct"){
        REQUIRE(rectangle.GetInertia(1.0f) == Approx(104.0f / 12.0f));
    }

    SECTION("Triangle Inertia is correct"){
        REQUIRE(triangle.GetInertia(1.0f) == Approx(25.0f/24.0f + 16.0f/18.0f)); 
    }

    SECTION("Shape dimensions are correct"){
        REQUIRE(rectangle.GetWidth() == Approx(10.0f));
        REQUIRE(rectangle.GetHeight() == Approx(2.0f));
        REQUIRE(triangle.GetWidth() == Approx(5.0f));
        REQUIRE(triangle.GetHeight() == Approx(4.0f));
        REQUIRE(circle.GetRadius() == Approx(5.0f));
    }
}

TEST_CASE("RigidBody operations are correct", "[RigidBody]") {
    Circle circle(5.0f);
    RigidBody rb(&circle, 1.0f, Vector2(0.0f, 0.0f));

    SECTION("Apply forces of rigid body is correct"){
        // apply force
        rb.ApplyForce(Vector2(10.0f, 2.0f));
        REQUIRE(rb.GetForce().x == Approx(10.0f));
        REQUIRE(rb.GetForce().y == Approx(2.0f));
        // apply gravity
        rb.ApplyForce(Vector2(0.0f, -9.81f));
        REQUIRE(rb.GetForce().y == Approx(-7.81f));
    }

    SECTION("Integrate") {
        rb.SetVelocity(Vector2(1.0f, 0.5f));
        rb.Integrate(0.016f); // integrate for 16ms. approximately 62.5 fps
        REQUIRE(rb.GetPosition().x == Approx(0.016f));
        REQUIRE(rb.GetPosition().y == Approx(0.008f));
    }

    SECTION("Apply torque of rigid body is correct"){
        rb.ApplyTorque(5.0f);
        REQUIRE(rb.GetTorque() == Approx(5.0f));
    }

    SECTION("RigidBody Mass is correct") {
        rb.SetMass(10);
        REQUIRE(rb.GetMass() == Approx(10.0f));
        REQUIRE(rb.GetInertia() == Approx(125.0f));
    }

    SECTION("RigidBody Inverse Mass and Inertia are correct") {
        rb.SetMass(10);
        REQUIRE(rb.GetInverseMass() == Approx(0.1f));
        REQUIRE(rb.GetInverseInertia() == Approx(1.0f / 125.0f));
    }

    SECTION("RigidBody Inertia is correct") {
        REQUIRE(rb.GetInertia() == Approx(rb.inertia));
    }

   SECTION("RigidBody Position is correct") {
        REQUIRE(rb.GetPosition().x == Approx(0.0f));
        REQUIRE(rb.GetPosition().y == Approx(0.0f));
    }

    SECTION("RigidBody Orientation is correct") {
        REQUIRE(rb.GetOrientation() == Approx(0.0f));
    }

    SECTION("RigidBody Velocity is correct") {
        REQUIRE(rb.GetVelocity().x == Approx(0.0f));
        REQUIRE(rb.GetVelocity().y == Approx(0.0f));
    }

    SECTION("RigidBody Angular Velocity is correct") {
        REQUIRE(rb.GetAngularVelocity() == Approx(0.0f));
    }
}