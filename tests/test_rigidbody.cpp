#include "catch_amalgamated.hpp"
#include "../include/physics/core/shape.h"
#include "../include/physics/core/rigidbody.h"
#include <cmath>

using namespace Catch;
using namespace PhysicsEngine;

TEST_CASE("Polygon Shape", "[shape]") {
    // A 2x2 square centered at 0,0
    auto poly = Polygon::MakeBox(2.0f, 2.0f);

    // Area should be 2 * 2 = 4
    REQUIRE(poly.GetArea() == Approx(4.0f));
    
    // Inertia for a box is m(w^2 + h^2)/12
    // Mass = 1, w=2, h=2 -> 1(4+4)/12 = 8/12 = 0.666...
    REQUIRE(poly.GetInertia(1.0f) == Approx(8.0f / 12.0f));
}

TEST_CASE("Polygon Static Helpers", "[Polygon]") {
    SECTION("MakeBox creates a valid box polygon") {
        auto box = Polygon::MakeBox(2.0f, 3.0f);
        REQUIRE(box.getVertices().size() == 4);
        REQUIRE(box.GetArea() == Approx(6.0f));
        // Inertia for a box is m(w^2 + h^2)/12
        // Mass = 1, w=2, h=3 -> 1 * (4 + 9) / 12 = 13/12
        REQUIRE(box.GetInertia(1.0f) == Approx(13.0f / 12.0f));
    }

    SECTION("MakeTriangle creates a valid triangle polygon") {
        auto triangle = Polygon::MakeTriangle(Vector2(-2.0f, 0.0f), Vector2(2.0f, 0.0f), Vector2(0.0f, 5.0f));
        
        REQUIRE(triangle.getVertices().size() == 3);
        REQUIRE(triangle.GetArea() == Catch::Approx(10.0f)); 
    
        // The centroid of this triangle is at (0, 5/3).
        // Using Parallel Axis Theorem: I_origin = I_centroid + m * d^2
        // I_centroid = 37.0f / 18.0f
        // Shift = 1.0f * (5.0f/3.0f)^2 = 25.0f / 9.0f = 50.0f / 18.0f
        // Total = 87.0f / 18.0f = 4.8333...
        REQUIRE(triangle.GetInertia(1.0f) == Catch::Approx(87.0f / 18.0f));
    }
}


TEST_CASE("RigidBody operations are correct", "[RigidBody]") {
    auto box = Polygon::MakeBox(2.0f, 2.0f);
    Material material = {1.0f, 0.5f};
    RigidBody rb(&box, material, Vector2(0.0f, 0.0f));

    SECTION("Apply forces of rigid body is correct"){
        // apply force
        rb.ApplyForce(Vector2(10.0f, 2.0f));
        REQUIRE(rb.GetForce().x == Approx(10.0f));
        REQUIRE(rb.GetForce().y == Approx(2.0f));
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
        REQUIRE(rb.GetInertia() == Approx(10.0f * (2.0f*2.0f + 2.0f*2.0f) / 12.0f));
    }

    SECTION("RigidBody Inverse Mass and Inertia are correct") {
        rb.SetMass(10);
        REQUIRE(rb.GetInverseMass() == Approx(0.1f));
        REQUIRE(rb.GetInverseInertia() == Approx(1.0f / (10.0f * (2.0f*2.0f + 2.0f*2.0f) / 12.0f)));
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

TEST_CASE("Static RigidBody", "[RigidBody]") {
    auto box = Polygon::MakeBox(2.0f, 2.0f);
    Material material = {1.0f, 0.5f};
    RigidBody staticBody(&box, material, Vector2(0.0f, 0.0f), true);

    SECTION("Static body has zero mass and inverse mass") {
        REQUIRE(staticBody.GetMass() == 0.0f);
        REQUIRE(staticBody.GetInverseMass() == 0.0f);
        REQUIRE(staticBody.GetInertia() == 0.0f);
        REQUIRE(staticBody.GetInverseInertia() == 0.0f);
    }

    SECTION("Static body does not move") {
        staticBody.ApplyForce(Vector2(10.0f, 10.0f));
        staticBody.ApplyTorque(5.0f);
        staticBody.Integrate(0.1f);

        REQUIRE(staticBody.GetPosition().x == 0.0f);
        REQUIRE(staticBody.GetPosition().y == 0.0f);
        REQUIRE(staticBody.GetVelocity().x == 0.0f);
        REQUIRE(staticBody.GetVelocity().y == 0.0f);
        REQUIRE(staticBody.GetOrientation() == 0.0f);
        REQUIRE(staticBody.GetAngularVelocity() == 0.0f);
    }
}
