#include "catch_amalgamated.hpp"
#include "../include/physics/core/shape.h"
#include "../include/physics/core/rigidbody.h"
#include <cmath>

using namespace Catch;
using namespace PhysicsEngine;

// -----------------------------------------------------------------------
// Shape tests
// -----------------------------------------------------------------------

TEST_CASE("Circle Shape", "[shape]") {
    Circle circle(2.0f); // radius = 2

    SECTION("Area is pi * r^2") {
        REQUIRE(circle.GetArea() == Approx(static_cast<float>(M_PI) * 4.0f));
    }

    SECTION("Inertia is 0.5 * m * r^2") {
        // mass=3, r=2: I = 0.5 * 3 * 4 = 6
        REQUIRE(circle.GetInertia(3.0f) == Approx(6.0f));
    }

    SECTION("GetRadius returns the radius") {
        REQUIRE(circle.GetRadius() == Approx(2.0f));
    }

    SECTION("ShapeType is CIRCLE") {
        REQUIRE(circle.type == ShapeType::CIRCLE);
    }
}

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

// -----------------------------------------------------------------------
// RigidBody tests
// -----------------------------------------------------------------------

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

TEST_CASE("RigidBody AABB", "[RigidBody][AABB]") {
    Material material = {1.0f, 0.5f};

    SECTION("Circle AABB at origin has correct bounds") {
        Circle circleShape(1.5f); // radius = 1.5
        RigidBody body(&circleShape, material, Vector2(0.0f, 0.0f));
        AABB aabb = body.GetAABB();
        REQUIRE(aabb.min.x == Approx(-1.5f));
        REQUIRE(aabb.min.y == Approx(-1.5f));
        REQUIRE(aabb.max.x == Approx(1.5f));
        REQUIRE(aabb.max.y == Approx(1.5f));
    }

    SECTION("Circle AABB at offset position has correct world-space bounds") {
        Circle circleShape(1.0f);
        RigidBody body(&circleShape, material, Vector2(3.0f, -2.0f));
        AABB aabb = body.GetAABB();
        REQUIRE(aabb.min.x == Approx(2.0f));
        REQUIRE(aabb.min.y == Approx(-3.0f));
        REQUIRE(aabb.max.x == Approx(4.0f));
        REQUIRE(aabb.max.y == Approx(-1.0f));
    }

    SECTION("Box AABB at origin has correct bounds") {
        auto boxShape = Polygon::MakeBox(4.0f, 2.0f); // half-extents: 2 x 1
        RigidBody body(&boxShape, material, Vector2(0.0f, 0.0f));
        AABB aabb = body.GetAABB();
        REQUIRE(aabb.min.x == Approx(-2.0f));
        REQUIRE(aabb.min.y == Approx(-1.0f));
        REQUIRE(aabb.max.x == Approx(2.0f));
        REQUIRE(aabb.max.y == Approx(1.0f));
    }

    SECTION("Touching AABBs (sharing exactly one edge) do not overlap") {
        // Two 2x2 boxes side by side with no gap: A at x=0, B at x=2
        // A's max.x == B's min.x = 1.0, strict inequality means no overlap
        auto boxShapeA = Polygon::MakeBox(2.0f, 2.0f);
        auto boxShapeB = Polygon::MakeBox(2.0f, 2.0f);
        RigidBody a(&boxShapeA, material, Vector2(0.0f, 0.0f));
        RigidBody b(&boxShapeB, material, Vector2(2.0f, 0.0f));
        AABB aabbA = a.GetAABB();
        AABB aabbB = b.GetAABB();
        // They share the edge at x=1.0; strict < in IsOverlapping means no overlap
        REQUIRE(aabbA.IsOverlapping(aabbB) == false);
    }

    SECTION("Clearly separated AABBs do not overlap") {
        Circle circleShape(1.0f);
        RigidBody a(&circleShape, material, Vector2(0.0f, 0.0f));
        RigidBody b(&circleShape, material, Vector2(10.0f, 0.0f));
        REQUIRE(a.GetAABB().IsOverlapping(b.GetAABB()) == false);
    }

    SECTION("Overlapping AABBs report overlap") {
        Circle circleShape(1.0f);
        RigidBody a(&circleShape, material, Vector2(0.0f, 0.0f));
        RigidBody b(&circleShape, material, Vector2(1.5f, 0.0f));
        REQUIRE(a.GetAABB().IsOverlapping(b.GetAABB()) == true);
    }
}

TEST_CASE("RigidBody ApplyImpulse", "[RigidBody]") {
    Material material = {1.0f, 0.5f};

    SECTION("Impulse at center of mass changes linear velocity only") {
        // mass = 4, impulse = (8, 0) -> delta_v = J/m = (2, 0)
        Circle circleShape(1.0f);
        RigidBody body(&circleShape, material, Vector2(0.0f, 0.0f));
        body.SetMass(4.0f);

        Vector2 impulse(8.0f, 0.0f);
        Vector2 contactVector(0.0f, 0.0f); // at center of mass
        body.ApplyImpulse(impulse, contactVector);

        REQUIRE(body.GetVelocity().x == Approx(2.0f));
        REQUIRE(body.GetVelocity().y == Approx(0.0f));
        REQUIRE(body.GetAngularVelocity() == Approx(0.0f));
    }

    SECTION("Impulse with offset contact vector also changes angular velocity") {
        // mass = 1, r_perp contribution -> angular impulse = r x J
        auto boxShape = Polygon::MakeBox(2.0f, 2.0f);
        RigidBody body(&boxShape, material, Vector2(0.0f, 0.0f));
        body.SetMass(1.0f);

        // Apply upward impulse at the right edge (+1, 0) offset from center
        // Torque = r x J = (1,0) x (0,1) = 1*1 - 0*0 = 1
        Vector2 impulse(0.0f, 1.0f);
        Vector2 contactVector(1.0f, 0.0f); // right side of body
        body.ApplyImpulse(impulse, contactVector);

        // Linear velocity changes
        REQUIRE(body.GetVelocity().y == Approx(1.0f));
        // Angular velocity should be non-zero
        REQUIRE(body.GetAngularVelocity() != Approx(0.0f));
    }
}

TEST_CASE("RigidBody GetVelocityAtPoint", "[RigidBody]") {
    Material material = {1.0f, 0.5f};

    SECTION("Stationary body has zero velocity at any point") {
        Circle circleShape(1.0f);
        RigidBody body(&circleShape, material, Vector2(0.0f, 0.0f));
        Vector2 worldPoint(1.0f, 0.0f);
        Vector2 vel = body.GetVelocityAtPoint(worldPoint);
        REQUIRE(vel.x == Approx(0.0f));
        REQUIRE(vel.y == Approx(0.0f));
    }

    SECTION("Purely translating body: velocity at any point equals linear velocity") {
        Circle circleShape(1.0f);
        RigidBody body(&circleShape, material, Vector2(0.0f, 0.0f));
        body.SetVelocity(Vector2(3.0f, 1.0f));
        // No angular velocity, so every point has the same velocity
        Vector2 worldPoint(5.0f, -2.0f);
        Vector2 vel = body.GetVelocityAtPoint(worldPoint);
        REQUIRE(vel.x == Approx(3.0f));
        REQUIRE(vel.y == Approx(1.0f));
    }

    SECTION("Spinning body: velocity at offset point includes rotational contribution") {
        // Body at origin spinning at w=1 rad/s (counter-clockwise)
        // At world point (1, 0) relative to center: v_rot = w x r = (-w*r.y, w*r.x) = (0, 1)
        // No linear velocity -> total v = (0, 1)
        Circle circleShape(1.0f);
        RigidBody body(&circleShape, material, Vector2(0.0f, 0.0f));
        body.SetAngularVelocity(1.0f);

        Vector2 worldPoint(1.0f, 0.0f);
        Vector2 vel = body.GetVelocityAtPoint(worldPoint);
        REQUIRE(vel.x == Approx(0.0f).margin(1e-5f));
        REQUIRE(vel.y == Approx(1.0f).margin(1e-5f));
    }

    SECTION("Spinning + translating body: velocity is sum of both contributions") {
        // Linear velocity (2, 0), angular velocity 1 rad/s, point at (1,0) from center
        // v_lin = (2, 0), v_rot = (0, 1) -> total = (2, 1)
        Circle circleShape(1.0f);
        RigidBody body(&circleShape, material, Vector2(0.0f, 0.0f));
        body.SetVelocity(Vector2(2.0f, 0.0f));
        body.SetAngularVelocity(1.0f);

        Vector2 worldPoint(1.0f, 0.0f);
        Vector2 vel = body.GetVelocityAtPoint(worldPoint);
        REQUIRE(vel.x == Approx(2.0f).margin(1e-5f));
        REQUIRE(vel.y == Approx(1.0f).margin(1e-5f));
    }
}
