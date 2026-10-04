#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/collisions/narrow_phase/collision_circle_circle.h"
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace PhysicsEngine;

TEST_CASE("Elastic contacts retain effective mass across representable mass scales", "[contact_numerics]") {
    for (float mass : {5e-39f, 1e-20f, 1.0f, 1e20f}) {
        CAPTURE(mass);
        Circle shape(2);
        Material material{1, 1, 0, 0};
        RigidBody a(shape, material, {0, 0}), b(shape, material, {3, 0});
        a.SetMass(mass); b.SetMass(mass);
        a.SetVelocity({1, 0}); b.SetVelocity({-1, 0});
        CollisionResolver::Resolve(CollisionCircleCircle(&a, &b));
        REQUIRE(a.GetVelocity().x == Catch::Approx(-1).epsilon(0).margin(2e-6));
        REQUIRE(b.GetVelocity().x == Catch::Approx(1).epsilon(0).margin(2e-6));
        const double energy = 0.5 * (double(a.GetVelocity().x) * a.GetVelocity().x
                                  + double(b.GetVelocity().x) * b.GetVelocity().x);
        REQUIRE(energy == Catch::Approx(1).epsilon(2e-6));
        REQUIRE(a.GetAngularVelocity() == 0);
        REQUIRE(b.GetAngularVelocity() == 0);
    }
}

TEST_CASE("Friction mixing preserves finite Coulomb limits for large coefficients", "[contact_numerics]") {
    Circle shape(2);
    Material material{1, 0, 1e20f, 1e20f};
    RigidBody a(shape, material, {0, 0}, true), b(shape, material, {3, 0});
    b.SetMass(1);
    b.SetVelocity({-1e-23f, 1});
    const double impulse = double(material.dynamicFriction) * -b.GetVelocity().x;
    CollisionResolver::Resolve(CollisionCircleCircle(&a, &b));
    REQUIRE(b.GetVelocity().x == Catch::Approx(0).epsilon(0).margin(1e-28));
    REQUIRE(b.GetVelocity().y == Catch::Approx(1 - impulse).epsilon(0).margin(2e-7));
    // Contact at x=2 is one unit left of B's center; I = m*r^2/2 = 2.
    REQUIRE(b.GetAngularVelocity() == Catch::Approx(impulse / 2).epsilon(1e-6));
}

TEST_CASE("Circle contact geometry is invariant across representable coordinate scales", "[contact_numerics][circle_geometry]") {
    // Static shapes isolate narrow-phase geometry from dynamic area/inertia range.
    for (float radius : {1e-30f, 1e-10f, 1.0f, 1e10f, 1e19f, 1e30f}) {
        CAPTURE(radius);
        Circle shape(radius);
        RigidBody a(shape, Material{}, {}, true);
        RigidBody b(shape, Material{}, {0.9f * radius, 1.2f * radius}, true);
        const auto contact = CollisionCircleCircle(&a, &b);
        REQUIRE(contact.hasCollision);
        REQUIRE(contact.normal.x == Catch::Approx(0.6).epsilon(2e-6));
        REQUIRE(contact.normal.y == Catch::Approx(0.8).epsilon(2e-6));
        REQUIRE(double(contact.penetration) / radius == Catch::Approx(0.5).epsilon(2e-6));
        REQUIRE(double(contact.contactPoint.x) / radius == Catch::Approx(0.6).epsilon(2e-6));
        REQUIRE(double(contact.contactPoint.y) / radius == Catch::Approx(0.8).epsilon(2e-6));
        const auto reverse = CollisionCircleCircle(&b, &a);
        REQUIRE(reverse.hasCollision);
        REQUIRE(reverse.normal.x == -contact.normal.x);
        REQUIRE(reverse.normal.y == -contact.normal.y);
        b.SetPosition({2 * radius, 0});
        REQUIRE_FALSE(CollisionCircleCircle(&a, &b).hasCollision);
        b.SetPosition({3 * radius, 0});
        REQUIRE_FALSE(CollisionCircleCircle(&a, &b).hasCollision);
    }
}

TEST_CASE("Circle geometry rejects an unrepresentable penetration", "[contact_numerics][circle_geometry]") {
    Circle shape(std::numeric_limits<float>::max());
    RigidBody a(shape, Material{}, {}, true), b(shape, Material{}, {}, true);
    REQUIRE_THROWS_AS(CollisionCircleCircle(&a, &b), std::overflow_error);

    a.SetPosition({-std::numeric_limits<float>::max(), 0});
    b.SetPosition({std::numeric_limits<float>::max(), 0});
    REQUIRE_FALSE(CollisionCircleCircle(&a, &b).hasCollision);
    b.position.x = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(CollisionCircleCircle(&a, &b), std::invalid_argument);
}

TEST_CASE("Circle geometry rejects an unrepresentable surface point", "[contact_numerics][circle_geometry]") {
    const float maximum = std::numeric_limits<float>::max();
    Circle shape(maximum / 2);
    RigidBody a(shape, Material{}, {0.75f * maximum, 0}, true);
    RigidBody b(shape, Material{}, {maximum, 0}, true);
    REQUIRE_THROWS_AS(CollisionCircleCircle(&a, &b), std::overflow_error);
}
