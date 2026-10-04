#include "catch_amalgamated.hpp"
#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/collisions/narrow_phase/collision_circle_circle.h"
#include "physics/core/world.h"
#include <cmath>
#include <limits>

using namespace PhysicsEngine;
namespace {
Material ContactMaterial(float restitution = 0, float friction = 0) {
    return {1, restitution, friction, friction};
}
CollisionManifold Contact(RigidBody &a, RigidBody &b, Vector2 point = {}, Vector2 normal = {1, 0}) {
    CollisionManifold m;
    m.A = &a;
    m.B = &b;
    m.hasCollision = true;
    m.normal = normal;
    m.contactPoint = point;
    return m;
}
void FiniteState(const RigidBody &b) {
    REQUIRE(std::isfinite(b.position.x));
    REQUIRE(std::isfinite(b.position.y));
    REQUIRE(std::isfinite(b.orientation));
    REQUIRE(std::isfinite(b.velocity.x));
    REQUIRE(std::isfinite(b.velocity.y));
    REQUIRE(std::isfinite(b.angularVelocity));
}
} // namespace
TEST_CASE("Contact stops a very heavy body without overflowing effective mass",
          "[contact-arithmetic]") {
    RigidBody fixed(Circle(0.5f), ContactMaterial(), {}, true),
        heavy(Circle(0.5f), ContactMaterial(), {0.9f, 0});
    heavy.SetMass(std::numeric_limits<float>::max());
    heavy.SetVelocity({-1e-10f, 0});
    const auto m = CollisionCircleCircle(&fixed, &heavy);
    REQUIRE(m.hasCollision);
    ContactImpulse impulse;
    CollisionResolver::Resolve(m, impulse);
    FiniteState(heavy);
    REQUIRE(heavy.velocity.x == Catch::Approx(0).epsilon(0).margin(1e-16));
    REQUIRE(heavy.velocity.y == 0);
    REQUIRE(heavy.angularVelocity == 0);
    REQUIRE(std::isfinite(impulse.normal));
    REQUIRE(impulse.normal == Catch::Approx(1e-10 / double(heavy.inverseMass)).epsilon(2e-7));
}
TEST_CASE("Contact retains finite impulses larger than float range", "[contact-arithmetic]") {
    RigidBody a(Circle(0.5f), ContactMaterial(1), {-0.45f, 0}),
        b(Circle(0.5f), ContactMaterial(1), {0.45f, 0});
    a.SetMass(1);
    b.SetMass(1);
    a.SetVelocity({2e38f, 0});
    b.SetVelocity({-2e38f, 0});
    ContactImpulse impulse;
    CollisionResolver::Resolve(CollisionCircleCircle(&a, &b), impulse);
    REQUIRE(a.velocity.x == Catch::Approx(-double(2e38f)).epsilon(1e-7));
    REQUIRE(b.velocity.x == Catch::Approx(double(2e38f)).epsilon(1e-7));
    REQUIRE(impulse.normal > std::numeric_limits<float>::max());
    REQUIRE(std::isfinite(impulse.normal));
    FiniteState(a);
    FiniteState(b);
}
TEST_CASE("Contact keeps tiny mass elastic exchange and large friction finite",
          "[contact-arithmetic]") {
    SECTION("tiny elastic masses") {
        RigidBody a(Circle(1), ContactMaterial(1), {-0.9f, 0}),
            b(Circle(1), ContactMaterial(1), {0.9f, 0});
        a.SetMass(1e-38f);
        b.SetMass(1e-38f);
        a.SetVelocity({1, 0});
        b.SetVelocity({-1, 0});
        CollisionResolver::Resolve(CollisionCircleCircle(&a, &b));
        REQUIRE(a.velocity.x == Catch::Approx(-1).epsilon(0).margin(2e-7));
        REQUIRE(b.velocity.x == Catch::Approx(1).epsilon(0).margin(2e-7));
        FiniteState(a);
        FiniteState(b);
    }
    SECTION("huge friction") {
        RigidBody a(Circle(1), ContactMaterial(0, std::numeric_limits<float>::max()), {}, true),
            b(Circle(1), ContactMaterial(0, std::numeric_limits<float>::max()), {1.8f, 0});
        b.SetMass(1);
        b.SetVelocity({-2, 3});
        CollisionResolver::Resolve(CollisionCircleCircle(&a, &b));
        REQUIRE(b.velocity.x == Catch::Approx(0).epsilon(0).margin(2e-7));
        REQUIRE(std::abs(b.velocity.y) < 3);
        FiniteState(b);
    }
}
TEST_CASE("Contact warm-start offcenter impulse matches Newton angular leverage",
          "[contact-arithmetic]") {
    RigidBody a(Circle(1), ContactMaterial()), b(Circle(1), ContactMaterial());
    a.SetMass(2);
    b.SetMass(4);
    CollisionResolver::WarmStart(Contact(a, b, {0, 2}), ContactImpulse{3, 4});
    REQUIRE(a.velocity.x == -1.5f);
    REQUIRE(a.velocity.y == -2);
    REQUIRE(b.velocity.x == 0.75f);
    REQUIRE(b.velocity.y == 1);
    REQUIRE(a.angularVelocity == 6);
    REQUIRE(b.angularVelocity == -3);
}
TEST_CASE("Rejected contact warm-start stages both endpoints", "[contact-arithmetic]") {
    RigidBody a(Circle(1), ContactMaterial()), b(Circle(1), ContactMaterial());
    a.SetMass(2);
    b.SetMass(4);
    a.SetVelocity({1, 2});
    a.SetAngularVelocity(3);
    Vector2 point;
    ContactImpulse impulse;
    SECTION("linear overflow in B") {
        b.SetVelocity({std::numeric_limits<float>::max(), 0});
        impulse.normal = 1e38;
    }
    SECTION("angular overflow") {
        point = {1e20f, 0};
        impulse.tangent = 1e20;
    }
    const auto av = a.velocity, bv = b.velocity;
    const float aw = a.angularVelocity, bw = b.angularVelocity;
    REQUIRE_THROWS_AS(CollisionResolver::WarmStart(Contact(a, b, point), impulse),
                      std::overflow_error);
    REQUIRE(a.velocity == av);
    REQUIRE(b.velocity == bv);
    REQUIRE(a.angularVelocity == aw);
    REQUIRE(b.angularVelocity == bw);
}
TEST_CASE("Contact double lever and point velocities avoid float intermediates",
          "[contact-arithmetic]") {
    RigidBody a(Circle(1), ContactMaterial(), {-2e38f, 0}),
        b(Circle(1), ContactMaterial(), {2e38f, 0});
    a.SetMass(2);
    b.SetMass(4);
    // Lever differences exceed float, but the complete angular updates fit.
    CollisionResolver::WarmStart(Contact(a, b, {2e38f, 0}), ContactImpulse{0, 1e-38});
    REQUIRE(std::isfinite(a.angularVelocity));
    REQUIRE(a.angularVelocity == Catch::Approx(-4).epsilon(2e-7));
    REQUIRE(b.angularVelocity == 0);
    FiniteState(a);
    FiniteState(b);
}
TEST_CASE("Contact midpoint and rotational point speed stay in double", "[contact-arithmetic]") {
    SECTION("large finite midpoint") {
        RigidBody a(Circle(0.5f), ContactMaterial(), {2e38f, 0}, true),
            b(Circle(0.5f), ContactMaterial(), {2e38f, 0});
        b.SetMass(1);
        b.SetVelocity({-1, 0});
        CollisionResolver::Resolve(CollisionCircleCircle(&a, &b));
        REQUIRE(b.velocity.x == 0);
        REQUIRE(b.angularVelocity == 0);
        FiniteState(a);
        FiniteState(b);
    }
    SECTION("rotational point velocity exceeds float") {
        RigidBody a(Circle(1), ContactMaterial(), {}, true), b(Circle(1), ContactMaterial());
        b.SetMass(1);
        b.SetAngularVelocity(2e38f);
        ContactImpulse impulse;
        CollisionResolver::Resolve(Contact(a, b, {0, 2e38f}), impulse);
        REQUIRE(impulse.normal == Catch::Approx(0.5).epsilon(0).margin(1e-7));
        REQUIRE(b.velocity.x == Catch::Approx(0.5).epsilon(0).margin(1e-7));
        FiniteState(b);
    }
}
TEST_CASE("Contact leaves static state and external wake requests unchanged",
          "[contact-arithmetic]") {
    RigidBody fixed(Circle(1), ContactMaterial(), {}, true), dynamic(Circle(1), ContactMaterial());
    fixed.SetVelocity({2, 1});
    fixed.SetAngularVelocity(3);
    dynamic.SetMass(1);
    const auto fp = fixed.position, fv = fixed.velocity;
    const float fw = fixed.angularVelocity, fo = fixed.orientation;
    CollisionResolver::Resolve(Contact(fixed, dynamic));
    REQUIRE(dynamic.velocity.x == Catch::Approx(2).epsilon(0).margin(1e-7));
    REQUIRE(fixed.position == fp);
    REQUIRE(fixed.velocity == fv);
    REQUIRE(fixed.angularVelocity == fw);
    REQUIRE(fixed.orientation == fo);
    SimulationConfig config;
    config.enableSleeping = true;
    config.sleepTimeThreshold = 0.01f;
    World world(config);
    auto a = std::make_shared<RigidBody>(Circle(1), ContactMaterial()),
         b = std::make_shared<RigidBody>(Circle(1), ContactMaterial(), Vector2{5, 0});
    world.addBody(a);
    world.addBody(b);
    world.step(0.02f);
    REQUIRE_FALSE(a->IsAwake());
    REQUIRE_FALSE(b->IsAwake());
    a->SetMass(2);
    b->SetMass(4);
    world.step(0.02f);
    REQUIRE_FALSE(a->IsAwake());
    REQUIRE_FALSE(b->IsAwake());
    CollisionResolver::WarmStart(Contact(*a, *b), {1, 0});
    REQUIRE_FALSE(a->IsAwake());
    REQUIRE_FALSE(b->IsAwake());
    const auto av = a->velocity, bv = b->velocity;
    REQUIRE_THROWS_AS(CollisionResolver::WarmStart(Contact(*a, *b, {1e20f, 0}), {0, 1e20}),
                      std::overflow_error);
    REQUIRE(a->velocity == av);
    REQUIRE(b->velocity == bv);
    REQUIRE_FALSE(a->IsAwake());
    REQUIRE_FALSE(b->IsAwake());
}
TEST_CASE("Contact invalid legacy state is rejected before pair publication",
          "[contact-arithmetic]") {
    RigidBody a(Circle(1), ContactMaterial()), b(Circle(1), ContactMaterial());
    const float nan = std::numeric_limits<float>::quiet_NaN();
    auto m = Contact(a, b);
    SECTION("velocity") {
        b.velocity.y = nan;
    }
    SECTION("position") {
        b.position.y = nan;
    }
    SECTION("orientation") {
        b.orientation = nan;
    }
    SECTION("inverse mass") {
        b.inverseMass = -1;
    }
    SECTION("inverse inertia") {
        b.inverseInertia = nan;
    }
    SECTION("zero inverse mass") {
        b.inverseMass = 0;
    }
    SECTION("negative friction") {
        b.material.dynamicFriction = -1;
    }
    SECTION("friction") {
        b.material.staticFriction = nan;
    }
    SECTION("restitution") {
        b.material.restitution = 2;
    }
    SECTION("normal") {
        m.normal.x = nan;
    }
    SECTION("contact point") {
        m.contactPoint.y = nan;
    }
    SECTION("penetration") {
        m.penetration = nan;
    }
    REQUIRE_THROWS_AS(CollisionResolver::Resolve(m), std::invalid_argument);
    REQUIRE(a.velocity == Vector2());
    REQUIRE(a.angularVelocity == 0);
    REQUIRE(a.position == Vector2());
}
TEST_CASE("Contact invalid cache and static inverse fields are rejected", "[contact-arithmetic]") {
    RigidBody a(Circle(1), ContactMaterial(), {}, true), b(Circle(1), ContactMaterial());
    auto m = Contact(a, b);
    ContactImpulse impulse;
    SECTION("cached normal") {
        impulse.normal = std::numeric_limits<double>::quiet_NaN();
    }
    SECTION("cached tangent") {
        impulse.tangent = std::numeric_limits<double>::infinity();
    }
    SECTION("static inverse mass") {
        a.inverseMass = 1;
    }
    SECTION("static inverse inertia") {
        a.inverseInertia = -1;
    }
    REQUIRE_THROWS_AS(CollisionResolver::WarmStart(m, impulse), std::invalid_argument);
    REQUIRE(a.velocity == Vector2());
    REQUIRE(b.velocity == Vector2());
    REQUIRE(a.angularVelocity == 0);
    REQUIRE(b.angularVelocity == 0);
}
