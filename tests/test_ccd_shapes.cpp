// CCD for every shape pair, with rotation: thin boxes, capsules and spinning
// polygons against thin walls, capsule bullets, polygon pairs, two moving CCD
// bodies, the time of impact against brute-force sub-stepping, and a pair
// pressed together past its allowed depth.
#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include "physics/math/matrix2x2.h"
#include "../src/physics/core/collisions/time_of_impact.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

using namespace PhysicsEngine;

namespace {
SimulationConfig Order(bool speculative) {
    SimulationConfig config;
    config.enableSpeculativeContacts = speculative;
    return config;
}

std::vector<Vector2> RegularOutline(int sides, float radius) {
    std::vector<Vector2> v;
    for (int i = 0; i < sides; ++i) {
        const double a = 2 * 3.141592653589793 * i / sides;
        v.push_back({float(radius * std::cos(a)), float(radius * std::sin(a))});
    }
    return v;
}

// Kinds: 0 thin box, 1 capsule, 2 pentagon, 3 triangle.
std::unique_ptr<Shape> Bullet(int kind) {
    switch (kind) {
    case 0: return std::make_unique<Polygon>(Polygon::MakeBox(0.6f, 0.06f));
    case 1: return std::make_unique<Capsule>(0.3f, 0.03f);
    case 2: return std::make_unique<Polygon>(RegularOutline(5, 0.25f));
    default: return std::make_unique<Polygon>(RegularOutline(3, 0.2f));
    }
}

// Largest x of the body's outline.
double Front(const RigidBody& b) {
    const auto rotation = Matrix2x2::rotation(b.orientation);
    if (b.shape->type == ShapeType::POLYGON) {
        double front = -1e9;
        for (const auto& v : static_cast<const Polygon&>(*b.shape).getVertices())
            front = std::max(front, double((b.position + rotation * v).x));
        return front;
    }
    if (b.shape->type == ShapeType::CAPSULE) {
        const auto& capsule = static_cast<const Capsule&>(*b.shape);
        const Vector2 axis = rotation * Vector2{capsule.GetHalfLength(), 0};
        return std::max(b.position.x + axis.x, b.position.x - axis.x) + capsule.GetRadius();
    }
    return b.position.x + b.shape->GetRadius();
}

// Smallest distance of any reported contact point from the line x = face.
struct FaceContacts : ICollisionListener {
    float face = 0;
    double nearest = 1e9;
    void note(const CollisionEvent& e) {
        for (int i = 0; i < e.contactCount; ++i)
            nearest = std::min(nearest, std::abs(double(e.contacts[i].position.x) - face));
    }
    void onCollisionBegin(const CollisionEvent& e) override { note(e); }
    void onCollisionPersist(const CollisionEvent& e) override { note(e); }
};
}

TEST_CASE("CCD stops thin boxes, capsules and spinning polygons at a thin static wall", "[ccd][ccd-shapes]") {
    // Before this change every polygon case could tunnel (about half of the
    // runs at these speeds did) and every capsule case made step() throw.
    // The wall is 5 cm thick; bodies cover up to 3.3 m per step.
    const float wallHalf = 0.025f;
    struct Case { int kind; float spin; };
    const Case cases[] = {{0, 0}, {0, 30}, {1, 0}, {1, 30}, {2, 30}, {3, 30}};
    for (const bool speculative : {true, false})
    for (const Case& c : cases)
    for (const float speed : {20.f, 80.f, 140.f, 200.f})
    for (int a = -6; a <= 6; a += 2)
    for (int o = 0; o < 4; ++o) {
        World world(Order(speculative));
        auto wall = std::make_shared<RigidBody>(Polygon::MakeBox(2 * wallHalf, 40), Material{}, Vector2{}, true);
        const float phi = a * 0.17453292f; // -60 to 60 degrees
        auto body = std::make_shared<RigidBody>(*Bullet(c.kind), Material{1, 0.5f, 0.3f, 0.2f},
                                                Vector2{-2, -2 * std::tan(phi)});
        body->SetOrientation(o * 0.7853982f + 0.1f * a);
        body->SetVelocity({speed * std::cos(phi), speed * std::sin(phi)});
        body->SetAngularVelocity(o % 2 ? -c.spin : c.spin);
        body->SetCcdEnabled(true);
        world.addBody(wall);
        world.addBody(body);
        FaceContacts contacts;
        contacts.face = -wallHalf;
        world.addCollisionListener(&contacts);
        bool through = false;
        double front = -1e9;
        for (int step = 0; step < 40; ++step) {
            world.step();
            through = through || body->position.x > 0;
            front = std::max(front, Front(*body));
        }
        INFO("speculative " << speculative << ", kind " << c.kind << ", spin " << c.spin << ", speed " << speed
                            << ", angle " << a << ", orientation " << o << ": front " << front
                            << ", nearest contact " << contacts.nearest);
        CHECK_FALSE(through);
        // Stopped at the face: penetration within the slop.
        CHECK(front <= -wallHalf + 0.005);
        // It reached the wall: a contact point on the face, or a step that
        // ended within the speculative allowance (a speculative contact that
        // bounces the body within one step raises no event).
        CHECK((contacts.nearest <= 0.01 || front >= -wallHalf - 0.021));
    }
}

TEST_CASE("Capsule bullets do not tunnel through static or dynamic plates", "[ccd][ccd-shapes][capsule]") {
    for (const bool speculative : {true, false})
    for (int target = 0; target < 4; ++target)
    for (const float spin : {0.f, 30.f}) {
        SimulationConfig config = Order(speculative);
        World world(config);
        // 0 static capsule, 1 static box, 2 dynamic heavy box, 3 dynamic heavy capsule;
        // each 4 cm thick across x = 0.
        const bool dynamic = target >= 2;
        RigidBodyPtr plate = target % 2 == 0
            ? std::make_shared<RigidBody>(Capsule(5, 0.02f), Material{}, Vector2{}, !dynamic)
            : std::make_shared<RigidBody>(Polygon::MakeBox(0.04f, 10), Material{}, Vector2{}, !dynamic);
        if (target % 2 == 0) plate->SetOrientation(1.5707964f);
        if (dynamic) plate->SetMass(50);
        auto bullet = std::make_shared<RigidBody>(Capsule(0.15f, 0.02f), Material{1, 0.2f, 0.3f, 0.2f}, Vector2{-3, 0.3f});
        bullet->SetVelocity({200, 0});
        bullet->SetAngularVelocity(spin);
        bullet->SetCcdEnabled(true);
        world.addBody(plate);
        world.addBody(bullet);
        std::uint32_t impacts = 0;
        for (int step = 0; step < 30; ++step) {
            world.step();
            impacts += world.getLastStepStatistics().ccdImpactCount;
            INFO("speculative " << speculative << ", target " << target << ", spin " << spin << ", step " << step);
            REQUIRE(bullet->position.x < plate->position.x);
        }
        CHECK(impacts > 0);
    }
    // A CCD circle against a dynamic capsule, which used to be rejected.
    for (const bool speculative : {true, false}) {
        World world(Order(speculative));
        auto plate = std::make_shared<RigidBody>(Capsule(5, 0.02f), Material{}, Vector2{});
        plate->SetOrientation(1.5707964f);
        plate->SetMass(50);
        auto ball = std::make_shared<RigidBody>(Circle(0.03f), Material{}, Vector2{-3, 0});
        ball->SetVelocity({200, 0});
        ball->SetCcdEnabled(true);
        world.addBody(plate);
        world.addBody(ball);
        for (int step = 0; step < 30; ++step) {
            world.step();
            REQUIRE(ball->position.x < plate->position.x);
        }
    }
}

TEST_CASE("CCD polygons do not pass through thin dynamic polygons", "[ccd][ccd-shapes]") {
    for (const bool speculative : {true, false})
    for (const float speed : {60.f, 200.f})
    for (const float spin : {0.f, -30.f, 30.f}) {
        World world(Order(speculative));
        auto plate = std::make_shared<RigidBody>(Polygon::MakeBox(0.04f, 3), Material{}, Vector2{});
        plate->SetMass(20);
        auto box = std::make_shared<RigidBody>(Polygon::MakeBox(0.3f, 0.05f), Material{1, 0.3f, 0.3f, 0.2f}, Vector2{-2, 0.2f});
        box->SetOrientation(0.3f);
        box->SetVelocity({speed, 0});
        box->SetAngularVelocity(spin);
        box->SetCcdEnabled(true);
        world.addBody(plate);
        world.addBody(box);
        for (int step = 0; step < 30; ++step) {
            world.step();
            INFO("speculative " << speculative << ", speed " << speed << ", spin " << spin << ", step " << step);
            REQUIRE(box->position.x < plate->position.x);
        }
        // The plate took momentum from the box.
        CHECK(plate->velocity.x > 0);
    }
}

TEST_CASE("Two fast CCD bodies meet head on and exchange momentum", "[ccd][ccd-shapes]") {
    for (const bool speculative : {true, false})
    for (int kind = 0; kind < 3; ++kind) {
        SimulationConfig config = Order(speculative);
        World world(config);
        auto a = std::make_shared<RigidBody>(*Bullet(kind), Material{1, 1, 0, 0}, Vector2{-3, 0});
        auto b = std::make_shared<RigidBody>(*Bullet(kind), Material{1, 1, 0, 0}, Vector2{3, 0.05f});
        a->SetOrientation(1.5707964f);
        b->SetOrientation(1.4f);
        a->SetVelocity({150, 0});
        b->SetVelocity({-150, 0});
        a->SetAngularVelocity(20);
        b->SetAngularVelocity(-25);
        a->SetCcdEnabled(true);
        b->SetCcdEnabled(true);
        world.addBody(a);
        world.addBody(b);
        const double momentum = a->GetMass() * a->velocity.x + b->GetMass() * b->velocity.x;
        for (int step = 0; step < 6; ++step) {
            world.step();
            INFO("speculative " << speculative << ", kind " << kind << ", step " << step);
            REQUIRE(a->position.x < b->position.x);
        }
        CHECK(a->velocity.x < 0);
        CHECK(b->velocity.x > 0);
        CHECK(a->GetMass() * a->velocity.x + b->GetMass() * b->velocity.x == Catch::Approx(momentum).margin(1e-3));
    }
}

TEST_CASE("CCD time of impact matches brute-force sub-stepping", "[ccd][ccd-shapes][numerical]") {
    // Random pairs of circles, capsules, boxes and pentagons that translate
    // and spin at up to the default limits (200 m/s, 30 rad/s) over a 60 Hz
    // step. The reference is the first of 20000 sub-steps at which the
    // narrow phase (separate code) reports overlap.
    std::mt19937 rng(20261031);
    std::uniform_real_distribution<double> unit(0, 1);
    const auto make = [&](int kind) -> std::unique_ptr<Shape> {
        switch (kind) {
        case 0: return std::make_unique<Circle>(float(0.05 + 0.3 * unit(rng)));
        case 1: return std::make_unique<Capsule>(float(0.05 + 0.4 * unit(rng)), float(0.02 + 0.1 * unit(rng)));
        case 2: return std::make_unique<Polygon>(Polygon::MakeBox(float(0.05 + 0.6 * unit(rng)), float(0.02 + 0.3 * unit(rng))));
        default: return std::make_unique<Polygon>(RegularOutline(5, float(0.05 + 0.3 * unit(rng))));
        }
    };
    const double dt = 1.0 / 60, tolerance = 1e-6;
    const int substeps = 20000;
    const double h = dt / substeps;
    int hits = 0, misses = 0;
    double worst = 0;
    for (int trial = 0; trial < 300; ++trial) {
        RigidBody a(*make(int(unit(rng) * 4)), Material{}), b(*make(int(unit(rng) * 4)), Material{});
        Detail::CcdProxy pa, pb;
        REQUIRE(Detail::MakeCcdProxy(*a.shape, pa));
        REQUIRE(Detail::MakeCcdProxy(*b.shape, pb));
        // A starts 0.5 to 3 m to the left of B, aimed near it.
        const double heading = (unit(rng) - 0.5) * 0.6, speed = 30 + 170 * unit(rng);
        Detail::CcdMotion ma{{-0.5 - 2.5 * unit(rng), (unit(rng) - 0.5), (unit(rng) - 0.5) * 6.28},
                             speed * std::cos(heading), speed * std::sin(heading), (unit(rng) - 0.5) * 60};
        Detail::CcdMotion mb{{0, 0, (unit(rng) - 0.5) * 6.28}, (unit(rng) - 0.5) * 40, (unit(rng) - 0.5) * 40,
                             (unit(rng) - 0.5) * 60};
        const auto place = [](RigidBody& body, const Detail::CcdPose& p) {
            body.position = {float(p.x), float(p.y)};
            body.orientation = float(p.angle);
        };
        // Skip pairs that start overlapping.
        place(a, ma.pose); place(b, mb.pose);
        if (CheckCollision(&a, &b).hasCollision) continue;
        int first = -1;
        for (int k = 1; k <= substeps && first < 0; ++k) {
            place(a, Detail::PoseAt(ma, k * h));
            place(b, Detail::PoseAt(mb, k * h));
            if (CheckCollision(&a, &b).hasCollision) first = k;
        }
        const auto toi = Detail::TimeOfImpact(pa, ma, pb, mb, dt, 0, tolerance, 256);
        INFO("trial " << trial << ", first overlapping sub-step " << first << ", toi " << toi.time << ", state "
                      << int(toi.state) << ", iterations " << toi.iterations);
        // Conservative: never later than the first overlap.
        if (first >= 0) {
            REQUIRE(toi.state != Detail::CcdToi::State::Separated);
            CHECK(toi.time <= first * h + 1e-8);
        }
        if (toi.state == Detail::CcdToi::State::Separated) { ++misses; continue; }
        // At the reported time the shapes are no more than the tolerance apart
        // according to the narrow phase, and not overlapping by more than
        // float rounding of the poses.
        place(a, Detail::PoseAt(ma, toi.time));
        place(b, Detail::PoseAt(mb, toi.time));
        const auto contact = CheckCollision(&a, &b, 1e-4f);
        CHECK(contact.hasCollision);
        CHECK(contact.penetration < 1e-4f);
        if (first >= 0) {
            ++hits;
            worst = std::max(worst, std::abs(toi.time - (first - 0.5) * h));
            CHECK(toi.time >= (first - 1) * h - 1e-7);
        }
    }
    INFO("hits " << hits << ", misses " << misses << ", worst time error " << worst << " s");
    CHECK(hits > 100);
    CHECK(worst < 2 * h);
}

TEST_CASE("A CCD circle pressed into a wall past its core is frozen, not pushed through", "[ccd][ccd-shapes]") {
    // A heavy ram drives a CCD circle that already overlaps a 10 cm wall by
    // more than three quarters of its radius. The circle-wall pair gets its
    // impulse, the ram pushes it back, and its core already touches the wall.
    // Such a pair was skipped for the rest of the step and the circle went
    // straight through in the legacy step order (9 to 40 m past the wall).
    for (const bool speculative : {true, false})
    for (const float overlap : {0.4f, 0.45f})
    for (const float speed : {30.f, 60.f, 120.f}) {
        World world(Order(speculative));
        auto wall = std::make_shared<RigidBody>(Polygon::MakeBox(0.1f, 10), Material{1, 0, 0, 0}, Vector2{0.05f, 0}, true);
        auto ball = std::make_shared<RigidBody>(Circle(0.5f), Material{1, 0, 0, 0}, Vector2{-0.5f + overlap, 0});
        ball->SetCcdEnabled(true);
        auto ram = std::make_shared<RigidBody>(Circle(0.5f), Material{1000, 0, 0, 0}, Vector2{-1.55f + overlap, 0});
        ram->SetVelocity({speed, 0});
        world.addBody(wall);
        world.addBody(ball);
        world.addBody(ram);
        for (int step = 0; step < 20; ++step) {
            world.step();
            INFO("speculative " << speculative << ", overlap " << overlap << ", speed " << speed << ", step " << step);
            REQUIRE(ball->position.x < 0);
        }
    }
}

TEST_CASE("Joint position correction never pulls a CCD box through a static wall", "[ccd][ccd-shapes][joint]") {
    // As for circles in test_ccd_driven.cpp: a heavy body that ignores the
    // wall drags a CCD box on a rigid distance joint, a rope or a revolute
    // joint. The correction after the solve is swept for every shape.
    for (const bool speculative : {true, false})
    for (int jointKind = 0; jointKind < 3; ++jointKind)
    for (const float speed : {5.f, 50.f, 200.f}) {
        World world(Order(speculative));
        auto wall = std::make_shared<RigidBody>(Polygon::MakeBox(0.1f, 2000), Material{}, Vector2{}, true);
        auto box = std::make_shared<RigidBody>(Polygon::MakeBox(0.1f, 0.1f), Material{}, Vector2{-0.5f, 0});
        box->SetMass(0.25f);
        box->SetCcdEnabled(true);
        auto heavy = std::make_shared<RigidBody>(Polygon::MakeBox(0.2f, 0.2f), Material{}, Vector2{-0.5f, 0.5f});
        heavy->SetMass(100);
        heavy->SetCollisionMaskBits(0);
        heavy->SetVelocity({speed, 0});
        world.addBody(wall);
        world.addBody(box);
        world.addBody(heavy);
        if (jointKind == 0) world.addJoint(std::make_shared<DistanceJoint>(heavy, box, 0.5f));
        if (jointKind == 1) {
            auto rope = std::make_shared<DistanceJoint>(heavy, box, 0.5f);
            rope->setLengthRange(0, 0.5f);
            world.addJoint(rope);
        }
        if (jointKind == 2) world.addJoint(std::make_shared<RevoluteJoint>(heavy, box, Vector2{0, -0.5f}, Vector2{}));
        double front = -1e9;
        bool through = false;
        for (int i = 0; i < 60; ++i) {
            world.step();
            REQUIRE(std::isfinite(box->position.x));
            front = std::max(front, Front(*box));
            through = through || box->position.x > 0;
        }
        INFO("speculative " << speculative << ", joint " << jointKind << ", speed " << speed << ": front " << front);
        CHECK_FALSE(through);
        // A box pressed into the wall by the joint spins against it; a pair
        // pressed back after its impulse may overlap by two slops (1 cm).
        CHECK(front <= -0.05 + 0.011);
    }
}

TEST_CASE("A CCD box held against a wall by a driven joint does not creep through it", "[ccd][ccd-shapes][joint]") {
    // The heavy body is driven at 5 m/s every step, so the joint keeps pulling
    // the box into the wall. An early version let the swept position
    // correction sink it by half the touching tolerance per step (the
    // closing test then looked at a wall corner 1000 m away), and the box
    // was through after about 180 steps.
    for (const bool speculative : {true, false})
    for (int jointKind = 0; jointKind < 2; ++jointKind) {
        World world(Order(speculative));
        auto wall = std::make_shared<RigidBody>(Polygon::MakeBox(0.1f, 2000), Material{}, Vector2{}, true);
        auto box = std::make_shared<RigidBody>(Polygon::MakeBox(0.1f, 0.1f), Material{}, Vector2{-0.5f, 0});
        box->SetMass(0.25f);
        box->SetCcdEnabled(true);
        auto heavy = std::make_shared<RigidBody>(Polygon::MakeBox(0.2f, 0.2f), Material{}, Vector2{-0.5f, 0.5f});
        heavy->SetMass(100);
        heavy->SetCollisionMaskBits(0);
        world.addBody(wall);
        world.addBody(box);
        world.addBody(heavy);
        if (jointKind == 0) world.addJoint(std::make_shared<DistanceJoint>(heavy, box, 0.5f));
        else world.addJoint(std::make_shared<RevoluteJoint>(heavy, box, Vector2{0, -0.5f}, Vector2{}));
        double front = -1e9;
        for (int i = 0; i < 600; ++i) {
            heavy->SetVelocity({5, 0});
            world.step();
            INFO("speculative " << speculative << ", joint " << jointKind << ", step " << i);
            REQUIRE(box->position.x < 0);
            front = std::max(front, Front(*box));
        }
        INFO("speculative " << speculative << ", joint " << jointKind << ": front " << front);
        CHECK(front <= -0.05 + 0.02);
    }
}
