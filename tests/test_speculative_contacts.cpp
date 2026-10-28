#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include "physics/core/collisions/broad_phase/uniform_grid.h"
#include <cmath>
#include <memory>

using namespace PhysicsEngine;
using Catch::Approx;

namespace {
constexpr double G = double(9.81f);
RigidBodyPtr Body(const Shape& shape, Vector2 position, bool fixed = false, Material material = {1, 0}) {
    return std::make_shared<RigidBody>(shape, material, position, fixed);
}
SimulationConfig Order(bool speculative) {
    SimulationConfig c;
    c.enableSpeculativeContacts = speculative;
    return c;
}
struct Counter : ICollisionListener {
    int begins = 0, persists = 0, ends = 0;
    void onCollisionBegin(const CollisionEvent& e) override { ++begins; REQUIRE(e.penetration >= 0); }
    void onCollisionPersist(const CollisionEvent& e) override { ++persists; REQUIRE(e.penetration >= 0); }
    void onCollisionEnd(const CollisionEvent&) override { ++ends; }
};
double PeakPenetration(World& world, int steps, float dt = 1.0f / 60) {
    double peak = 0;
    for (int i = 0; i < steps; ++i) {
        world.step(dt);
        const auto& b = world.getBodies();
        for (std::size_t j = 0; j < b.size(); ++j)
            for (std::size_t k = j + 1; k < b.size(); ++k)
                if (!(b[j]->IsStatic() && b[k]->IsStatic()))
                    peak = std::max(peak, double(CheckCollision(b[j].get(), b[k].get()).penetration));
    }
    return peak;
}
}

TEST_CASE("Margin queries report separated contacts with negative penetration", "[speculative][narrow]") {
    SECTION("circle pair") {
        auto a = Body(Circle(0.5f), {0, 0}), b = Body(Circle(0.5f), {1.25f, 0});
        REQUIRE_FALSE(CheckCollision(a.get(), b.get()).hasCollision);
        REQUIRE_FALSE(CheckCollision(a.get(), b.get(), 0.25f).hasCollision); // gap == margin
        const auto m = CheckCollision(a.get(), b.get(), 0.5f);
        REQUIRE(m.hasCollision);
        REQUIRE(m.penetration == Approx(-0.25f));
        REQUIRE(m.contacts[0].penetration == Approx(-0.25f));
        REQUIRE(m.normal.x == Approx(1));
    }
    SECTION("circle and polygon") {
        auto box = Body(Polygon::MakeBox(2, 2), {0, 0}, true), ball = Body(Circle(0.5f), {0, 1.75f});
        const auto m = CheckCollision(ball.get(), box.get(), 0.5f);
        REQUIRE(m.hasCollision);
        REQUIRE(m.penetration == Approx(-0.25f));
        REQUIRE(m.normal.y == Approx(-1)); // From the ball (A) toward the box (B).
        REQUIRE(m.contacts[0].position.y == Approx(1.25f)); // On the ball's real surface.
    }
    SECTION("parallel polygon faces keep two points") {
        auto floor = Body(Polygon::MakeBox(4, 1), {0, -0.5f}, true), box = Body(Polygon::MakeBox(1, 1), {0, 0.6f});
        const auto m = CheckCollision(floor.get(), box.get(), 0.2f);
        REQUIRE(m.hasCollision);
        REQUIRE(m.contactCount == 2);
        REQUIRE(m.contacts[0].penetration == Approx(-0.1f));
        REQUIRE(m.contacts[1].penetration == Approx(-0.1f));
        REQUIRE(m.normal.y == Approx(1));
    }
    SECTION("corner to corner uses the true gap") {
        // Face separation along either axis is 0.1, but the corners are
        // 0.1 * sqrt(2) apart along the diagonal.
        auto a = Body(Polygon::MakeBox(1, 1), {0, 0}), b = Body(Polygon::MakeBox(1, 1), {1.1f, 1.1f});
        REQUIRE_FALSE(CheckCollision(a.get(), b.get(), 0.14f).hasCollision);
        const auto m = CheckCollision(a.get(), b.get(), 0.2f);
        REQUIRE(m.hasCollision);
        REQUIRE(m.contactCount == 1);
        REQUIRE(m.penetration == Approx(-0.1 * std::sqrt(2.0)).margin(1e-6));
        REQUIRE(m.normal.x == Approx(std::sqrt(0.5)).margin(1e-6));
        REQUIRE(m.normal.y == Approx(std::sqrt(0.5)).margin(1e-6));
        REQUIRE(m.contacts[0].position.x == Approx(0.55f));
    }
    SECTION("capsule contacts sit midway between the real surfaces") {
        auto ball = Body(Circle(0.5f), {0, 1}), capsule = Body(Capsule(2, 0.25f), {0, 0}, true);
        REQUIRE_FALSE(CheckCollision(ball.get(), capsule.get()).hasCollision);
        const auto m = CheckCollision(ball.get(), capsule.get(), 0.5f);
        REQUIRE(m.hasCollision);
        REQUIRE(m.penetration == Approx(-0.25f));
        REQUIRE(m.contacts[0].position.y == Approx(0.375f));
    }
}

TEST_CASE("Broad phases grow bounds by each body's margin", "[speculative][BroadPhase]") {
    auto a = Body(Circle(0.5f), {0, 0}), b = Body(Circle(0.5f), {1.5f, 0});
    const std::vector<RigidBodyPtr> bodies{a, b};
    SweepAndPrune sap;
    UniformGrid grid(1.0f);
    for (IBroadPhase* phase : {static_cast<IBroadPhase*>(&sap), static_cast<IBroadPhase*>(&grid)}) {
        REQUIRE(phase->FindPotentialCollisions(bodies).empty());
        REQUIRE(phase->FindSpeculativeCollisions(bodies, {0.2f, 0.2f}).empty());
        REQUIRE(phase->FindSpeculativeCollisions(bodies, {0.3f, 0.3f}).size() == 1);
        REQUIRE_THROWS_AS(phase->FindSpeculativeCollisions(bodies, {0.3f}), std::invalid_argument);
    }
}

TEST_CASE("A fast box is stopped at the surface instead of sinking into it", "[speculative][world]") {
    {
        World world;
        world.addBody(Body(Polygon::MakeBox(20, 1), {0, -0.5f}, true));
        world.addBody(Body(Polygon::MakeBox(1, 1), {0, 0.5f}));
        // Five steps leave the falling box 1 cm above the resting one; the
        // sixth covers 0.217 m.
        auto box = Body(Polygon::MakeBox(1, 1), {0, 1.51f + 5 * 13.0f / 60});
        box->SetVelocity({0, -13});
        world.addBody(box);
        world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -float(G)}));
        // The legacy order sinks by up to v * dt = 0.22 m before seeing the
        // contact. What remains here is the resting slop under the impact load.
        REQUIRE(PeakPenetration(world, 60) < 1.5 * world.getSimulationConfig().penetrationSlop);
        REQUIRE(box->velocity.magnitude() < 1e-3f);
    }
}

TEST_CASE("Resting and sliding contacts follow the constant-load path", "[speculative][world][friction]") {
    const auto incline = [](double theta, float mu, int steps, float dt) {
        World world(SimulationConfig{});
        const Vector2 normal{float(-std::sin(theta)), float(std::cos(theta))};
        const Vector2 down{float(-std::cos(theta)), float(-std::sin(theta))};
        const Material material{1, 0, mu, mu};
        auto floor = Body(Polygon::MakeBox(60, 1), normal * -0.5f, true, material);
        floor->SetOrientation(float(theta));
        auto box = Body(Polygon::MakeBox(1, 1), normal * 0.5f, false, material);
        box->SetOrientation(float(theta));
        world.addBody(floor);
        world.addBody(box);
        world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -float(G)}));
        for (int i = 0; i < steps; ++i) world.step(dt);
        return std::pair<double, double>{double((box->position - normal * 0.5f).dot(down)),
                                         double(box->velocity.dot(down))};
    };
    const double pi = 3.14159265358979323846;
    SECTION("static friction holds without creep") {
        for (const float dt : {1.0f / 60, 1.0f / 240}) {
            const auto [distance, speed] = incline(pi / 12, 0.8f, int(std::lround(2 / dt)), dt);
            // The legacy order creeps g sin(theta) dt / 2 per second (0.021 m at 60 Hz).
            REQUIRE(std::abs(distance) < 1e-4);
            REQUIRE(std::abs(speed) < 1e-3);
        }
    }
    SECTION("kinetic friction integrates s = a t^2 / 2") {
        const double theta = 28 * pi / 180;
        const double acceleration = G * (std::sin(theta) - double(0.4f) * std::cos(theta));
        const auto [distance, speed] = incline(theta, 0.4f, 120, 1.0f / 60);
        REQUIRE(speed == Approx(acceleration * 2).epsilon(1e-4));
        REQUIRE(distance == Approx(acceleration * 2).epsilon(1e-4)); // 0.5 * a * 2^2
    }
}

TEST_CASE("A body resting on the ground has no velocity and no overlap", "[speculative][world]") {
    World world;
    world.addBody(Body(Polygon::MakeBox(20, 1), {0, -0.5f}, true));
    auto box = Body(Polygon::MakeBox(1, 1), {0, 0.5f});
    world.addBody(box);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -float(G)}));
    for (int i = 0; i < 120; ++i) world.step();
    REQUIRE(box->velocity.magnitude() < 1e-6f);
    REQUIRE(box->position.y == Approx(0.5f).margin(1e-5));
    REQUIRE(world.getPersistentContactCount() == 1);
}

TEST_CASE("Free flight steps exactly as in the legacy order", "[speculative][world]") {
    World speculative(Order(true)), legacy(Order(false));
    std::vector<RigidBodyPtr> bodies[2];
    for (int w = 0; w < 2; ++w) {
        World& world = w == 0 ? speculative : legacy;
        auto anchor = Body(Circle(0.1f), {0, 5}, true), bob = Body(Circle(0.2f), {2, 5});
        auto projectile = Body(Circle(0.3f), {-20, 0});
        projectile->SetVelocity({3, 7});
        bob->SetVelocity({0, 1});
        world.addBody(anchor); world.addBody(bob); world.addBody(projectile);
        world.addJoint(std::make_shared<DistanceJoint>(anchor, bob, 2.0f));
        world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -float(G)}));
        bodies[w] = {bob, projectile};
    }
    for (int i = 0; i < 90; ++i) {
        speculative.step(1.0f / 60);
        legacy.step(1.0f / 60);
        // The jointed bob differs: the speculative order solves joint rows with
        // the contacts, before positions integrate.
        REQUIRE(bodies[0][1]->position == bodies[1][1]->position);
        REQUIRE(bodies[0][1]->velocity == bodies[1][1]->velocity);
        REQUIRE(bodies[0][1]->orientation == bodies[1][1]->orientation);
        REQUIRE((bodies[0][0]->position - Vector2{0, 5}).magnitude() == Approx(2.0).margin(1e-3));
    }
}

TEST_CASE("Speculative-only pairs raise no events, statistics or wake-ups", "[speculative][world][events]") {
    SECTION("a body passing close by") {
        World world;
        Counter counter;
        world.addCollisionListener(&counter);
        world.addBody(Body(Polygon::MakeBox(20, 1), {0, -0.5f}, true));
        // Gliding 5 mm above the floor, inside the speculative margin, with no load.
        auto glider = Body(Polygon::MakeBox(1, 1), {-5, 0.505f});
        glider->SetVelocity({2, 0});
        world.addBody(glider);
        for (int i = 0; i < 60; ++i) {
            world.step();
            REQUIRE(world.getLastStepStatistics().activeContactCount == 0);
            REQUIRE(world.getLastStepStatistics().resolvedContactCount == 0);
            REQUIRE(world.getLastStepStatistics().solvedConstraintCount == 0);
            REQUIRE(world.getPersistentContactCount() == 0);
        }
        REQUIRE(counter.begins == 0);
        REQUIRE(glider->position.y == 0.505f);
        REQUIRE(glider->velocity == Vector2{2, 0});
    }
    SECTION("a sleeping body next to a moving one stays asleep") {
        auto config = SimulationConfig{};
        config.enableSleeping = true;
        World world(config);
        world.addBody(Body(Polygon::MakeBox(20, 1), {0, -0.5f}, true));
        auto sleeper = Body(Polygon::MakeBox(1, 1), {0, 0.5f});
        world.addBody(sleeper);
        world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -float(G)}));
        for (int i = 0; i < 120; ++i) world.step();
        REQUIRE_FALSE(sleeper->IsAwake());
        // A body gliding past 1 cm from the sleeper's side, inside its margin.
        auto passer = Body(Polygon::MakeBox(1, 1), {1.01f, 3});
        passer->SetVelocity({0, -1});
        world.addBody(passer);
        Counter counter;
        world.addCollisionListener(&counter);
        world.step();
        REQUIRE_FALSE(sleeper->IsAwake());
        REQUIRE(counter.begins == 0);
    }
}

TEST_CASE("A landing body begins touching on its landing step and never flickers", "[speculative][world][events]") {
    World world;
    Counter counter;
    world.addCollisionListener(&counter);
    world.addBody(Body(Polygon::MakeBox(20, 1), {0, -0.5f}, true));
    auto ball = Body(Circle(0.5f), {0, 0.5f + 0.4f});
    world.addBody(ball);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -float(G)}));
    int landed = -1;
    for (int i = 0; i < 180; ++i) {
        world.step();
        if (landed < 0 && counter.begins == 1) {
            landed = i;
            // The ball reached the floor, within the touching tolerance, in
            // this very step.
            REQUIRE(ball->position.y == Approx(0.5f).margin(0.25 * world.getSimulationConfig().penetrationSlop));
        }
    }
    REQUIRE(landed > 0);
    // At rest the overlap stays well inside the slop the legacy order sinks to.
    REQUIRE(ball->position.y == Approx(0.5f).margin(0.25 * world.getSimulationConfig().penetrationSlop));
    REQUIRE(counter.begins == 1);
    REQUIRE(counter.ends == 0);
    REQUIRE(counter.persists == 179 - landed);
}

TEST_CASE("Restitution reflects a speculative impact at the surface", "[speculative][world]") {
    World world;
    world.addBody(Body(Polygon::MakeBox(20, 1), {0, -0.5f}, true, Material{1, 1}));
    auto ball = Body(Circle(0.25f), {0, 2}, false, Material{1, 1});
    ball->SetVelocity({0, -10});
    world.addBody(ball);
    float lowest = 2, impactSpeed = 0, reboundSpeed = 0;
    for (int i = 0; i < 30; ++i) {
        const float before = ball->velocity.y;
        world.step();
        lowest = std::min(lowest, ball->position.y);
        if (before < 0 && ball->velocity.y > 0) { impactSpeed = -before; reboundSpeed = ball->velocity.y; }
    }
    REQUIRE(impactSpeed == Approx(10));
    REQUIRE(reboundSpeed == Approx(10));
    REQUIRE(lowest >= 0.25f - 1e-4f);
}

TEST_CASE("Restitution bounces off the speed at contact, not the end-of-step prediction", "[speculative][restitution]") {
    for (float e : {0.5f, 1.0f}) {
        World world;
        world.addBody(Body(Polygon::MakeBox(20, 2), {0, -1}, true, {1, e}));
        auto ball = Body(Circle(0.25f), {0, 1.25f}, false, {1, e});
        world.addBody(ball);
        world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -float(G)}));
        double apex = 0; bool bounced = false;
        for (int i = 0; i < 120; ++i) {
            world.step(1.0f / 60);
            if (ball->velocity.y > 0.1f) bounced = true;
            if (bounced) apex = std::max(apex, double(ball->position.y) - 0.25);
        }
        // Dropped from 1 m the ball rises to e^2 m; the old target added up to
        // (1 + e) g dt of speed at every bounce.
        REQUIRE(apex == Approx(double(e) * e).epsilon(0.02));
    }
}
