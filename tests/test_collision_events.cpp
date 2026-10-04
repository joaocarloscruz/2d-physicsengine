#include "catch_amalgamated.hpp"
#include "physics/core/world.h"
#include <functional>

using namespace PhysicsEngine;

namespace {
struct Events : ICollisionListener {
    std::vector<char> phases;
    std::vector<CollisionEvent> snapshots;
    std::function<void()> begin;
    void onCollisionBegin(const CollisionEvent& e) override {
        phases.push_back('B'); snapshots.push_back(e);
        if (begin) begin();
    }
    void onCollisionPersist(const CollisionEvent& e) override {
        phases.push_back('P'); snapshots.push_back(e);
    }
    void onCollisionEnd(const CollisionEvent& e) override {
        phases.push_back('E'); snapshots.push_back(e);
    }
};
}

TEST_CASE("Collision lifecycle survives separation filtering and removal", "[events]") {
    SimulationConfig config;
    config.positionCorrectionFactor = 0;
    World world(config);
    Circle circle(1);
    auto a = std::make_shared<RigidBody>(&circle, Material{1.0f, 0.0f}, Vector2(0, 0));
    auto b = std::make_shared<RigidBody>(&circle, Material{1.0f, 0.0f}, Vector2(1, 0));
    world.addBody(a); world.addBody(b);
    Events events;
    world.addCollisionListener(&events);
    world.addCollisionListener(&events);
    world.step(0); world.step(0);
    REQUIRE(events.phases == std::vector<char>{'B', 'P'});
    SECTION("separation") { b->SetPosition({5, 0}); world.step(0); }
    SECTION("filter") { b->SetCollisionMaskBits(0); world.step(0); }
    SECTION("remove") { world.removeBody(b); world.removeBody(b); }
    SECTION("clear") { world.clearBodies(); world.clearBodies(); }
    REQUIRE(events.phases == std::vector<char>{'B', 'P', 'E'});
    REQUIRE(events.snapshots.back().bodyAId == a->GetId());
    REQUIRE(events.snapshots.back().bodyBId == b->GetId());
    REQUIRE(events.snapshots.back().contactCount > 0);
    REQUIRE(events.snapshots.back().normal.magnitude() == Catch::Approx(1));
    REQUIRE(world.getPersistentContactCount() == 0);
    world.step(0);
    REQUIRE(events.phases.size() == 3);
}

TEST_CASE("Collision callbacks may remove bodies safely", "[events]") {
    World world;
    Circle circle(1);
    auto a = std::make_shared<RigidBody>(&circle, Material{1.0f, 0.0f});
    auto b = std::make_shared<RigidBody>(&circle, Material{1.0f, 0.0f}, Vector2(1, 0));
    world.addBody(a); world.addBody(b);
    Events events;
    events.begin = [&] {
        REQUIRE_THROWS_AS(world.step(0), std::logic_error);
        world.removeBody(b);
        b.reset();
    };
    world.addCollisionListener(&events);
    world.step(0);
    REQUIRE(events.phases == std::vector<char>{'B', 'E'});
    REQUIRE(world.getBodies().size() == 1);
    REQUIRE(world.getPotentialCollisions().empty());
    world.removeCollisionListener(&events);
    world.clearBodies();
    REQUIRE(events.snapshots.back().contacts[0].penetration >= 0);
}

TEST_CASE("Contact returning after a filter change begins again", "[events]") {
    SimulationConfig config;
    config.positionCorrectionFactor = 0;
    World world(config);
    Circle circle(1);
    auto a = std::make_shared<RigidBody>(&circle, Material{1.0f, 0.0f});
    auto b = std::make_shared<RigidBody>(&circle, Material{1.0f, 0.0f}, Vector2(1, 0));
    world.addBody(a); world.addBody(a); world.addBody(b);
    Events events;
    world.addCollisionListener(&events);
    world.step(0);
    b->SetCollisionMaskBits(0); world.step(0);
    b->SetCollisionMaskBits(0xffffffffu); world.step(0);
    REQUIRE(events.phases == std::vector<char>{'B', 'E', 'B'});
}
