#include "catch_amalgamated.hpp"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/core/shape.h"
#include "../include/physics/core/world.h"
#include "../include/physics/core/collisions/collision_listener.h"

#include <memory>

using namespace PhysicsEngine;

namespace {
class CountingCollisionListener : public ICollisionListener {
public:
    void onCollision(const CollisionManifold&) override {
        ++count;
    }

    int count = 0;
};
}

TEST_CASE("Rigid bodies collide by default", "[collision_filter]") {
    Circle shape(1.0f);
    Material material = {1.0f, 0.0f};
    RigidBody first(&shape, material);
    RigidBody second(&shape, material);

    REQUIRE(first.GetCollisionCategoryBits() == 0x00000001u);
    REQUIRE(first.GetCollisionMaskBits() == 0xFFFFFFFFu);
    REQUIRE(first.CanCollideWith(second));
    REQUIRE(second.CanCollideWith(first));
}

TEST_CASE("Collision filtering requires mutual category and mask agreement", "[collision_filter]") {
    Circle shape(1.0f);
    Material material = {1.0f, 0.0f};
    RigidBody player(&shape, material);
    RigidBody enemy(&shape, material);

    player.SetCollisionCategoryBits(0x00000001u);
    player.SetCollisionMaskBits(0x00000002u);
    enemy.SetCollisionCategoryBits(0x00000002u);
    enemy.SetCollisionMaskBits(0x00000000u);

    REQUIRE_FALSE(player.CanCollideWith(enemy));
    REQUIRE_FALSE(enemy.CanCollideWith(player));

    enemy.SetCollisionMaskBits(0x00000001u);

    REQUIRE(player.CanCollideWith(enemy));
    REQUIRE(enemy.CanCollideWith(player));
}

TEST_CASE("World excludes filtered pairs and clears stale contacts", "[collision_filter][World]") {
    World world;
    Circle shape(1.0f);
    Material material = {1.0f, 0.0f};
    auto first = std::make_shared<RigidBody>(
        &shape,
        material,
        Vector2(0.0f, 0.0f)
    );
    auto second = std::make_shared<RigidBody>(
        &shape,
        material,
        Vector2(1.5f, 0.0f)
    );

    world.addBody(first);
    world.addBody(second);
    CountingCollisionListener listener;
    world.addCollisionListener(&listener);
    world.step(0.0f);

    REQUIRE(world.getPotentialCollisions().size() == 1);
    REQUIRE(world.getPersistentContactCount() == 1);
    REQUIRE(listener.count == 1);

    first->SetCollisionMaskBits(0x00000000u);
    world.step(0.0f);

    REQUIRE(world.getPotentialCollisions().empty());
    REQUIRE(world.getPersistentContactCount() == 0);
    REQUIRE(listener.count == 1);
}
