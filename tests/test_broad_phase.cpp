#include "catch_amalgamated.hpp"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include "physics/core/collisions/broad_phase/uniform_grid.h"
#include "physics/core/world.h"
#include "physics/core/shape.h"
#include "physics/core/material.h"
#include <memory>
#include <algorithm>

using namespace PhysicsEngine;

namespace {
class RecordingBroadPhase : public IBroadPhase {
public:
    RecordingBroadPhase(int& callCount, CollisionPair pair)
        : callCount(callCount), pair(std::move(pair)) {}

    std::vector<CollisionPair> FindPotentialCollisions(
        const std::vector<RigidBodyPtr>&
    ) override {
        ++callCount;
        return {pair};
    }

private:
    int& callCount;
    CollisionPair pair;
};
}

TEST_CASE("Broad Phase - Uniform Grid vs Sweep And Prune", "[BroadPhase]") {
    // Setup some bodies
    Material defaultMat{1.0f, 0.5f};
    
    // Body A (0,0) bounds (-5,-5) to (5,5)
    auto shapeA = std::make_unique<Circle>(5.0f);
    auto bodyA = std::make_shared<RigidBody>(shapeA.get(), defaultMat, Vector2{0.0f, 0.0f});

    // Body B (8,0) bounds (3,-5) to (13,5) - Should overlap with A
    auto shapeB = std::make_unique<Circle>(5.0f);
    auto bodyB = std::make_shared<RigidBody>(shapeB.get(), defaultMat, Vector2{8.0f, 0.0f});

    // Body C (50, 50) - Far away, no overlap
    auto shapeC = std::make_unique<Circle>(5.0f);
    auto bodyC = std::make_shared<RigidBody>(shapeC.get(), defaultMat, Vector2{50.0f, 50.0f});

    std::vector<RigidBodyPtr> bodies = {bodyA, bodyB, bodyC};

    SECTION("SweepAndPrune identifies A and B") {
        SweepAndPrune sap;
        auto pairs = sap.FindPotentialCollisions(bodies);
        REQUIRE(pairs.size() == 1);
        bool match = (pairs[0].first == bodyA && pairs[0].second == bodyB) ||
                     (pairs[0].first == bodyB && pairs[0].second == bodyA);
        REQUIRE(match);
    }

    SECTION("UniformGrid identifies exactly A and B also") {
        UniformGrid grid(20.0f);
        auto pairs = grid.FindPotentialCollisions(bodies);
        REQUIRE(pairs.size() == 1);
        bool match = (pairs[0].first == bodyA && pairs[0].second == bodyB) ||
                     (pairs[0].first == bodyB && pairs[0].second == bodyA);
        REQUIRE(match);
    }
}

TEST_CASE("UniformGrid handles cell boundaries without duplicate pairs", "[BroadPhase][UniformGrid]") {
    Material material{1.0f, 0.5f};
    Circle largeShape(6.0f);
    Circle overlappingShape(6.0f);
    Circle negativeShape(1.0f);

    auto large = std::make_shared<RigidBody>(&largeShape, material, Vector2{0.0f, 0.0f});
    auto overlapping = std::make_shared<RigidBody>(&overlappingShape, material, Vector2{5.0f, 0.0f});
    auto negative = std::make_shared<RigidBody>(&negativeShape, material, Vector2{-20.0f, -20.0f});

    UniformGrid grid(2.0f);
    auto pairs = grid.FindPotentialCollisions({large, overlapping, negative});

    REQUIRE(pairs.size() == 1);
    REQUIRE(((pairs[0].first == large && pairs[0].second == overlapping)
        || (pairs[0].first == overlapping && pairs[0].second == large)));
}

TEST_CASE("UniformGrid validates configuration and ignores static pairs", "[BroadPhase][UniformGrid]") {
    UniformGrid grid(0.0f);
    REQUIRE(grid.getCellSize() == 100.0f);

    grid.setCellSize(-5.0f);
    REQUIRE(grid.getCellSize() == 100.0f);

    Circle shape(2.0f);
    Material material{1.0f, 0.5f};
    auto first = std::make_shared<RigidBody>(&shape, material, Vector2{0.0f, 0.0f}, true);
    auto second = std::make_shared<RigidBody>(&shape, material, Vector2{1.0f, 0.0f}, true);

    REQUIRE(grid.FindPotentialCollisions({first, second}).empty());
}

TEST_CASE("World delegates broad-phase selection through IBroadPhase", "[BroadPhase][World]") {
    World world;
    Circle shape(1.0f);
    Material material{1.0f, 0.0f};
    auto first = std::make_shared<RigidBody>(&shape, material, Vector2{0.0f, 0.0f});
    auto second = std::make_shared<RigidBody>(&shape, material, Vector2{1.5f, 0.0f});
    world.addBody(first);
    world.addBody(second);

    int callCount = 0;
    world.setBroadPhase(std::make_unique<RecordingBroadPhase>(
        callCount,
        CollisionPair{first, second}
    ));

    world.step(0.0f);

    REQUIRE(callCount > 0);
    REQUIRE(world.getPotentialCollisions().size() == 1);
}
