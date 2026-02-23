#include "catch_amalgamated.hpp"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include "physics/core/collisions/broad_phase/uniform_grid.h"
#include "physics/core/shape.h"
#include "physics/core/material.h"
#include <memory>
#include <algorithm>

using namespace PhysicsEngine;

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
