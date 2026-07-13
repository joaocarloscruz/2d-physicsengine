#include "catch_amalgamated.hpp"

#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include "physics/core/collisions/broad_phase/uniform_grid.h"
#include "physics/core/material.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"

#include <algorithm>
#include <memory>
#include <random>
#include <set>
#include <utility>
#include <vector>

using namespace PhysicsEngine;

namespace {
using IdPair = std::pair<std::uint64_t, std::uint64_t>;

IdPair canonicalPair(const CollisionPair& pair) {
    return std::minmax(pair.first->GetId(), pair.second->GetId());
}

std::set<IdPair> pairSet(const std::vector<CollisionPair>& pairs) {
    std::set<IdPair> result;
    for (const CollisionPair& pair : pairs) {
        result.insert(canonicalPair(pair));
    }
    return result;
}

std::set<IdPair> bruteForcePairs(const std::vector<RigidBodyPtr>& bodies) {
    std::set<IdPair> result;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        for (std::size_t j = i + 1; j < bodies.size(); ++j) {
            if (bodies[i]->IsStatic() && bodies[j]->IsStatic()) {
                continue;
            }
            if (bodies[i]->GetAABB().IsOverlapping(bodies[j]->GetAABB())) {
                result.emplace(
                    std::min(bodies[i]->GetId(), bodies[j]->GetId()),
                    std::max(bodies[i]->GetId(), bodies[j]->GetId())
                );
            }
        }
    }
    return result;
}
}

TEST_CASE("Broad phases match brute-force AABB overlap", "[BroadPhase][differential]") {
    std::mt19937 random(0xC0111DEu);
    std::uniform_real_distribution<float> position(-50.0f, 50.0f);
    std::uniform_real_distribution<float> radius(0.25f, 4.0f);
    Material material{1.0f, 0.0f};
    std::vector<std::unique_ptr<Circle>> shapes;
    std::vector<RigidBodyPtr> bodies;
    shapes.reserve(120);
    bodies.reserve(120);

    for (int i = 0; i < 120; ++i) {
        shapes.push_back(std::make_unique<Circle>(radius(random)));
        bodies.push_back(std::make_shared<RigidBody>(
            shapes.back().get(),
            material,
            Vector2(position(random), position(random)),
            i % 17 == 0
        ));
    }

    const std::set<IdPair> expected = bruteForcePairs(bodies);
    SweepAndPrune sweep;
    UniformGrid grid(7.5f);

    REQUIRE(pairSet(sweep.FindPotentialCollisions(bodies)) == expected);
    REQUIRE(pairSet(grid.FindPotentialCollisions(bodies)) == expected);
}

TEST_CASE("Broad phases return no duplicate pairs for huge AABBs", "[BroadPhase][differential]") {
    Material material{1.0f, 0.0f};
    Circle hugeShape(20.0f);
    Circle smallShape(1.0f);
    auto huge = std::make_shared<RigidBody>(&hugeShape, material, Vector2());
    std::vector<RigidBodyPtr> bodies{huge};

    for (int i = 0; i < 20; ++i) {
        bodies.push_back(std::make_shared<RigidBody>(
            &smallShape,
            material,
            Vector2(static_cast<float>(i - 10), 0.0f)
        ));
    }

    UniformGrid grid(0.5f);
    const auto pairs = grid.FindPotentialCollisions(bodies);

    REQUIRE(pairSet(pairs).size() == pairs.size());
    REQUIRE(pairSet(pairs) == bruteForcePairs(bodies));
}
