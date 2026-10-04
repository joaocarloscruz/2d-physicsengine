#include "catch_amalgamated.hpp"

#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include "physics/core/collisions/broad_phase/uniform_grid.h"
#include "physics/core/material.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/core/world.h"
#include "physics/core/collisions/narrow_phase/collision_circle_circle.h"
#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"

#include <algorithm>
#include <cmath>
#include <limits>
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

TEST_CASE("Broad-phase bounds retain shapes below the position's floating-point spacing",
          "[BroadPhase][aabb-rounding]") {
    const bool polygon = GENERATE(false, true);
    for (float x : {-1e30f, -1e20f, 1e20f, 1e30f}) {
        auto make = [&](float y) {
            if (polygon)
                return std::make_shared<RigidBody>(Polygon::MakeBox(2, 2), Material{}, Vector2{x, y});
            return std::make_shared<RigidBody>(Circle(1), Material{}, Vector2{x, y});
        };
        auto a = make(-.75f), b = make(.75f), separated = make(10);
        const auto hit = polygon ? CollisionPolygonPolygon(a.get(), b.get())
                                 : CollisionCircleCircle(a.get(), b.get());
        REQUIRE(hit.hasCollision);
        REQUIRE(a->GetAABB().min.x < x);
        REQUIRE(a->GetAABB().max.x > x);
        const std::set<IdPair> expected{{a->GetId(), b->GetId()}};
        std::vector<RigidBodyPtr> bodies{a, b, separated};
        SweepAndPrune sweep;
        // Keep cell coordinates in range while testing exactly the same bounds.
        UniformGrid grid(std::abs(x));
        do {
            const auto candidates = sweep.FindPotentialCollisions(bodies);
            REQUIRE(pairSet(candidates) == expected);
            REQUIRE(candidates.size() == 1);
            REQUIRE(pairSet(grid.FindPotentialCollisions(bodies)) == expected);
        } while (std::next_permutation(bodies.begin(), bodies.end(),
                                     [](const auto& lhs, const auto& rhs) {
                                         return lhs->GetId() < rhs->GetId();
                                     }));
    }
}

TEST_CASE("World resolves an impact whose x extent was lost in float addition",
          "[BroadPhase][aabb-rounding]") {
    SimulationConfig config;
    config.solverIterations = 1;
    config.enableLinearVelocityLimit = false;
    config.enableAngularVelocityLimit = false;
    config.positionCorrectionFactor = 0;
    World world(config);
    const Material elastic{1, 1, 0, 0};
    auto a = std::make_shared<RigidBody>(Circle(1), elastic, Vector2{1e20f, -.75f});
    auto b = std::make_shared<RigidBody>(Circle(1), elastic, Vector2{1e20f, .75f});
    a->SetMass(1); b->SetMass(1);
    a->SetVelocity({0, 1}); b->SetVelocity({0, -1});
    world.addBody(a); world.addBody(b);
    world.step(0);
    REQUIRE(world.getLastStepStatistics().activeContactCount == 1);
    REQUIRE(a->velocity.y == Catch::Approx(-1).epsilon(0).margin(1e-7));
    REQUIRE(b->velocity.y == Catch::Approx(1).epsilon(0).margin(1e-7));
    REQUIRE(a->angularVelocity == 0);
    REQUIRE(b->angularVelocity == 0);
}

TEST_CASE("Rotated polygon bounds contain independently transformed vertices",
          "[BroadPhase][aabb-rounding]") {
    for (float scale : {1e-12f, 1.f, 1e12f}) {
        const auto polygon = Polygon::MakeBox(2 * scale, .7f * scale);
        for (float translation : {-1e30f, -1e10f, 0.f, 1e10f, 1e30f}) {
            for (float angle : {0.f, .37f, 1.23f, -2.14f}) {
                RigidBody body(polygon, Material{}, {translation, -translation}, true);
                body.SetOrientation(angle);
                const auto bounds = body.GetAABB();
                REQUIRE(bounds.min.x < bounds.max.x);
                REQUIRE(bounds.min.y < bounds.max.y);
                // Use the represented double rotation basis, with independent
                // extended-precision vertex transforms where the host has it.
                const long double c = std::cos(double(angle)), s = std::sin(double(angle));
                for (const auto& vertex : polygon.getVertices()) {
                    const long double x = translation + c * vertex.x - s * vertex.y;
                    const long double y = -static_cast<long double>(translation)
                                          + s * vertex.x + c * vertex.y;
                    REQUIRE(static_cast<long double>(bounds.min.x) <= x);
                    REQUIRE(static_cast<long double>(bounds.max.x) >= x);
                    REQUIRE(static_cast<long double>(bounds.min.y) <= y);
                    REQUIRE(static_cast<long double>(bounds.max.y) >= y);
                }
            }
        }
    }
}

TEST_CASE("AABB construction rejects invalid consumed state and unrepresentable bounds",
          "[BroadPhase][aabb-rounding]") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    RigidBody circle(Circle(1), Material{});
    circle.position.x = nan;
    REQUIRE_THROWS_AS(circle.GetAABB(), std::invalid_argument);
    circle.position = {};
    circle.orientation = nan; // Orientation is irrelevant to circle geometry.
    REQUIRE(circle.GetAABB().min.x == -1);
    circle.position.x = std::numeric_limits<float>::max();
    REQUIRE_THROWS_AS(circle.GetAABB(), std::overflow_error);

    RigidBody polygon(Polygon::MakeBox(2, 2), Material{});
    polygon.orientation = nan;
    REQUIRE_THROWS_AS(polygon.GetAABB(), std::invalid_argument);

    class TaggedShape final : public Shape {
      public:
        explicit TaggedShape(ShapeType tag) : Shape(tag) {}
        float GetArea() const override { return 1; }
        float GetInertia(float) const override { return 1; }
        std::unique_ptr<Shape> Clone() const override { return std::make_unique<TaggedShape>(*this); }
    };
    for (auto tag : {ShapeType::CIRCLE, ShapeType::POLYGON, ShapeType::COUNT}) {
        RigidBody malformed(TaggedShape(tag), Material{});
        REQUIRE_THROWS_AS(malformed.GetAABB(), std::invalid_argument);
    }
}

TEST_CASE("Sweep bounds refresh after transforms and failed calls", "[BroadPhase][differential]") {
    SweepAndPrune sweep;
    auto box = std::make_shared<RigidBody>(Polygon::MakeBox(4, 1), Material{});
    auto fixed = std::make_shared<RigidBody>(Circle(.75f), Material{}, Vector2{1.75f, 0}, true);
    auto otherFixed = std::make_shared<RigidBody>(Circle(.75f), Material{}, Vector2{1.75f, 0}, true);
    const std::vector<RigidBodyPtr> bodies{box, fixed, otherFixed};
    REQUIRE(pairSet(sweep.FindPotentialCollisions(bodies)) == bruteForcePairs(bodies));
    REQUIRE(sweep.FindPotentialCollisions(bodies).size() == 2);
    box->SetPosition({20, 0});
    REQUIRE(sweep.FindPotentialCollisions(bodies).empty());
    box->SetPosition({});
    box->SetOrientation(1.5707963267948966f);
    REQUIRE(sweep.FindPotentialCollisions(bodies).empty());
    fixed->SetPosition({});
    REQUIRE(pairSet(sweep.FindPotentialCollisions(bodies)) == bruteForcePairs(bodies));
    REQUIRE(sweep.FindPotentialCollisions(bodies).size() == 1);
    REQUIRE_THROWS_AS(sweep.FindPotentialCollisions({nullptr}), std::invalid_argument);
    REQUIRE_THROWS_AS(sweep.FindPotentialCollisions({box, nullptr}), std::invalid_argument);
    box->position.x = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(sweep.FindPotentialCollisions(bodies), std::invalid_argument);
    box->SetPosition({});
    REQUIRE(pairSet(sweep.FindPotentialCollisions(bodies)) == bruteForcePairs(bodies));
}
