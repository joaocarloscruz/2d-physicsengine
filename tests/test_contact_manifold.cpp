#include "catch_amalgamated.hpp"

#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"
#include "physics/core/collisions/collision_listener.h"
#include "physics/core/forces/gravity.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/core/world.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

using namespace PhysicsEngine;

namespace {
std::array<std::uint32_t, 2> SortedFeatureIds(
    const CollisionManifold& manifold
) {
    std::array<std::uint32_t, 2> result{};
    for (std::uint8_t i = 0; i < manifold.contactCount; ++i) {
        result[i] = manifold.contacts[i].featureId;
    }
    std::sort(result.begin(), result.begin() + manifold.contactCount);
    return result;
}

std::array<Vector2, 2> SortedContactPositions(
    const CollisionManifold& manifold
) {
    std::array<Vector2, 2> result{};
    for (std::uint8_t i = 0; i < manifold.contactCount; ++i) {
        result[i] = manifold.contacts[i].position;
    }
    std::sort(
        result.begin(),
        result.begin() + manifold.contactCount,
        [](const Vector2& left, const Vector2& right) {
            if (left.x != right.x) {
                return left.x < right.x;
            }
            return left.y < right.y;
        }
    );
    return result;
}

class RestingPatchListener : public ICollisionListener {
public:
    void onCollision(const CollisionManifold& manifold) override {
        if (manifold.contactCount != 2) {
            return;
        }
        const auto current = SortedFeatureIds(manifold);
        if (twoPointFrames == 0) {
            stableFeatures = current;
        } else if (current != stableFeatures) {
            ++featureChanges;
        }
        ++twoPointFrames;
    }

    std::array<std::uint32_t, 2> stableFeatures{};
    int twoPointFrames = 0;
    int featureChanges = 0;
};
}

TEST_CASE("Face-on polygon collision produces two clipped contacts", "[manifold][polygon]") {
    auto shape = Polygon::MakeBox(2.0f, 2.0f);
    Material material{1.0f, 0.0f};
    RigidBody first(&shape, material, Vector2(0.0f, 0.0f));
    RigidBody second(&shape, material, Vector2(1.5f, 0.0f));

    const CollisionManifold manifold = CollisionPolygonPolygon(&first, &second);

    REQUIRE(manifold.hasCollision);
    REQUIRE(manifold.contactCount == 2);
    REQUIRE(manifold.contacts[0].featureId != 0);
    REQUIRE(manifold.contacts[1].featureId != 0);
    REQUIRE(manifold.contacts[0].featureId != manifold.contacts[1].featureId);
    REQUIRE(manifold.contacts[0].penetration == Catch::Approx(0.5f));
    REQUIRE(manifold.contacts[1].penetration == Catch::Approx(0.5f));

    const auto positions = SortedContactPositions(manifold);
    REQUIRE(positions[0].x == Catch::Approx(0.5f));
    REQUIRE(positions[1].x == Catch::Approx(0.5f));
    REQUIRE(positions[0].y == Catch::Approx(-1.0f));
    REQUIRE(positions[1].y == Catch::Approx(1.0f));
}

TEST_CASE("Polygon manifold is equivalent when body order is reversed", "[manifold][polygon][property]") {
    auto firstShape = Polygon::MakeBox(3.0f, 1.0f);
    auto secondShape = Polygon::MakeBox(2.0f, 2.0f);
    Material material{1.0f, 0.0f};
    RigidBody first(&firstShape, material, Vector2(0.0f, 0.0f));
    RigidBody second(&secondShape, material, Vector2(1.0f, 0.2f));
    first.SetOrientation(0.35f);
    second.SetOrientation(-0.2f);

    const CollisionManifold forward = CollisionPolygonPolygon(&first, &second);
    const CollisionManifold reverse = CollisionPolygonPolygon(&second, &first);

    REQUIRE(forward.hasCollision);
    REQUIRE(reverse.hasCollision);
    REQUIRE(forward.contactCount == reverse.contactCount);
    REQUIRE(forward.normal.x == Catch::Approx(-reverse.normal.x).margin(1e-5f));
    REQUIRE(forward.normal.y == Catch::Approx(-reverse.normal.y).margin(1e-5f));
    REQUIRE(SortedFeatureIds(forward) == SortedFeatureIds(reverse));

    const auto forwardPositions = SortedContactPositions(forward);
    const auto reversePositions = SortedContactPositions(reverse);
    for (std::uint8_t i = 0; i < forward.contactCount; ++i) {
        REQUIRE(forwardPositions[i].x == Catch::Approx(reversePositions[i].x).margin(1e-5f));
        REQUIRE(forwardPositions[i].y == Catch::Approx(reversePositions[i].y).margin(1e-5f));
    }
}

TEST_CASE("Polygon contact features persist under small tangential motion", "[manifold][polygon][stability]") {
    auto floorShape = Polygon::MakeBox(10.0f, 1.0f);
    auto boxShape = Polygon::MakeBox(1.0f, 1.0f);
    Material material{1.0f, 0.0f};
    RigidBody floor(&floorShape, material, Vector2(0.0f, -0.5f), true);
    RigidBody box(&boxShape, material, Vector2(0.0f, 0.49f));

    const CollisionManifold initial = CollisionPolygonPolygon(&floor, &box);
    box.SetPosition(Vector2(0.02f, 0.49f));
    const CollisionManifold moved = CollisionPolygonPolygon(&floor, &box);

    REQUIRE(initial.contactCount == 2);
    REQUIRE(moved.contactCount == 2);
    REQUIRE(SortedFeatureIds(initial) == SortedFeatureIds(moved));
}

TEST_CASE("Corner polygon collision produces at most two contacts", "[manifold][polygon]") {
    auto shape = Polygon::MakeBox(2.0f, 2.0f);
    Material material{1.0f, 0.0f};
    RigidBody first(&shape, material, Vector2(0.0f, 0.0f));
    RigidBody second(&shape, material, Vector2(1.2f, 1.2f));
    second.SetOrientation(0.35f);

    const CollisionManifold manifold = CollisionPolygonPolygon(&first, &second);

    REQUIRE(manifold.hasCollision);
    REQUIRE(manifold.contactCount >= 1);
    REQUIRE(manifold.contactCount <= 2);
}

TEST_CASE("Resting box retains a stable two-point contact patch", "[manifold][stability]") {
    World world;
    auto floorShape = Polygon::MakeBox(20.0f, 1.0f);
    auto boxShape = Polygon::MakeBox(1.0f, 1.0f);
    Material material{1.0f, 0.0f, 0.7f, 0.5f};
    auto floor = std::make_shared<RigidBody>(
        &floorShape,
        material,
        Vector2(0.0f, -0.5f),
        true
    );
    auto box = std::make_shared<RigidBody>(
        &boxShape,
        material,
        Vector2(0.0f, 0.5f)
    );
    RestingPatchListener listener;
    world.addBody(floor);
    world.addBody(box);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2(0.0f, -9.81f)));
    world.addCollisionListener(&listener);

    for (int i = 0; i < 600; ++i) {
        world.step(1.0f / 60.0f);
    }

    REQUIRE(listener.twoPointFrames > 300);
    REQUIRE(listener.featureChanges == 0);
    REQUIRE(box->GetPosition().y == Catch::Approx(0.5f).margin(0.03f));
    REQUIRE(std::abs(box->GetOrientation()) < 0.02f);
    REQUIRE(std::abs(box->GetAngularVelocity()) < 0.05f);
    REQUIRE(std::abs(box->GetVelocity().y) < 0.05f);
}

TEST_CASE("Clipped contacts support clockwise convex polygons", "[manifold][polygon]") {
    Polygon clockwise({
        Vector2(-1.0f, -1.0f),
        Vector2(-1.0f, 1.0f),
        Vector2(1.0f, 1.0f),
        Vector2(1.0f, -1.0f)
    });
    Material material{1.0f, 0.0f};
    RigidBody first(&clockwise, material, Vector2(0.0f, 0.0f));
    RigidBody second(&clockwise, material, Vector2(1.5f, 0.0f));

    const CollisionManifold manifold = CollisionPolygonPolygon(&first, &second);

    REQUIRE(manifold.hasCollision);
    REQUIRE(manifold.contactCount == 2);
    REQUIRE(manifold.normal.x == Catch::Approx(1.0f));
}
