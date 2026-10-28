// Minimal regressions for narrow-phase bugs found by the seeded property tests
// in test_fuzz_narrow_phase.cpp. Each case is the exact failing input (hex
// floats), checked against the independent oracle in fuzz_support.h.
#include "catch_amalgamated.hpp"
#include "fuzz_support.h"

#include "physics/core/collisions/collision_dispatcher.h"
#include "physics/core/material.h"

using namespace PhysicsEngine;
using namespace FuzzSupport;

namespace {
struct Pose {
    ShapeSpec spec;
    Vector2 position;
    float orientation;
};
struct Pair {
    std::unique_ptr<Shape> shapeA, shapeB;
    std::unique_ptr<RigidBody> a, b;
    Geo ga, gb;
    Pair(const Pose& pa, const Pose& pb) {
        shapeA = pa.spec.Make();
        shapeB = pb.spec.Make();
        a = std::make_unique<RigidBody>(*shapeA, Material{}, pa.position);
        b = std::make_unique<RigidBody>(*shapeB, Material{}, pb.position);
        a->orientation = pa.orientation;
        b->orientation = pb.orientation;
        ga = GeoOf(pa.spec, pa.position, pa.orientation);
        gb = GeoOf(pb.spec, pb.position, pb.orientation);
    }
};
ShapeSpec CapsuleSpec(float halfLength, float radius) {
    ShapeSpec spec;
    spec.type = ShapeType::CAPSULE;
    spec.halfLength = halfLength;
    spec.radius = radius;
    return spec;
}
ShapeSpec CircleSpec(float radius) {
    ShapeSpec spec;
    spec.type = ShapeType::CIRCLE;
    spec.radius = radius;
    return spec;
}
ShapeSpec PolygonSpec(std::vector<Vector2> vertices) {
    ShapeSpec spec;
    spec.type = ShapeType::POLYGON;
    spec.vertices = std::move(vertices);
    return spec;
}
// Capsule contacts are midpoints of two surface points the contact depth
// apart along the normal (docs/capsules.md).
void RequireMidpointContacts(const CollisionManifold& m, const Pair& pair, double tolerance) {
    for (int i = 0; i < m.contactCount; ++i) {
        const D2 p = ToD2(m.contacts[i].position);
        const D2 half = ToD2(m.normal) * (0.5 * double(m.contacts[i].penetration));
        CHECK(std::abs(SignedDistance(pair.ga, p + half)) <= tolerance);
        CHECK(SignedDistance(pair.gb, p - half) <= tolerance);
    }
}
} // namespace

TEST_CASE("Capsule manifold depth is the minimum translation when clipping drops the deepest point",
          "[capsule][narrow-phase][regression]") {
    // Crossing capsule spines: the deepest incident spine end lies beyond the
    // reference spine's extent and is clipped away. The manifold used to
    // report the deepest remaining point (0.967377) instead of the exact
    // minimum translation (0.969804) that docs/capsules.md promises.
    const Pair capsules({CapsuleSpec(0x1.8ed1a6p-2f, 0x1.22865ep+0f), {-0x1.673f2p+9f, -0x1.a258p+4f}, 0x1.8ad908p-1f},
                        {CapsuleSpec(0x1.eafe62p-2f, 0x1.578d5ep+0f), {-0x1.68016p+9f, -0x1.8e58p+4f}, 0x1.f8cffep+0f});
    const auto m = CheckCollision(capsules.a.get(), capsules.b.get(), 0);
    const double expected = Penetration(capsules.ga, capsules.gb).penetration;
    REQUIRE(m.hasCollision);
    CHECK(std::abs(double(m.penetration) - expected) <= 1e-5);
    for (int i = 0; i < m.contactCount; ++i) CHECK(m.contacts[i].penetration <= m.penetration);
    // Translating B along the normal by the reported depth just separates it.
    const D2 n = ToD2(m.normal);
    CHECK(OverlapAlong(capsules.ga, capsules.gb, n) <= double(m.penetration) + 1e-5);

    // The same for a polygon against a capsule, with and without a margin.
    const Pair polygon({PolygonSpec({{-0x1.29f9bap-4f, 0x1.8b5c70p-4f}, {-0x1.4dfe4cp-4f, 0x1.1b4d2ap-5f},
                                     {0x1.bcb2ecp-6f, 0x1.345f9ap-5f}, {-0x1.24d9b2p-4f, 0x1.8b6c24p-4f}}),
                        {0x1.d6674p+8f, -0x1.2b16p+6f}, -0x1.68b1d0p+1f},
                       {CapsuleSpec(0x1.f56960p-2f, 0x1.a4bdc8p+0f), {0x1.d71p+8f, -0x1.291p+6f}, 0x1.6d54fap+0f});
    const double polygonExpected = Penetration(polygon.ga, polygon.gb).penetration;
    for (float margin : {0.0f, 0x1.1ea19ap-9f}) {
        const auto p = CheckCollision(polygon.a.get(), polygon.b.get(), margin);
        REQUIRE(p.hasCollision);
        CHECK(std::abs(double(p.penetration) - polygonExpected) <= 1e-5); // was 1.145748 vs 1.146003
    }
}

TEST_CASE("Capsule face ties against disjoint cores place the contact between the closest points",
          "[capsule][narrow-phase][regression]") {
    // A circle just past the end of a capsule spine: its side face ties the
    // closest-point direction within the allowance and is chosen, but the
    // clip to the spine extent drops the circle centre. The fallback used
    // the sign of axis.dot(normal), which is rounding noise for a face
    // normal, and placed the contact at the far spine end, 0.1 away.
    const Pair circle({CircleSpec(0x1.676f24p-4f), {-0x1.95203p+12f, -0x1.5b4168p+11f}, 0},
                      {CapsuleSpec(0x1.a01468p-3f, 0x1.1c9f28p-2f), {-0x1.95210cp+12f, -0x1.5b376p+11f}, -0x1.0c1898p-1f});
    const auto m = CheckCollision(circle.a.get(), circle.b.get(), 0);
    REQUIRE(m.hasCollision);
    REQUIRE(m.contactCount == 1);
    CHECK(std::abs(double(m.penetration) - Penetration(circle.ga, circle.gb).penetration) <= 1e-5);
    RequireMidpointContacts(m, circle, 1e-3); // float coordinates near 6000 round to 5e-4

    // The polygon version: a zero-length capsule near a polygon corner.
    const Pair polygon({PolygonSpec({{-0x1.122038p-4f, 0x1.caa5f6p-3f}, {-0x1.c36364p-1f, -0x1.0b0134p-2f},
                                     {-0x1.de0fdcp-1f, -0x1.697eaep-3f}, {-0x1.e3aa4ep-1f, -0x1.2e33b8p-3f},
                                     {-0x1.c4b49cp-1f, 0x1.7cb674p-3f}, {-0x1.147de4p-4f, 0x1.cc807ap-3f}}),
                        {-0x1.f4bdp+11f, 0x1.ba4568p+11f}, 0x1.6dca38p-1f},
                       {CapsuleSpec(0, 0x1.448a14p+0f), {-0x1.f4e5b8p+11f, 0x1.ba4778p+11f}, 0x1.7f3a7ap+1f});
    const auto p = CheckCollision(polygon.a.get(), polygon.b.get(), 0);
    REQUIRE(p.hasCollision);
    CHECK(std::abs(double(p.penetration) - Penetration(polygon.ga, polygon.gb).penetration) <= 1e-5);
    RequireMidpointContacts(p, polygon, 6e-4);

    // And two capsules, one of zero length, near the origin.
    const Pair capsules({CapsuleSpec(0, 0x1.728796p-2f), {-0x1.bp+1f, 0x1.95p+1f}, -0x1.ff8624p+0f},
                        {CapsuleSpec(0x1.685a32p-6f, 0x1.7401e4p-2f), {-0x1.6a8p+1f, 0x1.6bap+1f}, -0x1.08df0cp+1f});
    const auto q = CheckCollision(capsules.a.get(), capsules.b.get(), 0);
    REQUIRE(q.hasCollision);
    CHECK(std::abs(double(q.penetration) - Penetration(capsules.ga, capsules.gb).penetration) <= 1e-5);
    RequireMidpointContacts(q, capsules, 1e-5);
}
