#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include "physics/core/collisions/collision_dispatcher.h"
#include "physics/core/collisions/narrow_phase/collision_capsule.h"
#include "physics/core/collisions/narrow_phase/collision_circle_circle.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include "physics/core/collisions/broad_phase/uniform_grid.h"
#include "physics/core/fluids/fluid_boundary.h"
#include "physics/core/fluids/fluid_rigid_coupler.h"
#include "physics/core/particles/particle_system.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <vector>

using namespace PhysicsEngine;
using Catch::Approx;

namespace {
constexpr double Pi = 3.14159265358979323846;

// ----- Independent geometry oracles (double precision, no engine code) -----
struct P { double x, y; };
P operator+(P a, P b) { return {a.x + b.x, a.y + b.y}; }
P operator-(P a, P b) { return {a.x - b.x, a.y - b.y}; }
P operator*(P a, double s) { return {a.x * s, a.y * s}; }
double Dot(P a, P b) { return a.x * b.x + a.y * b.y; }
double Cross(P a, P b) { return a.x * b.y - a.y * b.x; }
double PointSegment(P p, P a, P b) {
    const P ab = b - a;
    const double length = Dot(ab, ab);
    const double t = length > 0 ? std::clamp(Dot(p - a, ab) / length, 0.0, 1.0) : 0.0;
    const P d = p - (a + ab * t);
    return std::hypot(d.x, d.y);
}
bool OnSegment(P p, P a, P b) {
    return std::min(a.x, b.x) <= p.x && p.x <= std::max(a.x, b.x)
        && std::min(a.y, b.y) <= p.y && p.y <= std::max(a.y, b.y);
}
bool SegmentsIntersect(P a, P b, P c, P d) {
    const double d1 = Cross(b - a, c - a), d2 = Cross(b - a, d - a);
    const double d3 = Cross(d - c, a - c), d4 = Cross(d - c, b - c);
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
        return true;
    return (d1 == 0 && OnSegment(c, a, b)) || (d2 == 0 && OnSegment(d, a, b))
        || (d3 == 0 && OnSegment(a, c, d)) || (d4 == 0 && OnSegment(b, c, d));
}
double SegmentDistance(P a, P b, P c, P d) {
    if (SegmentsIntersect(a, b, c, d)) return 0;
    return std::min({PointSegment(a, c, d), PointSegment(b, c, d),
                     PointSegment(c, a, b), PointSegment(d, a, b)});
}
bool InsideConvex(P p, const std::vector<P>& polygon) {
    double sign = 0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const double side = Cross(polygon[(i + 1) % polygon.size()] - polygon[i], p - polygon[i]);
        if (side == 0) continue;
        if (sign != 0 && (side > 0) != (sign > 0)) return false;
        sign = side;
    }
    return true;
}
double SegmentPolygonDistance(P a, P b, const std::vector<P>& polygon) {
    if (InsideConvex(a, polygon) || InsideConvex(b, polygon)) return 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < polygon.size(); ++i)
        best = std::min(best, SegmentDistance(a, b, polygon[i], polygon[(i + 1) % polygon.size()]));
    return best;
}

struct Spine { P a, b; double radius; };
Spine SpineOf(const RigidBody& body, P shift = {0, 0}) {
    const P c{body.position.x + shift.x, body.position.y + shift.y};
    if (body.shape->type == ShapeType::CIRCLE) return {c, c, body.shape->GetRadius()};
    const auto& capsule = dynamic_cast<const Capsule&>(*body.shape);
    const P axis{std::cos(double(body.orientation)) * capsule.GetHalfLength(),
                 std::sin(double(body.orientation)) * capsule.GetHalfLength()};
    return {c - axis, c + axis, capsule.GetRadius()};
}
std::vector<P> OutlineOf(const RigidBody& body, P shift = {0, 0}) {
    const auto& polygon = dynamic_cast<const Polygon&>(*body.shape);
    const double c = std::cos(double(body.orientation)), s = std::sin(double(body.orientation));
    std::vector<P> result;
    for (const auto& v : polygon.getVertices())
        result.push_back({body.position.x + shift.x + c * v.x - s * v.y,
                          body.position.y + shift.y + s * v.x + c * v.y});
    return result;
}
// Positive separation (negative overlap) of b, translated by shift, from a.
double Clearance(const RigidBody& a, const RigidBody& b, P shift) {
    const auto rounded = [](const RigidBody& body) { return body.shape->type != ShapeType::POLYGON; };
    if (rounded(a) && rounded(b)) {
        const Spine sa = SpineOf(a), sb = SpineOf(b, shift);
        return SegmentDistance(sa.a, sa.b, sb.a, sb.b) - sa.radius - sb.radius;
    }
    const bool capsuleFirst = rounded(a);
    const Spine s = capsuleFirst ? SpineOf(a) : SpineOf(b, shift);
    const auto outline = capsuleFirst ? OutlineOf(b, shift) : OutlineOf(a);
    return SegmentPolygonDistance(s.a, s.b, outline) - s.radius;
}
// Minimum translation of b that separates the shapes, over sampled
// directions, each found by bisection on the oracle clearance.
struct Translation { double depth; P direction; };
Translation MinimumTranslation(const RigidBody& a, const RigidBody& b, double reach) {
    Translation best{std::numeric_limits<double>::infinity(), {}};
    constexpr int directions = 1440;
    for (int k = 0; k < directions; ++k) {
        const double angle = 2 * Pi * k / directions;
        const P d{std::cos(angle), std::sin(angle)};
        double low = 0, high = reach;
        for (int i = 0; i < 52; ++i) {
            const double mid = 0.5 * (low + high);
            (Clearance(a, b, d * mid) < 0 ? low : high) = mid;
        }
        if (high < best.depth) best = {high, d};
    }
    return best;
}

RigidBodyPtr Make(const Shape& shape, Vector2 position, float orientation = 0, bool fixed = false,
                  Material material = Material{}) {
    auto body = std::make_shared<RigidBody>(shape, material, position, fixed);
    body->SetOrientation(orientation);
    return body;
}
std::uint64_t Lcg(std::uint64_t& state) {
    state = state * 6364136223846793005ull + 1442695040888963407ull;
    return state >> 11;
}
double Uniform(std::uint64_t& state, double low, double high) {
    return low + (high - low) * (double(Lcg(state)) / double(1ull << 53));
}

// Each manifold must be the exact minimum translation and independent of
// argument order, up to the normal's sign.
void CheckAgainstOracle(RigidBody& a, RigidBody& b, double reach) {
    const auto forward = CheckCollision(&a, &b);
    const auto reverse = CheckCollision(&b, &a);
    const double clearance = Clearance(a, b, {0, 0});
    const double scale = std::max(1.0, reach);
    if (std::abs(clearance) < 1e-6 * scale) return; // Borderline touching.
    REQUIRE(forward.hasCollision == (clearance < 0));
    REQUIRE(reverse.hasCollision == forward.hasCollision);
    if (!forward.hasCollision) return;
    const auto oracle = MinimumTranslation(a, b, reach);
    REQUIRE(forward.penetration == Approx(oracle.depth).epsilon(2e-5).margin(2e-6 * scale));
    REQUIRE(reverse.penetration == Approx(forward.penetration).epsilon(1e-6).margin(1e-7 * scale));
    REQUIRE(reverse.normal.x == Approx(-forward.normal.x).margin(1e-6));
    REQUIRE(reverse.normal.y == Approx(-forward.normal.y).margin(1e-6));
    REQUIRE(std::hypot(double(forward.normal.x), double(forward.normal.y)) == Approx(1).epsilon(1e-6));
    // Translating b along the normal by the reported depth just separates it.
    const P n{forward.normal.x, forward.normal.y};
    const double depth = forward.penetration;
    REQUIRE(Clearance(a, b, n * (depth * (1 + 1e-4) + 1e-6 * scale)) > 0);
    REQUIRE(Clearance(a, b, n * (depth * (1 - 1e-3))) < 0);
    REQUIRE(forward.contactCount >= 1);
    REQUIRE(forward.contactCount == reverse.contactCount);
    std::set<std::uint32_t> features;
    for (std::uint8_t i = 0; i < forward.contactCount; ++i) {
        features.insert(forward.contacts[i].featureId);
        REQUIRE(forward.contacts[i].penetration <= forward.penetration);
        REQUIRE(forward.contacts[i].featureId == reverse.contacts[i].featureId);
        REQUIRE(forward.contacts[i].position.x == reverse.contacts[i].position.x);
        REQUIRE(forward.contacts[i].position.y == reverse.contacts[i].position.y);
    }
    REQUIRE(features.size() == forward.contactCount);
}
}

TEST_CASE("Capsule construction validates its dimensions", "[capsule][validation]") {
    REQUIRE_THROWS_AS(Capsule(-1, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(Capsule(-0.0f, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(Capsule(1, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(Capsule(1, -1), std::invalid_argument);
    const float inf = std::numeric_limits<float>::infinity(), nan = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(Capsule(inf, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(Capsule(nan, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(Capsule(1, nan), std::invalid_argument);
    const float maximum = std::numeric_limits<float>::max();
    REQUIRE_THROWS_AS(Capsule(maximum, maximum), std::invalid_argument); // Extent overflows.
    REQUIRE_THROWS_AS(Capsule(1e30f, 1e30f), std::invalid_argument);   // Area overflows.
    REQUIRE_NOTHROW(Capsule(0, 1));
    REQUIRE_NOTHROW(Capsule(-0.0f, 1));

    const Capsule capsule(2.5f, 0.75f);
    REQUIRE(capsule.type == ShapeType::CAPSULE);
    REQUIRE(capsule.GetHalfLength() == 2.5f);
    REQUIRE(capsule.GetRadius() == 0.75f);
    const auto clone = capsule.Clone();
    const auto* copy = dynamic_cast<const Capsule*>(clone.get());
    REQUIRE(copy != nullptr);
    REQUIRE(copy != &capsule);
    REQUIRE(copy->GetHalfLength() == 2.5f);
    REQUIRE(copy->GetRadius() == 0.75f);
}

TEST_CASE("Capsule mass properties match independent slice quadrature", "[capsule][numerical]") {
    // Integrate horizontal slices y = r sin(t): each spans |x| <= h + r cos(t).
    // This decomposition differs from the rectangle-plus-caps formula.
    const auto quadrature = [](double h, double r) {
        constexpr int intervals = 4000;
        double area = 0, moment = 0;
        for (int i = 0; i <= intervals; ++i) {
            const double t = -Pi / 2 + Pi * i / intervals;
            const double weight = (i == 0 || i == intervals) ? 1 : (i % 2 ? 4 : 2);
            const double y = r * std::sin(t), dy = r * std::cos(t);
            const double half = h + r * std::cos(t);
            area += weight * 2 * half * dy;
            moment += weight * (2 * half * half * half / 3 + 2 * half * y * y) * dy;
        }
        const double step = Pi / intervals / 3;
        return std::pair<double, double>{area * step, moment * step};
    };
    for (const auto& [h, r] : std::vector<std::pair<float, float>>{
             {0.0f, 1.0f}, {2.0f, 0.5f}, {0.25f, 3.0f}, {1e-3f, 1.0f}, {1000.0f, 1e-3f}, {7.5f, 7.5f}}) {
        const Capsule capsule(h, r);
        const auto [area, polar] = quadrature(h, r);
        REQUIRE(capsule.GetArea() == Approx(area).epsilon(1e-6));
        const float mass = 3.5f;
        REQUIRE(capsule.GetInertia(mass) == Approx(mass * polar / area).epsilon(1e-6));
        const RigidBody body(capsule, Material{2.0f});
        REQUIRE(body.GetMass() == Approx(2.0 * area).epsilon(1e-6));
    }
    // The zero-length spine is exactly the circle limit.
    REQUIRE(Capsule(0, 1.5f).GetArea() == Approx(Circle(1.5f).GetArea()).epsilon(1e-7));
    REQUIRE(Capsule(0, 1.5f).GetInertia(2) == Approx(Circle(1.5f).GetInertia(2)).epsilon(1e-7));
}

TEST_CASE("Capsule bounds are conservative under rotation and translation", "[capsule][BroadPhase]") {
    const Capsule capsule(2, 0.5f);
    for (int k = 0; k < 64; ++k) {
        const float angle = static_cast<float>(-Pi + 2 * Pi * k / 64);
        for (const Vector2 position : {Vector2{0, 0}, Vector2{1e6f, -3e5f}, Vector2{1e30f, 1e30f}}) {
            RigidBody body(capsule, Material{}, position);
            body.SetOrientation(angle);
            const auto bounds = body.GetAABB();
            const double reachX = 2 * std::abs(std::cos(double(angle))) + 0.5;
            const double reachY = 2 * std::abs(std::sin(double(angle))) + 0.5;
            REQUIRE(double(bounds.min.x) <= position.x - reachX);
            REQUIRE(double(bounds.max.x) >= position.x + reachX);
            REQUIRE(double(bounds.min.y) <= position.y - reachY);
            REQUIRE(double(bounds.max.y) >= position.y + reachY);
            if (position.x == 0) {
                REQUIRE(bounds.max.x == Approx(reachX).epsilon(1e-6));
                REQUIRE(bounds.max.y == Approx(reachY).epsilon(1e-6));
            }
        }
    }
    RigidBody edge(capsule, Material{}, {std::numeric_limits<float>::max(), 0});
    REQUIRE_THROWS_AS(edge.GetAABB(), std::overflow_error);
    RigidBody invalid(capsule, Material{});
    invalid.orientation = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(invalid.GetAABB(), std::invalid_argument);

    class TaggedShape final : public Shape {
      public:
        TaggedShape() : Shape(ShapeType::CAPSULE) {}
        float GetArea() const override { return 1; }
        float GetInertia(float) const override { return 1; }
        std::unique_ptr<Shape> Clone() const override { return std::make_unique<TaggedShape>(*this); }
    };
    RigidBody malformed(TaggedShape{}, Material{});
    RigidBody circle(Circle(1), Material{});
    REQUIRE_THROWS_AS(malformed.GetAABB(), std::invalid_argument);
    REQUIRE_THROWS_AS(CheckCollision(&malformed, &circle), std::invalid_argument);
    REQUIRE_THROWS_AS(CheckCollision(&circle, &malformed), std::invalid_argument);
}

TEST_CASE("Circle-capsule manifolds cover sides, caps, touching and the spine", "[capsule][collision]") {
    const Capsule capsule(2, 0.5f);
    auto body = Make(capsule, {0, 0});
    SECTION("side contact") {
        auto circle = Make(Circle(1), {0.5f, 1.25f});
        const auto m = CheckCollision(circle.get(), body.get());
        REQUIRE(m.hasCollision);
        REQUIRE(m.contactCount == 1);
        REQUIRE(m.penetration == Approx(0.25));
        REQUIRE(m.normal.x == Approx(0).margin(1e-7));
        REQUIRE(m.normal.y == Approx(-1));
        REQUIRE(m.contacts[0].position.x == Approx(0.5));
        REQUIRE(m.contacts[0].position.y == Approx(0.375)); // Midway between 0.25 and 0.5.
        const auto r = CheckCollision(body.get(), circle.get());
        REQUIRE(r.A == body.get());
        REQUIRE(r.normal.y == Approx(1));
        REQUIRE(r.contacts[0].position.y == m.contacts[0].position.y);
    }
    SECTION("cap contact") {
        auto circle = Make(Circle(1), {3.25f, 0});
        const auto m = CheckCollision(body.get(), circle.get());
        REQUIRE(m.hasCollision);
        REQUIRE(m.penetration == Approx(0.25));
        REQUIRE(m.normal.x == Approx(1));
        REQUIRE(m.contacts[0].position.x == Approx(2.375));
    }
    SECTION("exact touching is not overlap") {
        auto side = Make(Circle(1), {0, 1.5f});
        auto cap = Make(Circle(1), {-3.5f, 0});
        REQUIRE_FALSE(CheckCollision(body.get(), side.get()).hasCollision);
        REQUIRE_FALSE(CheckCollision(cap.get(), body.get()).hasCollision);
    }
    SECTION("center on the spine escapes perpendicular to it") {
        auto circle = Make(Circle(1), {1, 0});
        const auto m = CheckCollision(body.get(), circle.get());
        REQUIRE(m.hasCollision);
        REQUIRE(m.penetration == Approx(1.5));
        REQUIRE(m.normal.x == Approx(0).margin(1e-7));
        REQUIRE(std::abs(m.normal.y) == Approx(1));
    }
    SECTION("zero spine length matches circle-circle") {
        auto point = Make(Capsule(0, 0.5f), {0, 0});
        auto disk = Make(Circle(0.5f), {0, 0});
        for (const Vector2 position : {Vector2{0.3f, 0.4f}, Vector2{-0.7f, 0.1f}, Vector2{0.05f, -0.9f}}) {
            auto circle = Make(Circle(1), position);
            const auto capsuleManifold = CheckCollision(point.get(), circle.get());
            const auto circleManifold = CollisionCircleCircle(disk.get(), circle.get());
            REQUIRE(capsuleManifold.hasCollision);
            REQUIRE(capsuleManifold.penetration == Approx(circleManifold.penetration).epsilon(1e-6));
            REQUIRE(capsuleManifold.normal.x == Approx(circleManifold.normal.x).margin(1e-6));
            REQUIRE(capsuleManifold.normal.y == Approx(circleManifold.normal.y).margin(1e-6));
        }
    }
    SECTION("covariance under rotation and large translation") {
        auto circle = Make(Circle(1), {0.5f, 1.25f});
        const auto reference = CheckCollision(circle.get(), body.get());
        for (const float angle : {0.3f, 1.0f, -2.5f}) {
            const Vector2 offset{1024, -512}; // Float spacing here is about 1e-4.
            const double c = std::cos(double(angle)), s = std::sin(double(angle));
            auto rotatedCapsule = Make(capsule, offset, angle);
            auto rotatedCircle = Make(Circle(1), {static_cast<float>(offset.x + c * 0.5 - s * 1.25),
                                                  static_cast<float>(offset.y + s * 0.5 + c * 1.25)});
            const auto m = CheckCollision(rotatedCircle.get(), rotatedCapsule.get());
            REQUIRE(m.hasCollision);
            REQUIRE(m.penetration == Approx(reference.penetration).margin(2e-4));
            REQUIRE(m.normal.x == Approx(c * reference.normal.x - s * reference.normal.y).margin(1e-4));
            REQUIRE(m.normal.y == Approx(s * reference.normal.x + c * reference.normal.y).margin(1e-4));
        }
    }
}

TEST_CASE("Capsule-capsule side contacts produce two clipped points", "[capsule][collision]") {
    const Capsule capsule(2, 0.5f);
    auto lower = Make(capsule, {0, 0});
    auto upper = Make(capsule, {1, 0.875f});
    const auto m = CheckCollision(lower.get(), upper.get());
    REQUIRE(m.hasCollision);
    REQUIRE(m.contactCount == 2);
    REQUIRE(m.normal.x == Approx(0).margin(1e-7));
    REQUIRE(m.normal.y == Approx(1));
    REQUIRE(m.penetration == Approx(0.125));
    REQUIRE(m.contacts[0].position.x == Approx(-1));
    REQUIRE(m.contacts[1].position.x == Approx(2));
    for (int i = 0; i < 2; ++i) {
        REQUIRE(m.contacts[i].position.y == Approx(0.4375));
        REQUIRE(m.contacts[i].penetration == Approx(0.125));
    }
    REQUIRE(m.contacts[0].featureId != m.contacts[1].featureId);
    const auto r = CheckCollision(upper.get(), lower.get());
    REQUIRE(r.normal.y == Approx(-1));
    REQUIRE(r.contacts[0].position.x == m.contacts[0].position.x);
    REQUIRE(r.contacts[1].position.x == m.contacts[1].position.x);

    // Slight tilt keeps both points only while both penetrate.
    auto tilted = Make(capsule, {0, 0.875f}, 0.02f);
    const auto t = CheckCollision(lower.get(), tilted.get());
    REQUIRE(t.hasCollision);
    REQUIRE(t.contactCount == 2);
    auto steep = Make(capsule, {0, 0.875f + 2 * std::sin(0.3f)}, 0.3f);
    REQUIRE(CheckCollision(lower.get(), steep.get()).contactCount == 1);

    // End-to-end collinear overlap uses the cap direction and one point.
    auto collinear = Make(capsule, {4.875f, 0});
    const auto e = CheckCollision(lower.get(), collinear.get());
    REQUIRE(e.hasCollision);
    REQUIRE(e.contactCount == 1);
    REQUIRE(e.normal.x == Approx(1));
    REQUIRE(e.penetration == Approx(0.125));
    auto touching = Make(capsule, {5, 0});
    REQUIRE_FALSE(CheckCollision(lower.get(), touching.get()).hasCollision);
}

TEST_CASE("Crossing and contained capsules report the separating translation", "[capsule][collision]") {
    SECTION("perpendicular crossing spines") {
        auto horizontal = Make(Capsule(2, 0.25f), {0, 0});
        auto vertical = Make(Capsule(1, 0.25f), {0, 0}, static_cast<float>(Pi / 2));
        const auto m = CheckCollision(horizontal.get(), vertical.get());
        REQUIRE(m.hasCollision);
        // Sliding the short capsule along its own spine is cheaper than
        // crossing the long one; the closest spine distance would report 0.5.
        REQUIRE(m.penetration == Approx(1.5));
        REQUIRE(m.normal.x == Approx(0).margin(1e-6));
        CheckAgainstOracle(*horizontal, *vertical, 10);
    }
    SECTION("identical coincident capsules") {
        auto a = Make(Capsule(1, 0.5f), {0, 0});
        auto b = Make(Capsule(1, 0.5f), {0, 0});
        const auto m = CheckCollision(a.get(), b.get());
        REQUIRE(m.penetration == Approx(1));
        REQUIRE(m.normal.x == Approx(0).margin(1e-7));
        CheckAgainstOracle(*a, *b, 10);
    }
    SECTION("small capsule inside a large one") {
        auto outer = Make(Capsule(3, 2), {0, 0});
        auto inner = Make(Capsule(0.5f, 0.25f), {0.5f, 0.75f}, 0.3f);
        CheckAgainstOracle(*outer, *inner, 20);
    }
    SECTION("long perpendicular spines with deep overlap") {
        auto a = Make(Capsule(10, 0.1f), {0, 0}, 0.2f);
        auto b = Make(Capsule(10, 0.1f), {0.5f, -0.25f}, 0.2f + static_cast<float>(Pi / 2));
        CheckAgainstOracle(*a, *b, 40);
    }
    SECTION("near-parallel spines") {
        auto a = Make(Capsule(2, 0.5f), {0, 0});
        auto b = Make(Capsule(2, 0.5f), {0.5f, 0.75f}, 1e-6f);
        CheckAgainstOracle(*a, *b, 10);
        REQUIRE(CheckCollision(a.get(), b.get()).contactCount == 2);
    }
}

TEST_CASE("Rounded-segment manifolds match the translation oracle", "[capsule][collision][property]") {
    std::uint64_t state = 0x5eedu;
    int overlapping = 0;
    for (int trial = 0; trial < 160; ++trial) {
        const bool circleFirst = trial % 5 == 0;
        auto a = circleFirst ? Make(Circle(static_cast<float>(Uniform(state, 0.1, 1))), {0, 0})
                             : Make(Capsule(static_cast<float>(Uniform(state, 0, 2)),
                                            static_cast<float>(Uniform(state, 0.05, 1))),
                                    {0, 0}, static_cast<float>(Uniform(state, -Pi, Pi)));
        const float halfLength = trial % 7 == 0 ? 0.0f : static_cast<float>(Uniform(state, 0, 2));
        auto b = Make(Capsule(halfLength, static_cast<float>(Uniform(state, 0.05, 1))),
                      {static_cast<float>(Uniform(state, -2, 2)), static_cast<float>(Uniform(state, -2, 2))},
                      static_cast<float>(Uniform(state, -Pi, Pi)));
        overlapping += Clearance(*a, *b, {0, 0}) < 0;
        CheckAgainstOracle(*a, *b, 12);
    }
    REQUIRE(overlapping > 60);
}

TEST_CASE("Capsule-polygon manifolds handle faces, corners and containment", "[capsule][collision]") {
    auto ground = Make(Polygon::MakeBox(10, 1), {0, 0}, 0, true);
    const Capsule capsule(1, 0.5f);
    SECTION("resting side produces two face contacts") {
        auto body = Make(capsule, {0, 0.875f});
        const auto m = CheckCollision(body.get(), ground.get());
        REQUIRE(m.hasCollision);
        REQUIRE(m.contactCount == 2);
        REQUIRE(m.normal.y == Approx(-1));
        REQUIRE(m.penetration == Approx(0.125));
        REQUIRE(std::min(m.contacts[0].position.x, m.contacts[1].position.x) == Approx(-1));
        REQUIRE(std::max(m.contacts[0].position.x, m.contacts[1].position.x) == Approx(1));
        REQUIRE(m.contacts[0].position.y == Approx(0.4375));
        REQUIRE(m.contacts[1].position.y == Approx(0.4375));
        const auto r = CheckCollision(ground.get(), body.get());
        REQUIRE(r.normal.y == Approx(1));
        REQUIRE(r.contacts[1].position.x == m.contacts[1].position.x);
    }
    SECTION("an overhang is clipped to the face") {
        auto body = Make(capsule, {4.5f, 0.875f});
        const auto m = CheckCollision(ground.get(), body.get());
        REQUIRE(m.contactCount == 2);
        REQUIRE(std::min(m.contacts[0].position.x, m.contacts[1].position.x) == Approx(3.5));
        REQUIRE(std::max(m.contacts[0].position.x, m.contacts[1].position.x) == Approx(5));
    }
    SECTION("a tilted capsule touches with one end") {
        auto body = Make(capsule, {0, 0.875f + std::sin(0.5f)}, 0.5f);
        const auto m = CheckCollision(ground.get(), body.get());
        REQUIRE(m.contactCount == 1);
        REQUIRE(m.penetration == Approx(0.125).margin(1e-6));
        REQUIRE(m.contacts[0].position.x == Approx(-std::cos(0.5)).margin(1e-6));
    }
    SECTION("exact touching is not overlap") {
        REQUIRE_FALSE(CheckCollision(ground.get(), Make(capsule, {0, 1}).get()).hasCollision);
        REQUIRE_FALSE(CheckCollision(Make(capsule, {6.5f, 0}).get(), ground.get()).hasCollision);
    }
    SECTION("cap against a box corner") {
        auto body = Make(capsule, {6.25f, 0.75f});
        CheckAgainstOracle(*ground, *body, 10);
        REQUIRE(CheckCollision(ground.get(), body.get()).contactCount == 1);
    }
    SECTION("deep crossing and containment") {
        auto box = Make(Polygon::MakeBox(2, 2), {0, 0}, 0.4f);
        CheckAgainstOracle(*box, *Make(Capsule(5, 0.1f), {0.2f, 0.1f}, 1.9f), 30);
        CheckAgainstOracle(*box, *Make(Capsule(0.25f, 0.1f), {0.1f, -0.2f}, 0.7f), 30);
        CheckAgainstOracle(*Make(Capsule(4, 3), {0, 0}), *Make(Polygon::MakeBox(1, 0.5f), {0.5f, 1}, 0.2f), 30);
        CheckAgainstOracle(*box, *Make(Capsule(0, 0.5f), {0.3f, 0.3f}), 30);
    }
}

TEST_CASE("Capsule-polygon manifolds match the translation oracle", "[capsule][collision][property]") {
    std::uint64_t state = 0xc0ffeeu;
    const std::vector<std::vector<Vector2>> outlines{
        {{-1, -0.5f}, {1, -0.5f}, {1, 0.5f}, {-1, 0.5f}},
        {{0, 0}, {2, 0}, {0.5f, 1.5f}},
        {{1.5f, 2}, {3, 2}, {3.5f, 3}, {2, 3.5f}}, // Offset from the local origin, clockwise below.
        {{0.8f, 0}, {0.4f, 0.69f}, {-0.4f, 0.69f}, {-0.8f, 0}, {-0.4f, -0.69f}, {0.4f, -0.69f}}};
    int overlapping = 0;
    for (int trial = 0; trial < 160; ++trial) {
        auto vertices = outlines[trial % outlines.size()];
        if (trial % 8 == 2) std::reverse(vertices.begin(), vertices.end());
        auto polygon = Make(Polygon(vertices), {static_cast<float>(Uniform(state, -0.5, 0.5)),
                                                static_cast<float>(Uniform(state, -0.5, 0.5))},
                            static_cast<float>(Uniform(state, -Pi, Pi)));
        const auto outline = OutlineOf(*polygon);
        P centroid{0, 0};
        for (const auto& v : outline) centroid = centroid + v * (1.0 / outline.size());
        auto capsule = Make(Capsule(trial % 9 == 0 ? 0.0f : static_cast<float>(Uniform(state, 0, 2)),
                                    static_cast<float>(Uniform(state, 0.05, 0.8))),
                            {static_cast<float>(centroid.x + Uniform(state, -2, 2)),
                             static_cast<float>(centroid.y + Uniform(state, -2, 2))},
                            static_cast<float>(Uniform(state, -Pi, Pi)));
        overlapping += Clearance(*capsule, *polygon, {0, 0}) < 0;
        if (trial % 2) CheckAgainstOracle(*capsule, *polygon, 14);
        else CheckAgainstOracle(*polygon, *capsule, 14);
    }
    REQUIRE(overlapping > 60);
}

TEST_CASE("Capsule queries cover points, circles, rays and sweeps", "[capsule][queries]") {
    const Capsule capsule(2, 1);
    SECTION("point and circle overlap are closed") {
        REQUIRE(ContainsPoint(capsule, {0, 1}));
        REQUIRE(ContainsPoint(capsule, {3, 0}));
        REQUIRE_FALSE(ContainsPoint(capsule, {2.8f, 0.8f}));
        REQUIRE(ContainsPoint(capsule, {0, -2}, {0, 0}, static_cast<float>(Pi / 2)));
        REQUIRE_FALSE(ContainsPoint(capsule, {0, -2}));
        REQUIRE(OverlapsCircle(capsule, {0, 3}, 2));
        REQUIRE(OverlapsCircle(capsule, {5, 0}, 2));
        REQUIRE_FALSE(OverlapsCircle(capsule, {5.5f, 0}, 2));
        REQUIRE(OverlapsCircle(capsule, {0, 1}, 0)); // Delegates to the contained boundary.
        const Capsule point(0, 1.5f);
        for (const Vector2 p : {Vector2{1, 1}, Vector2{1.2f, 0.9f}, Vector2{0, 1.5f}})
            REQUIRE(ContainsPoint(point, p) == ContainsPoint(Circle(1.5f), p));
    }
    SECTION("rays hit sides, caps and tangents") {
        auto side = RayCast(capsule, {0, 5}, {0, -5});
        REQUIRE(side);
        REQUIRE(side->fraction == Approx(0.4));
        REQUIRE(side->point.y == Approx(1));
        REQUIRE(side->normal.y == Approx(1));
        auto cap = RayCast(capsule, {10, 0}, {0, 0});
        REQUIRE(cap);
        REQUIRE(cap->fraction == Approx(0.7));
        REQUIRE(cap->normal.x == Approx(1));
        auto tangent = RayCast(capsule, {-10, 1}, {10, 1});
        REQUIRE(tangent);
        REQUIRE(tangent->point.y == Approx(1));
        REQUIRE(tangent->fraction == Approx(0.4));
        REQUIRE_FALSE(RayCast(capsule, {-10, 1.0001f}, {10, 1.0001f}));
        auto inside = RayCast(capsule, {0.5f, 0.2f}, {10, 10});
        REQUIRE(inside);
        REQUIRE(inside->fraction == 0);
        REQUIRE(inside->normal.x == 0);
        REQUIRE(RayCast(capsule, {1, 0}, {1, 0}));
        REQUIRE_FALSE(RayCast(capsule, {5, 0}, {5, 0}));
        auto limit = RayCast(capsule, {0, 5}, {0, 1});
        REQUIRE(limit);
        REQUIRE(limit->fraction == 1);
        REQUIRE_FALSE(RayCast(capsule, {0, 5}, {0, 1.001f}));
        auto rotated = RayCast(capsule, {5, 0}, {-5, 0}, {0, 0}, static_cast<float>(Pi / 2));
        REQUIRE(rotated);
        REQUIRE(rotated->fraction == Approx(0.4));
        REQUIRE(rotated->normal.x == Approx(1));
    }
    SECTION("swept circles stop on the expanded capsule") {
        auto side = SweepCircle(capsule, {0, 5}, {0, -5}, 0.5f);
        REQUIRE(side);
        REQUIRE(side->fraction == Approx(0.35));
        REQUIRE(side->center.y == Approx(1.5));
        REQUIRE(side->contactPoint.y == Approx(1));
        REQUIRE(side->normal.y == Approx(1));
        auto cap = SweepCircle(capsule, {-10, 0}, {0, 0}, 1);
        REQUIRE(cap);
        REQUIRE(cap->center.x == Approx(-4));
        REQUIRE(cap->contactPoint.x == Approx(-3));
        REQUIRE(cap->normal.x == Approx(-1));
        auto initial = SweepCircle(capsule, {0, 1.4f}, {0, 10}, 0.5f);
        REQUIRE(initial);
        REQUIRE(initial->fraction == 0);
        REQUIRE_FALSE(SweepCircle(capsule, {0, 5}, {10, 5}, 0.5f));
        auto zero = SweepCircle(capsule, {0, 5}, {0, -5}, 0);
        REQUIRE(zero);
        REQUIRE(zero->fraction == Approx(0.4));
    }
    SECTION("world queries include capsule bodies") {
        World world;
        auto body = Make(capsule, {10, 0}, 0, true);
        world.addBody(body);
        REQUIRE(QueryPoint(world, {12.5f, 0}).size() == 1);
        REQUIRE(QueryCircle(world, {10, 3}, 2.0f).size() == 1);
        const auto ray = RayCastNearest(world, {10, 10}, {10, -10});
        REQUIRE(ray);
        REQUIRE(ray->body == body);
        REQUIRE(ray->hit.point.y == Approx(1));
        REQUIRE(SweepCircleNearest(world, {0, 0}, {20, 0}, 0.5f)->hit.center.x == Approx(6.5));
    }
}

TEST_CASE("Capsules rest, bounce and conserve momentum in a World", "[capsule][world]") {
    SECTION("a dropped capsule settles on its side") {
        World world;
        auto ground = Make(Polygon::MakeBox(20, 1), {0, -0.5f}, 0, true);
        auto body = Make(Capsule(1, 0.25f), {0, 2}, 0.3f);
        world.addBody(ground); world.addBody(body);
        world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
        for (int i = 0; i < 300; ++i) world.step(1.0f / 60);
        REQUIRE(body->position.y == Approx(0.25).margin(0.02));
        REQUIRE(std::abs(std::remainder(double(body->orientation), Pi)) < 0.02);
        REQUIRE(body->velocity.magnitude() < 0.05f);
        REQUIRE(std::abs(body->angularVelocity) < 0.05f);
    }
    SECTION("elastic head-on capsules exchange velocities") {
        SimulationConfig config;
        config.restitutionVelocityThreshold = 0;
        World world(config);
        Material elastic{1, 1, 0, 0};
        auto left = Make(Capsule(1, 0.5f), {-3, 0}, 0, false, elastic);
        auto right = Make(Capsule(1, 0.5f), {3, 0}, 0, false, elastic);
        left->SetVelocity({4, 0}); right->SetVelocity({-2, 0});
        world.addBody(left); world.addBody(right);
        const double momentum = left->GetMass() * 4.0 - right->GetMass() * 2.0;
        for (int i = 0; i < 120; ++i) world.step(1.0f / 120);
        REQUIRE(left->velocity.x == Approx(-2).margin(1e-3));
        REQUIRE(right->velocity.x == Approx(4).margin(1e-3));
        REQUIRE(left->GetMass() * left->velocity.x + right->GetMass() * right->velocity.x
                == Approx(momentum).margin(1e-3));
        REQUIRE(std::abs(left->angularVelocity) < 1e-4f);
    }
    SECTION("friction stops a sliding capsule and frictionless sliding continues") {
        for (const bool friction : {true, false}) {
            World world;
            Material material{1, 0, friction ? 0.6f : 0.0f, friction ? 0.5f : 0.0f};
            auto ground = Make(Polygon::MakeBox(200, 1), {0, -0.5f}, 0, true, material);
            auto body = Make(Capsule(1, 0.25f), {0, 0.245f}, 0, false, material);
            body->SetVelocity({3, 0});
            world.addBody(ground); world.addBody(body);
            world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
            for (int i = 0; i < 120; ++i) world.step(1.0f / 60);
            if (friction) REQUIRE(std::abs(body->velocity.x) < 0.05f);
            else REQUIRE(body->velocity.x == Approx(3).margin(0.02));
        }
    }
    SECTION("filtering, events and both broad phases") {
        for (const bool grid : {false, true}) {
            World world;
            if (grid) world.setBroadPhase(std::make_unique<UniformGrid>(1.5f));
            else world.setBroadPhase(std::make_unique<SweepAndPrune>());
            auto a = Make(Capsule(1, 0.5f), {0, 0}, 0, true);
            auto b = Make(Capsule(1, 0.5f), {0.5f, 0.9f}, 0.4f);
            auto filtered = Make(Capsule(1, 0.5f), {-0.5f, -0.9f});
            filtered->SetCollisionMaskBits(0);
            world.addBody(a); world.addBody(b); world.addBody(filtered);
            struct Listener : ICollisionListener {
                int begins = 0, persists = 0, ends = 0;
                void onCollisionBegin(const CollisionEvent&) override { ++begins; }
                void onCollisionPersist(const CollisionEvent&) override { ++persists; }
                void onCollisionEnd(const CollisionEvent&) override { ++ends; }
            } listener;
            world.addCollisionListener(&listener);
            world.step(0);
            world.step(0);
            REQUIRE(listener.begins == 1);
            REQUIRE(listener.persists == 1);
            b->SetPosition({10, 10});
            world.step(0);
            REQUIRE(listener.ends == 1);
        }
    }
    SECTION("state export names capsule dimensions") {
        World world;
        world.addBody(Make(Capsule(1.5f, 0.25f), {1, 2}));
        const std::string json = ExportWorldJson(world, 0);
        REQUIRE(json.find("\"type\":\"capsule\",\"halfLength\":1.5,\"radius\":0.25") != std::string::npos);
    }
}

TEST_CASE("Capsule CCD sweeps circles against static capsules only", "[capsule][ccd]") {
    SECTION("standalone sweep geometry") {
        const auto side = SweepCircleCapsule({0, 5}, {0, -10}, 0.5f, {-2, 0}, {2, 0}, 1);
        REQUIRE(side.hit);
        REQUIRE(side.fraction == Approx(0.35));
        REQUIRE(side.normal.y == Approx(-1));
        REQUIRE(side.point.y == Approx(1));
        const auto cap = SweepCircleCapsule({-10, 0}, {10, 0}, 1, {-2, 0}, {2, 0}, 1);
        REQUIRE(cap.hit);
        REQUIRE(cap.fraction == Approx(0.6));
        REQUIRE(cap.point.x == Approx(-3));
        // Prescribed capsule translation is relative motion.
        const auto moving = SweepCircleCapsule({0, 5}, {0, 0}, 0.5f, {-2, 0}, {2, 0}, 1, {0, 10});
        REQUIRE(moving.hit);
        REQUIRE(moving.fraction == Approx(0.35));
        REQUIRE(moving.point.y == Approx(4.5));
        REQUIRE_FALSE(SweepCircleCapsule({0, 5}, {10, 0}, 0.5f, {-2, 0}, {2, 0}, 1).hit);
        const auto overlap = SweepCircleCapsule({0, 1}, {0, 5}, 0.5f, {-2, 0}, {2, 0}, 1);
        REQUIRE(overlap.hit);
        REQUIRE(overlap.fraction == 0);
        REQUIRE(overlap.normal.y == Approx(-1));
        REQUIRE_THROWS_AS(SweepCircleCapsule({0, 1}, {0, 5}, 0.5f, {-2, 0}, {2, 0}, 0), std::invalid_argument);
    }
    SECTION("a fast circle does not tunnel through a thin static capsule") {
        for (const bool ccd : {false, true}) {
            SimulationConfig config;
            config.enableLinearVelocityLimit = false;
            World world(config);
            auto wall = Make(Capsule(5, 0.05f), {0, 0}, static_cast<float>(Pi / 2), true);
            auto bullet = Make(Circle(0.1f), {-5, 0});
            bullet->SetVelocity({600, 0});
            bullet->SetCcdEnabled(ccd);
            world.addBody(wall); world.addBody(bullet);
            world.step(1.0f / 60);
            if (ccd) {
                REQUIRE(bullet->position.x <= 0);
                REQUIRE(bullet->velocity.x <= 0);
                REQUIRE(world.getLastStepStatistics().ccdImpactCount == 1);
            } else {
                REQUIRE(bullet->position.x > 0);
            }
        }
    }
    SECTION("unsupported capsule CCD is rejected before state advances") {
        const auto attempt = [](RigidBodyPtr first, RigidBodyPtr second) {
            World world;
            auto particles = std::make_shared<ParticleSystem>();
            particles->addParticle({0, 0}, {1, 0});
            world.addParticleSystem(particles);
            world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
            first->SetVelocity({1, 2});
            second->SetVelocity({-3, 0});
            world.addBody(first); world.addBody(second);
            const auto p1 = first->position, p2 = second->position;
            const auto v1 = first->velocity, v2 = second->velocity;
            REQUIRE_THROWS_AS(world.step(1.0f / 60), std::invalid_argument);
            REQUIRE(first->position == p1);
            REQUIRE(second->position == p2);
            REQUIRE(first->velocity == v1);
            REQUIRE(second->velocity == v2);
            REQUIRE(first->force == Vector2{});
            REQUIRE(particles->getParticles()[0].position == Vector2{});
            REQUIRE(particles->getParticles()[0].velocity == Vector2{1, 0});
            REQUIRE_NOTHROW(world.step(0)); // Zero duration runs no sweep.
        };
        auto dynamicCapsule = Make(Capsule(1, 0.5f), {0, 0});
        dynamicCapsule->SetCcdEnabled(true);
        attempt(dynamicCapsule, Make(Circle(0.5f), {5, 0}));
        auto circle = Make(Circle(0.5f), {5, 0});
        circle->SetCcdEnabled(true);
        attempt(Make(Capsule(1, 0.5f), {0, 0}), circle);
        auto box = Make(Polygon::MakeBox(1, 1), {5, 0});
        box->SetCcdEnabled(true);
        attempt(Make(Capsule(1, 0.5f), {0, 0}, 0, true), box);
        auto other = Make(Capsule(1, 0.5f), {5, 0});
        other->SetCcdEnabled(true);
        attempt(Make(Capsule(1, 0.5f), {0, 0}, 0, true), other);
    }
    SECTION("filtered capsule pairs do not request CCD") {
        World world;
        auto capsule = Make(Capsule(1, 0.5f), {0, 0});
        auto circle = Make(Circle(0.5f), {5, 0});
        circle->SetCcdEnabled(true);
        circle->SetCollisionMaskBits(0);
        world.addBody(capsule); world.addBody(circle);
        REQUIRE_NOTHROW(world.step(1.0f / 60));
    }
}

TEST_CASE("Fluid coupling rejects capsules before any change", "[capsule][fluids]") {
    auto capsule = Make(Capsule(1, 0.5f), {0, 0});
    auto circle = Make(Circle(0.5f), {3, 0});
    std::vector<FluidParticle> particles{FluidParticle({2.6f, 0}, {1, 0})};
    const auto before = particles[0];
    FluidRigidCoupler coupler;
    REQUIRE_THROWS_AS(coupler.couple(particles, {circle.get(), capsule.get()}, 0.01f), std::invalid_argument);
    REQUIRE(particles[0].velocity == before.velocity);
    REQUIRE(circle->velocity == Vector2{});
    REQUIRE(circle->angularVelocity == 0);
    REQUIRE_THROWS_AS(SampleRigidBodyBoundaries({capsule.get()}, FluidBoundarySamplingSettings{}),
                      std::invalid_argument);
}
