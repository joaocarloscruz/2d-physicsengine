#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

using namespace PhysicsEngine;
using Catch::Approx;

namespace {
void CheckVector(Vector2 actual, Vector2 expected, double margin = 1e-5) {
    REQUIRE(std::isfinite(actual.x));
    REQUIRE(std::isfinite(actual.y));
    REQUIRE(actual.x == Approx(expected.x).margin(margin));
    REQUIRE(actual.y == Approx(expected.y).margin(margin));
}
Vector2 TransformPoint(Vector2 p, Vector2 position, float angle) {
    const double c = std::cos(static_cast<double>(angle));
    const double s = std::sin(static_cast<double>(angle));
    return {static_cast<float>(position.x + c * p.x - s * p.y),
        static_cast<float>(position.y + s * p.x + c * p.y)};
}
RigidBodyPtr Body(const Shape& shape, Vector2 position = {}) {
    return std::make_shared<RigidBody>(shape, Material{}, position, true);
}
}

TEST_CASE("Circle point queries include boundaries and use the body transform", "[queries]") {
    Circle circle(2);
    REQUIRE(ContainsPoint(circle, {0, 0}));
    REQUIRE(ContainsPoint(circle, {2, 0}));
    REQUIRE(ContainsPoint(circle, {0, -2}));
    REQUIRE_FALSE(ContainsPoint(circle, {2.00001f, 0}));
    REQUIRE_FALSE(ContainsPoint(circle, {2, 2}));
    auto body = Body(circle, {10, -4});
    body->SetOrientation(0.7f);
    REQUIRE(ContainsPoint(*body, {12, -4}));
    REQUIRE_FALSE(ContainsPoint(*body, {12.01f, -4}));
}

TEST_CASE("Circle finite rays report entry point fraction and outward normal", "[queries]") {
    Circle circle(1);
    auto hit = RayCast(circle, {-3, 0}, {3, 0});
    REQUIRE(hit);
    REQUIRE(hit->fraction == Approx(1.0 / 3.0));
    CheckVector(hit->point, {-1, 0});
    CheckVector(hit->normal, {-1, 0});
    hit = RayCast(circle, {0, 4}, {0, -4}, {0, 1});
    REQUIRE(hit);
    REQUIRE(hit->fraction == Approx(0.25));
    CheckVector(hit->point, {0, 2});
    CheckVector(hit->normal, {0, 1});
    hit = RayCast(circle, {-2, 0.6f}, {2, 0.6f});
    REQUIRE(hit);
    REQUIRE(hit->fraction == Approx(0.3).margin(1e-8));
    CheckVector(hit->point, {-0.8f, 0.6f});
    CheckVector(hit->normal, {-0.8f, 0.6f});
    REQUIRE_FALSE(RayCast(circle, {-3, 0}, {-2, 0}));
    REQUIRE_FALSE(RayCast(circle, {2, 0}, {4, 0}));
}

TEST_CASE("Circle tangents and endpoint contacts are included", "[queries]") {
    Circle circle(1);
    auto hit = RayCast(circle, {-2, 1}, {2, 1});
    REQUIRE(hit);
    REQUIRE(hit->fraction == Approx(0.5));
    CheckVector(hit->point, {0, 1});
    CheckVector(hit->normal, {0, 1});
    hit = RayCast(circle, {-2, 0}, {-1, 0});
    REQUIRE(hit);
    REQUIRE(hit->fraction == 1);
    REQUIRE_FALSE(RayCast(circle, {-2, 1.000001f}, {2, 1.000001f}));
}

TEST_CASE("Contained starts and zero-length segments have a defined zero normal", "[queries]") {
    Circle circle(1);
    Polygon polygon = Polygon::MakeBox(2, 2);
    for (const Shape* shape : {static_cast<const Shape*>(&circle), static_cast<const Shape*>(&polygon)}) {
        for (Vector2 start : {Vector2{0, 0}, Vector2{1, 0}}) {
            for (Vector2 end : {start, Vector2{3, 0}, Vector2{0, 0}}) {
                const auto hit = RayCast(*shape, start, end);
                REQUIRE(hit);
                REQUIRE(hit->fraction == 0);
                CheckVector(hit->point, start);
                CheckVector(hit->normal, {});
            }
        }
        REQUIRE_FALSE(RayCast(*shape, {2, 2}, {2, 2}));
    }
}

TEST_CASE("Convex polygon point queries support winding and offset vertices", "[queries]") {
    const std::vector<Vector2> vertices{{2, 3}, {6, 3}, {6, 5}, {2, 5}};
    for (bool clockwise : {false, true}) {
        auto order = vertices;
        if (clockwise) std::reverse(order.begin(), order.end());
        Polygon polygon(order);
        REQUIRE(ContainsPoint(polygon, {4, 4}));
        REQUIRE(ContainsPoint(polygon, {2, 3}));
        REQUIRE(ContainsPoint(polygon, {6, 4}));
        REQUIRE_FALSE(ContainsPoint(polygon, {0, 0}));
        REQUIRE_FALSE(ContainsPoint(polygon, {6.001f, 4}));
        const Vector2 position{10, -7};
        const float angle = 0.7f;
        auto body = Body(polygon, position);
        body->SetOrientation(angle);
        REQUIRE(ContainsPoint(*body, TransformPoint({4, 4}, position, angle)));
        REQUIRE_FALSE(ContainsPoint(*body, TransformPoint({1, 4}, position, angle)));
        auto hit = RayCast(*body, TransformPoint({0, 4}, position, angle),
            TransformPoint({8, 4}, position, angle));
        REQUIRE(hit);
        REQUIRE(hit->fraction == Approx(0.25).margin(1e-6));
        CheckVector(hit->point, TransformPoint({2, 4}, position, angle));
        CheckVector(hit->normal, TransformPoint({-1, 0}, {}, angle));
    }
}

TEST_CASE("Box ray clipping covers all faces corners parallel edges and misses", "[queries]") {
    Polygon box = Polygon::MakeBox(2, 2);
    struct Case { Vector2 start, end, point, normal; double fraction; };
    const Case cases[] = {
        {{-3, 0}, {3, 0}, {-1, 0}, {-1, 0}, 1.0 / 3},
        {{3, 0}, {-3, 0}, {1, 0}, {1, 0}, 1.0 / 3},
        {{0, -3}, {0, 3}, {0, -1}, {0, -1}, 1.0 / 3},
        {{0, 3}, {0, -3}, {0, 1}, {0, 1}, 1.0 / 3},
        {{-2, -2}, {0, 0}, {-1, -1}, {0, -1}, 0.5},
        {{-2, 1}, {2, 1}, {-1, 1}, {-1, 0}, 0.25},
        {{-2, 0}, {-1, 0}, {-1, 0}, {-1, 0}, 1},
        // The segment only touches the bottom-left corner before leaving.
        {{-2, 0}, {0, -2}, {-1, -1}, {-1, 0}, 0.5}
    };
    for (const auto& c : cases) {
        const auto hit = RayCast(box, c.start, c.end);
        REQUIRE(hit);
        REQUIRE(hit->fraction == Approx(c.fraction));
        CheckVector(hit->point, c.point);
        CheckVector(hit->normal, c.normal);
    }
    REQUIRE_FALSE(RayCast(box, {-2, 1.01f}, {2, 1.01f}));
    REQUIRE_FALSE(RayCast(box, {-2, 0}, {-1.01f, 0}));
    REQUIRE_FALSE(RayCast(box, {-3, 0}, {-4, 0}));
    REQUIRE_FALSE(RayCast(box, {-2, -0.1f}, {-0.1f, -2}));
}

TEST_CASE("Triangle ray queries are not restricted to box geometry", "[queries]") {
    Polygon triangle({{0, 0}, {4, 0}, {0, 3}});
    REQUIRE(ContainsPoint(triangle, {1, 1}));
    REQUIRE_FALSE(ContainsPoint(triangle, {3, 2}));
    const auto hit = RayCast(triangle, {4, 1}, {0, 1});
    REQUIRE(hit);
    REQUIRE(hit->fraction == Approx(1.0 / 3.0));
    CheckVector(hit->point, {8.0f / 3.0f, 1});
    CheckVector(hit->normal, {0.6f, 0.8f});
}

TEST_CASE("Analytic entries remain correct across translated rotated directions", "[queries]") {
    Circle circle(2);
    Polygon box = Polygon::MakeBox(2, 4);
    const Vector2 position{13, -8};
    for (int i = 0; i < 32; ++i) {
        const float angle = static_cast<float>(i * 0.19);
        const auto circleHit = RayCast(circle,
            TransformPoint({-5, 1}, position, angle),
            TransformPoint({5, 1}, position, angle), position, angle);
        REQUIRE(circleHit);
        REQUIRE(circleHit->fraction == Approx((5 - std::sqrt(3.0)) / 10).margin(1e-6));
        CheckVector(circleHit->point, TransformPoint({-std::sqrt(3.0f), 1}, position, angle));
        CheckVector(circleHit->normal, TransformPoint({-std::sqrt(3.0f) / 2, 0.5f}, {}, angle));
        const auto boxHit = RayCast(box,
            TransformPoint({-3, 0.5f}, position, angle),
            TransformPoint({3, 0.5f}, position, angle), position, angle);
        REQUIRE(boxHit);
        REQUIRE(boxHit->fraction == Approx(1.0 / 3).margin(1e-6));
        CheckVector(boxHit->point, TransformPoint({-1, 0.5f}, position, angle));
        CheckVector(boxHit->normal, TransformPoint({-1, 0}, {}, angle));
    }
}

TEST_CASE("World hits are ordered deterministically and retain shared handles", "[queries]") {
    World world;
    Circle circle(1);
    auto first = Body(circle);
    auto second = Body(circle);
    auto far = Body(circle, {4, 0});
    world.addBody(far); world.addBody(second); world.addBody(first);
    auto hits = RayCastAll(world, {-3, 0}, {6, 0});
    REQUIRE(hits.size() == 3);
    REQUIRE(hits[0].body == first);
    REQUIRE(hits[1].body == second);
    REQUIRE(hits[2].body == far);
    REQUIRE(RayCastNearest(world, {-3, 0}, {6, 0})->body == first);
    auto stationary = RayCastAll(world, {}, {});
    REQUIRE(stationary.size() == 2);
    REQUIRE(stationary[0].body == first);
    REQUIRE(stationary[1].body == second);
    REQUIRE(stationary[0].hit.fraction == 0);
    CheckVector(stationary[0].hit.normal, {});
    auto points = QueryPoint(world, {});
    REQUIRE(points == std::vector<RigidBodyPtr>{first, second});
    std::weak_ptr<RigidBody> retained = first;
    world.clearBodies();
    first.reset(); second.reset(); far.reset();
    REQUIRE_FALSE(retained.expired());
    REQUIRE(hits[0].body->GetId() < hits[1].body->GetId());
    points.clear(); hits.clear();
    // The independent stationary result also retains the same body.
    REQUIRE_FALSE(retained.expired());
    stationary.clear();
    REQUIRE(retained.expired());
    REQUIRE(QueryPoint(world, {}).empty());
    REQUIRE(RayCastAll(world, {}, {1, 0}).empty());
    REQUIRE_FALSE(RayCastNearest(world, {}, {1, 0}));
}

TEST_CASE("World query filters require mutual category-mask agreement", "[queries]") {
    World world;
    auto body = Body(Circle(1));
    body->SetCollisionCategoryBits(0x2);
    body->SetCollisionMaskBits(0x4);
    world.addBody(body);
    const QueryFilter accept{0x4, 0x2};
    REQUIRE(QueryPoint(world, {}, accept).size() == 1);
    REQUIRE(RayCastAll(world, {-2, 0}, {2, 0}, accept).size() == 1);
    REQUIRE(RayCastNearest(world, {-2, 0}, {2, 0}, accept));
    for (QueryFilter reject : {QueryFilter{0x1, 0x2}, QueryFilter{0x4, 0x1},
        QueryFilter{0, 0xFFFFFFFFu}, QueryFilter{0xFFFFFFFFu, 0}}) {
        REQUIRE(QueryPoint(world, {}, reject).empty());
        REQUIRE(RayCastAll(world, {-2, 0}, {2, 0}, reject).empty());
        REQUIRE_FALSE(RayCastNearest(world, {-2, 0}, {2, 0}, reject));
    }
    body->SetCollisionMaskBits(0);
    REQUIRE(QueryPoint(world, {}).empty());
}

TEST_CASE("World queries never truncate hits or merge nearby distinct fractions", "[queries]") {
    World world;
    for (int i = 0; i < 300; ++i) world.addBody(Body(Circle(1), {static_cast<float>(i), 0}));
    const auto hits = RayCastAll(world, {-2, 0}, {301, 0});
    REQUIRE(hits.size() == 300);
    for (std::size_t i = 1; i < hits.size(); ++i)
        REQUIRE(hits[i - 1].hit.fraction < hits[i].hit.fraction);
    World closeWorld;
    // Create the farther body's lower ID first: proximity must take priority.
    auto far = Body(Circle(1), {1.000001f, 0});
    auto near = Body(Circle(1), {1, 0});
    closeWorld.addBody(far); closeWorld.addBody(near);
    const auto closeHits = RayCastAll(closeWorld, {-100000, 0}, {100000, 0});
    REQUIRE(closeHits[0].body == near);
    REQUIRE(closeHits[0].hit.fraction < closeHits[1].hit.fraction);
    REQUIRE(RayCastNearest(closeWorld, {-100000, 0}, {100000, 0})->body == near);
    World overlaps;
    for (int i = 0; i < 300; ++i) overlaps.addBody(Body(Circle(1)));
    REQUIRE(QueryPoint(overlaps, {}).size() == 300);
}

TEST_CASE("Spatial queries reject non-finite coordinates even with no candidates", "[queries]") {
    const float infinity = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    Circle circle(1);
    Polygon box = Polygon::MakeBox(2, 2);
    World empty;
    for (float invalid : {infinity, -infinity, nan}) {
        const Vector2 bad{invalid, 0};
        REQUIRE_THROWS_AS(ContainsPoint(circle, bad), std::invalid_argument);
        REQUIRE_THROWS_AS(ContainsPoint(box, {}, bad), std::invalid_argument);
        REQUIRE_THROWS_AS(ContainsPoint(circle, {}, {}, invalid), std::invalid_argument);
        REQUIRE_THROWS_AS(RayCast(circle, bad, {}), std::invalid_argument);
        REQUIRE_THROWS_AS(RayCast(box, {}, bad), std::invalid_argument);
        REQUIRE_THROWS_AS(RayCast(box, {}, {}, {}, invalid), std::invalid_argument);
        REQUIRE_THROWS_AS(QueryPoint(empty, bad), std::invalid_argument);
        REQUIRE_THROWS_AS(RayCastAll(empty, bad, {}), std::invalid_argument);
        REQUIRE_THROWS_AS(RayCastNearest(empty, {}, bad), std::invalid_argument);
        auto body = Body(circle);
        body->position = bad; // Legacy public mutation bypasses setter validation.
        REQUIRE_THROWS_AS(ContainsPoint(*body, {}), std::invalid_argument);
        REQUIRE_THROWS_AS(RayCast(*body, {}, {}), std::invalid_argument);
        body->position = {};
        body->orientation = invalid;
        empty.addBody(body);
        REQUIRE_THROWS_AS(QueryPoint(empty, {}), std::invalid_argument);
        REQUIRE_THROWS_AS(RayCastAll(empty, {}, {}), std::invalid_argument);
        empty.clearBodies();
    }
}

TEST_CASE("Extreme finite geometry avoids float overflow and quadratic cancellation", "[queries]") {
    const float maximum = std::numeric_limits<float>::max();
    Circle circle(1);
    REQUIRE_FALSE(ContainsPoint(circle, {maximum, maximum}));
    auto hit = RayCast(circle, {-maximum, 0}, {maximum, 0});
    REQUIRE(hit);
    REQUIRE(hit->fraction == Approx(0.5));
    CheckVector(hit->point, {-1, 0});
    CheckVector(hit->normal, {-1, 0});
    REQUIRE_FALSE(RayCast(circle, {-maximum, 0}, {-1.000001f, 0}));
    hit = RayCast(circle, {-maximum, 1}, {maximum, 1});
    REQUIRE(hit);
    CheckVector(hit->point, {0, 1});
    hit = RayCast(circle, {-maximum, -maximum}, {maximum, maximum});
    REQUIRE(hit);
    CheckVector(hit->point, {-std::sqrt(0.5f), -std::sqrt(0.5f)});
    REQUIRE(ContainsPoint(circle, {maximum, maximum}, {maximum, maximum}));
    Circle enormous(maximum * 0.5f);
    REQUIRE_FALSE(ContainsPoint(enormous, {maximum, 0}));
    hit = RayCast(enormous, {-maximum, 0}, {maximum, 0});
    REQUIRE(hit);
    REQUIRE(hit->fraction == Approx(0.25));
    REQUIRE(hit->point.x == -maximum * 0.5f);
    CheckVector(hit->normal, {-1, 0});
    Polygon box = Polygon::MakeBox(2, 2);
    REQUIRE_FALSE(ContainsPoint(box, {maximum, maximum}));
    hit = RayCast(box, {-maximum, -maximum}, {maximum, maximum});
    REQUIRE(hit);
    REQUIRE(hit->fraction == Approx(0.5));
    CheckVector(hit->point, {-1, -1});
    CheckVector(hit->normal, {0, -1});
    hit = RayCast(box, {-maximum, 0}, {maximum, 0});
    REQUIRE(hit);
    CheckVector(hit->point, {-1, 0});
    const float angle = 0.7f;
    hit = RayCast(box, {-maximum, 0}, {maximum, 0}, {}, angle);
    REQUIRE(hit);
    REQUIRE(hit->fraction == Approx(0.5));
    CheckVector(hit->point, {-1 / std::cos(angle), 0});
    CheckVector(hit->normal, {-std::cos(angle), -std::sin(angle)});
    REQUIRE_FALSE(RayCast(box, {-maximum, 0}, {-1.31f, 0}, {}, angle));
    Polygon enormousBox = Polygon::MakeBox(1e30f, 1e30f);
    hit = RayCast(enormousBox, {-maximum, 0}, {maximum, 0});
    REQUIRE(hit);
    REQUIRE(hit->point.x == Approx(-5e29).epsilon(1e-6));
    CheckVector(hit->normal, {-1, 0});
    Circle tiny(1e-30f);
    hit = RayCast(tiny, {-1e-20f, 0}, {1e-20f, 0});
    REQUIRE(hit);
    REQUIRE(hit->point.x == -tiny.GetRadius());
    CheckVector(hit->normal, {-1, 0});
}
