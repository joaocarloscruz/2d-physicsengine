#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

using namespace PhysicsEngine;
namespace {
RigidBodyPtr QueryBody(const Shape& shape, Vector2 position = {}) {
    return std::make_shared<RigidBody>(shape, Material{}, position, true);
}
Vector2 RotateTranslate(Vector2 p, Vector2 position, float angle) {
    const double c = std::cos(static_cast<double>(angle)), s = std::sin(static_cast<double>(angle));
    return {static_cast<float>(position.x + c*p.x - s*p.y),
            static_cast<float>(position.y + s*p.x + c*p.y)};
}
}

TEST_CASE("Circle overlap includes contact containment and excludes separated disks", "[circle-queries]") {
    Circle circle(2);
    REQUIRE(OverlapsCircle(circle, {}, 0.5f));
    REQUIRE(OverlapsCircle(circle, {3, 0}, 1));
    REQUIRE_FALSE(OverlapsCircle(circle, {3.000001f, 0}, 1));
    REQUIRE(OverlapsCircle(circle, {3, 4}, 3));
    REQUIRE_FALSE(OverlapsCircle(circle, {3, 4}, 2.999f));
    auto body = QueryBody(circle, {10, -7});
    body->SetOrientation(0.7f);
    REQUIRE(OverlapsCircle(*body, {13, -7}, 1));
}

TEST_CASE("Polygon circle overlap uses finite edges and rounded corners", "[circle-queries]") {
    for (bool reverse : {false, true}) {
        std::vector<Vector2> vertices{{-1,-1},{1,-1},{1,1},{-1,1}};
        if (reverse) std::reverse(vertices.begin(), vertices.end());
        Polygon box(vertices);
        REQUIRE(OverlapsCircle(box, {}, 0.25f));
        REQUIRE(OverlapsCircle(box, {1.5f,0}, 0.5f));
        REQUIRE(OverlapsCircle(box, {1.3f,1.4f}, 0.5f));
        REQUIRE_FALSE(OverlapsCircle(box, {1.4f,1.4f}, 0.5f));
        REQUIRE_FALSE(OverlapsCircle(box, {1.50001f,0}, 0.5f));
        REQUIRE(OverlapsCircle(box, {4,0}, 3));
    }
}

TEST_CASE("Circle overlaps preserve transformed original offset polygon frames", "[circle-queries]") {
    Polygon triangle({{2,3},{6,3},{2,6}});
    const Vector2 position{7,-9};
    const float angle=0.7f;
    auto body=QueryBody(triangle,position);
    body->SetOrientation(angle);
    REQUIRE(OverlapsCircle(*body,RotateTranslate({4,2.75f},position,angle),0.3f));
    REQUIRE_FALSE(OverlapsCircle(*body,RotateTranslate({4,2.6f},position,angle),0.3f));
    REQUIRE_FALSE(OverlapsCircle(*body,position,0.25f));
}

TEST_CASE("Zero radius circle overlap exactly agrees with point queries", "[circle-queries]") {
    Circle circle(1);
    Polygon box=Polygon::MakeBox(2,2);
    for (const Shape* shape : {static_cast<const Shape*>(&circle),static_cast<const Shape*>(&box)})
        for (Vector2 point : {Vector2{},Vector2{1,0},Vector2{1,1},Vector2{2,2}})
            REQUIRE(OverlapsCircle(*shape,point,0)==ContainsPoint(*shape,point));
    World world;
    world.addBody(QueryBody(circle));
    world.addBody(QueryBody(box));
    REQUIRE(QueryCircle(world,{1,1},0)==QueryPoint(world,{1,1}));
}

TEST_CASE("Circle overlap arithmetic spans finite float scales", "[circle-queries]") {
    for (float scale : {1e-30f,1e30f}) {
        Circle circle(scale);
        REQUIRE(OverlapsCircle(circle,{2*scale,0},scale));
        REQUIRE_FALSE(OverlapsCircle(circle,{2.01f*scale,0},scale));
        Polygon box=Polygon::MakeBox(2*scale,2*scale);
        REQUIRE(OverlapsCircle(box,{1.4f*scale,1.4f*scale},scale));
        REQUIRE_FALSE(OverlapsCircle(box,{1.8f*scale,1.8f*scale},scale));
    }
    const float maximum=std::numeric_limits<float>::max();
    Circle enormous(maximum);
    REQUIRE(OverlapsCircle(enormous,{maximum,maximum},maximum));
    REQUIRE_FALSE(OverlapsCircle(enormous,{maximum,maximum},maximum/4));
    REQUIRE_FALSE(OverlapsCircle(Circle(1),{maximum,maximum},1));
}

TEST_CASE("World circle overlaps filter order and retain all matching bodies", "[circle-queries]") {
    World world;
    auto first=QueryBody(Circle(1)),second=QueryBody(Polygon::MakeBox(2,2));
    first->SetCollisionCategoryBits(2); first->SetCollisionMaskBits(4);
    second->SetCollisionCategoryBits(2); second->SetCollisionMaskBits(4);
    world.addBody(second);world.addBody(first);
    const QueryFilter accept{4,2};
    auto hits=QueryCircle(world,{1.5f,0},0.5f,accept);
    REQUIRE(hits==std::vector<RigidBodyPtr>{first,second});
    for (QueryFilter filter : {QueryFilter{1,2},QueryFilter{4,1},QueryFilter{0,~0u},QueryFilter{~0u,0}})
        REQUIRE(QueryCircle(world,{},1,filter).empty());
    std::weak_ptr<RigidBody> retained=first;
    world.clearBodies(); first.reset();second.reset();
    REQUIRE_FALSE(retained.expired());
    hits.clear(); REQUIRE(retained.expired());
    for (int i=0;i<300;++i) world.addBody(QueryBody(Circle(1)));
    REQUIRE(QueryCircle(world,{2,0},1).size()==300);
}

TEST_CASE("Circle overlap validates finite query geometry even in empty worlds", "[circle-queries]") {
    Circle circle(1);
    World empty;
    for (float invalid : {std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN(),-1.0f}) {
        REQUIRE_THROWS_AS(OverlapsCircle(circle,{},invalid),std::invalid_argument);
        REQUIRE_THROWS_AS(QueryCircle(empty,{},invalid),std::invalid_argument);
    }
    const float nan=std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(OverlapsCircle(circle,{nan,0},1),std::invalid_argument);
    REQUIRE_THROWS_AS(OverlapsCircle(circle,{},1,{nan,0}),std::invalid_argument);
    REQUIRE_THROWS_AS(OverlapsCircle(circle,{},1,{},nan),std::invalid_argument);
    REQUIRE_THROWS_AS(QueryCircle(empty,{nan,0},1),std::invalid_argument);
    auto body=QueryBody(circle);body->position={nan,0};empty.addBody(body);
    REQUIRE_THROWS_AS(QueryCircle(empty,{},1),std::invalid_argument);
    REQUIRE(QueryCircle(empty,{},1,QueryFilter{0,0}).empty());
}
