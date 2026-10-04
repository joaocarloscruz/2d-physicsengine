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
void CheckCircleVector(Vector2 actual, Vector2 expected, double margin=2e-6) {
    REQUIRE(std::isfinite(actual.x)); REQUIRE(std::isfinite(actual.y));
    REQUIRE(actual.x==Catch::Approx(expected.x).epsilon(0).margin(margin));
    REQUIRE(actual.y==Catch::Approx(expected.y).epsilon(0).margin(margin));
}
double SampleDistance(const Polygon& polygon, double x, double y) {
    const auto& vertices=polygon.getVertices();
    bool positive=false,negative=false;
    double minimum=std::numeric_limits<double>::infinity();
    for (std::size_t i=0;i<vertices.size();++i) {
        const auto a=vertices[i],b=vertices[(i+1)%vertices.size()];
        const double dx=static_cast<double>(b.x)-a.x,dy=static_cast<double>(b.y)-a.y;
        const double cross=dx*(y-a.y)-dy*(x-a.x);
        positive=positive||cross>0;negative=negative||cross<0;
        const double t=std::clamp(((x-a.x)*dx+(y-a.y)*dy)/(dx*dx+dy*dy),0.0,1.0);
        minimum=std::min(minimum,std::hypot(x-a.x-t*dx,y-a.y-t*dy));
    }
    return positive&&negative?minimum:0;
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

TEST_CASE("Circle target sweeps report center target point and normal analytically", "[circle-queries]") {
    Circle circle(1);
    auto hit=SweepCircle(circle,{-3,0},{3,0},0.5f);
    REQUIRE(hit); REQUIRE(hit->fraction==0.25);
    CheckCircleVector(hit->center,{-1.5f,0});
    CheckCircleVector(hit->contactPoint,{-1,0});
    CheckCircleVector(hit->normal,{-1,0});
    hit=SweepCircle(circle,{-3,1.5f},{3,1.5f},0.5f);
    REQUIRE(hit); REQUIRE(hit->fraction==0.5);
    CheckCircleVector(hit->center,{0,1.5f});
    CheckCircleVector(hit->contactPoint,{0,1});
    CheckCircleVector(hit->normal,{0,1});
    REQUIRE_FALSE(SweepCircle(circle,{-3,1.50001f},{3,1.50001f},0.5f));
    hit=SweepCircle(circle,{-3,0},{-1.5f,0},0.5f);
    REQUIRE(hit); REQUIRE(hit->fraction==1);
    REQUIRE_FALSE(SweepCircle(circle,{-3,0},{-1.50001f,0},0.5f));
    REQUIRE_FALSE(SweepCircle(circle,{-3,0},{-4,0},0.5f));
    hit=SweepCircle(circle,{-3,0},{3,0},0.5f,{1,0},1.2f);
    REQUIRE(hit); REQUIRE(hit->fraction==Catch::Approx(2.5/6).epsilon(0).margin(1e-15));
    CheckCircleVector(hit->center,{-0.5f,0});CheckCircleVector(hit->contactPoint,{0,0});
}

TEST_CASE("Polygon sweeps use finite offset faces and circular corners in either winding", "[circle-queries]") {
    for (bool reverse : {false,true}) {
        std::vector<Vector2> vertices{{-1,-1},{1,-1},{1,1},{-1,1}};
        if (reverse) std::reverse(vertices.begin(),vertices.end());
        Polygon box(vertices);
        auto hit=SweepCircle(box,{-3,0},{3,0},0.5f);
        REQUIRE(hit); REQUIRE(hit->fraction==0.25);
        CheckCircleVector(hit->center,{-1.5f,0});CheckCircleVector(hit->contactPoint,{-1,0});
        CheckCircleVector(hit->normal,{-1,0});
        hit=SweepCircle(box,{-3,-3},{0,0},0.5f);
        REQUIRE(hit);
        const double component=std::sqrt(0.5);
        REQUIRE(hit->fraction==Catch::Approx((2-0.5*component)/3).epsilon(0).margin(1e-15));
        CheckCircleVector(hit->center,{static_cast<float>(-1-0.5*component),static_cast<float>(-1-0.5*component)});
        CheckCircleVector(hit->contactPoint,{-1,-1});
        CheckCircleVector(hit->normal,{static_cast<float>(-component),static_cast<float>(-component)});
        hit=SweepCircle(box,{-3,1.5f},{3,1.5f},0.5f);
        REQUIRE(hit); REQUIRE(hit->fraction==Catch::Approx(1.0/3).epsilon(0).margin(1e-15));
        CheckCircleVector(hit->center,{-1,1.5f});CheckCircleVector(hit->contactPoint,{-1,1});
        CheckCircleVector(hit->normal,{0,1});
        REQUIRE_FALSE(SweepCircle(box,{-3,1.50001f},{3,1.50001f},0.5f));
        // x+y=-2.8 traverses the expanded half-plane square corner, but its
        // distance from (-1,-1) is .8/sqrt(2)>.5: the rounded target misses.
        REQUIRE_FALSE(SweepCircle(box,{-2,-0.8f},{-0.8f,-2},0.5f));
        REQUIRE_FALSE(SweepCircle(box,{-3,0},{-1.50001f,0},0.5f));
        hit=SweepCircle(box,{-3,0},{-1.5f,0},0.5f);
        REQUIRE(hit);REQUIRE(hit->fraction==1);
    }
}

TEST_CASE("Circle sweep initial overlap touching and stationary conventions are explicit", "[circle-queries]") {
    Circle circle(1);Polygon box=Polygon::MakeBox(2,2);
    for (const Shape* shape : {static_cast<const Shape*>(&circle),static_cast<const Shape*>(&box)})
        for (Vector2 start : {Vector2{},Vector2{1.5f,0}})
            for (Vector2 end : {start,Vector2{3,0},Vector2{-3,0}}) {
                const auto hit=SweepCircle(*shape,start,end,0.5f);
                REQUIRE(hit); REQUIRE(hit->fraction==0);
                CheckCircleVector(hit->center,start);CheckCircleVector(hit->contactPoint,start);
                CheckCircleVector(hit->normal,{});
            }
    REQUIRE_FALSE(SweepCircle(circle,{3,0},{3,0},0.5f));
    REQUIRE_FALSE(SweepCircle(box,{3,0},{3,0},0.5f));
}

TEST_CASE("Circle sweeps preserve rotated translated offset polygon frames", "[circle-queries]") {
    Polygon triangle({{2,3},{6,3},{2,6}});
    const Vector2 position{7,-9};
    for (float angle : {0.0f,0.7f,2.1f}) {
        auto body=QueryBody(triangle,position);body->SetOrientation(angle);
        const auto hit=SweepCircle(*body,RotateTranslate({4,0},position,angle),
            RotateTranslate({4,5},position,angle),0.5f);
        REQUIRE(hit); REQUIRE(hit->fraction==Catch::Approx(0.5).epsilon(0).margin(3e-7));
        CheckCircleVector(hit->center,RotateTranslate({4,2.5f},position,angle));
        CheckCircleVector(hit->contactPoint,RotateTranslate({4,3},position,angle));
        CheckCircleVector(hit->normal,RotateTranslate({0,-1},{},angle));
    }
    Polygon slanted({{0,0},{4,0},{0,3}});
    const auto hit=SweepCircle(slanted,{5,1},{0,1},0.5f);
    REQUIRE(hit);REQUIRE(hit->fraction==Catch::Approx(0.3).epsilon(0).margin(1e-15));
    CheckCircleVector(hit->center,{3.5f,1});CheckCircleVector(hit->contactPoint,{3.2f,0.6f});
    CheckCircleVector(hit->normal,{0.6f,0.8f});
}

TEST_CASE("Zero radius circle sweeps exactly reproduce ray results", "[circle-queries]") {
    Circle circle(1);Polygon box=Polygon::MakeBox(2,2);
    for (const Shape* shape : {static_cast<const Shape*>(&circle),static_cast<const Shape*>(&box)})
        for (Vector2 start : {Vector2{},Vector2{-3,0},Vector2{-3,-3},Vector2{-3,1},Vector2{3,3}})
            for (Vector2 end : {start,Vector2{3,0},Vector2{},Vector2{3,3}}) {
                const auto ray=RayCast(*shape,start,end);
                const auto disk=SweepCircle(*shape,start,end,0);
                REQUIRE(static_cast<bool>(ray)==static_cast<bool>(disk));
                if (!ray) continue;
                REQUIRE(disk->fraction==ray->fraction);
                REQUIRE(disk->center.x==ray->point.x);REQUIRE(disk->center.y==ray->point.y);
                REQUIRE(disk->contactPoint.x==ray->point.x);REQUIRE(disk->contactPoint.y==ray->point.y);
                REQUIRE(disk->normal.x==ray->normal.x);REQUIRE(disk->normal.y==ray->normal.y);
            }
}

TEST_CASE("Circle sweeps agree with independent dense polygon distance samples", "[circle-queries]") {
    const Polygon box=Polygon::MakeBox(2,2),triangle({{-1,-1},{2,-1},{0,2}});
    constexpr double radius=0.3;
    for (const Polygon* polygon : {&box,&triangle})
        for (int i=0;i<120;++i) {
            const Vector2 start{static_cast<float>(3.3*std::cos(i*1.13)),static_cast<float>(3.1*std::sin(i*1.13))};
            const Vector2 end{static_cast<float>(3*std::cos(i*0.73+1)),static_cast<float>(3.2*std::sin(i*0.73+1))};
            const auto hit=SweepCircle(*polygon,start,end,static_cast<float>(radius));
            double minimum=std::numeric_limits<double>::infinity(),firstInside=2;
            for (int j=0;j<=2000;++j) {
                const double fraction=j/2000.0;
                const double distance=SampleDistance(*polygon,start.x+fraction*(static_cast<double>(end.x)-start.x),
                    start.y+fraction*(static_cast<double>(end.y)-start.y));
                minimum=std::min(minimum,distance);
                if (distance<radius-1e-5&&firstInside==2) firstInside=fraction;
            }
            if (minimum<radius-0.005) REQUIRE(hit);
            if (minimum>radius+0.005) REQUIRE_FALSE(hit);
            if (hit) {
                REQUIRE(hit->fraction>=0);REQUIRE(hit->fraction<=1);
                if (firstInside<2) REQUIRE(hit->fraction<=firstInside);
                REQUIRE(SampleDistance(*polygon,hit->center.x,hit->center.y)
                    ==Catch::Approx(radius).epsilon(0).margin(2e-6));
                REQUIRE(SampleDistance(*polygon,hit->contactPoint.x,hit->contactPoint.y)<2e-6);
                REQUIRE(std::hypot(static_cast<double>(hit->center.x)-hit->contactPoint.x,
                    static_cast<double>(hit->center.y)-hit->contactPoint.y)
                    ==Catch::Approx(radius).epsilon(0).margin(2e-6));
            }
        }
}

TEST_CASE("Circle sweep finite float scales and huge endpoints retain local contact geometry", "[circle-queries]") {
    for (float scale : {1e-30f,1e30f}) {
        Circle circle(scale);Polygon box=Polygon::MakeBox(2*scale,2*scale);
        for (const Shape* shape : {static_cast<const Shape*>(&circle),static_cast<const Shape*>(&box)}) {
            const auto hit=SweepCircle(*shape,{-3*scale,0},{3*scale,0},0.5f*scale);
            REQUIRE(hit); REQUIRE(hit->fraction==Catch::Approx(0.25).epsilon(0).margin(2e-8));
            REQUIRE(hit->center.x/scale==Catch::Approx(-1.5).epsilon(0).margin(2e-7));
            REQUIRE(hit->contactPoint.x/scale==Catch::Approx(-1).epsilon(0).margin(2e-7));
        }
    }
    const float maximum=std::numeric_limits<float>::max();
    Circle circle(1);Polygon box=Polygon::MakeBox(2,2);
    for (const Shape* shape : {static_cast<const Shape*>(&circle),static_cast<const Shape*>(&box)}) {
        auto hit=SweepCircle(*shape,{-maximum,0},{maximum,0},0.5f);
        REQUIRE(hit);CheckCircleVector(hit->center,{-1.5f,0});CheckCircleVector(hit->contactPoint,{-1,0});
        REQUIRE_FALSE(SweepCircle(*shape,{-maximum,0},{-1.50001f,0},0.5f));
        hit=SweepCircle(*shape,{-maximum,-maximum},{maximum,maximum},0.5f);
        REQUIRE(hit);
        const double component=std::sqrt(0.5);
        const float target=shape==&circle?static_cast<float>(-component):-1.0f;
        CheckCircleVector(hit->contactPoint,{target,target});
    }
    Polygon outOfRange({{maximum/2,-1},{maximum,-1},{maximum,1},{maximum/2,1}});
    REQUIRE_THROWS_AS(SweepCircle(outOfRange,{0,0},{maximum,0},maximum,{maximum,0}),std::overflow_error);
}

TEST_CASE("World circle sweeps filter order retain ownership and preserve zero radius parity", "[circle-queries]") {
    World world;
    auto first=QueryBody(Circle(1)),second=QueryBody(Circle(1)),far=QueryBody(Circle(1),{4,0});
    for (const auto& body : {first,second,far}) {body->SetCollisionCategoryBits(2);body->SetCollisionMaskBits(4);}
    world.addBody(far);world.addBody(second);world.addBody(first);
    const QueryFilter accept{4,2};
    auto hits=SweepCircleAll(world,{-3,0},{6,0},0.5f,accept);
    REQUIRE(hits.size()==3);REQUIRE(hits[0].body==first);REQUIRE(hits[1].body==second);REQUIRE(hits[2].body==far);
    REQUIRE(SweepCircleNearest(world,{-3,0},{6,0},0.5f,accept)->body==first);
    const auto rays=RayCastAll(world,{-3,0},{6,0},accept);
    const auto zero=SweepCircleAll(world,{-3,0},{6,0},0,accept);
    REQUIRE(zero.size()==rays.size());
    for (std::size_t i=0;i<zero.size();++i) {REQUIRE(zero[i].body==rays[i].body);REQUIRE(zero[i].hit.fraction==rays[i].hit.fraction);}
    for (QueryFilter filter : {QueryFilter{1,2},QueryFilter{4,1},QueryFilter{0,~0u},QueryFilter{~0u,0}}) {
        REQUIRE(SweepCircleAll(world,{-3,0},{6,0},0.5f,filter).empty());
        REQUIRE_FALSE(SweepCircleNearest(world,{-3,0},{6,0},0.5f,filter));
    }
    auto stationary=SweepCircleAll(world,{1.5f,0},{1.5f,0},0.5f,accept);
    REQUIRE(stationary.size()==2);REQUIRE(stationary[0].hit.fraction==0);
    std::weak_ptr<RigidBody> retained=far;
    world.clearBodies();first.reset();second.reset();far.reset();
    REQUIRE_FALSE(retained.expired());REQUIRE(hits[2].body->GetPosition().x==4);
    hits.clear(); // rays/zero still own far until this test ends.
    REQUIRE_FALSE(retained.expired());
    REQUIRE(SweepCircleAll(world,{}, {1,0},1).empty());
    REQUIRE_FALSE(SweepCircleNearest(world,{}, {1,0},1));
}

TEST_CASE("World circle sweeps keep every hit and order nearby distinct fractions", "[circle-queries]") {
    World world;
    for (int i=0;i<300;++i) world.addBody(QueryBody(Circle(1),{static_cast<float>(i),0}));
    const auto hits=SweepCircleAll(world,{-2,0},{301,0},0.5f);
    REQUIRE(hits.size()==300);
    for (std::size_t i=1;i<hits.size();++i) REQUIRE(hits[i-1].hit.fraction<hits[i].hit.fraction);
    World close;
    auto far=QueryBody(Circle(1),{1.000001f,0}),near=QueryBody(Circle(1),{1,0});
    close.addBody(far);close.addBody(near);
    const auto nearby=SweepCircleAll(close,{-100000,0},{100000,0},0.5f);
    REQUIRE(nearby.size()==2);REQUIRE(nearby[0].body==near);
    REQUIRE(nearby[0].hit.fraction<nearby[1].hit.fraction);
    REQUIRE(SweepCircleNearest(close,{-100000,0},{100000,0},0.5f)->body==near);
}

TEST_CASE("Circle sweeps reject invalid numeric inputs before scanning", "[circle-queries]") {
    Circle circle(1);World empty;
    const float nan=std::numeric_limits<float>::quiet_NaN();
    for (float radius : {-1.0f,nan,std::numeric_limits<float>::infinity()}) {
        REQUIRE_THROWS_AS(SweepCircle(circle,{}, {1,0},radius),std::invalid_argument);
        REQUIRE_THROWS_AS(SweepCircleAll(empty,{}, {1,0},radius),std::invalid_argument);
        REQUIRE_THROWS_AS(SweepCircleNearest(empty,{}, {1,0},radius),std::invalid_argument);
    }
    REQUIRE_THROWS_AS(SweepCircle(circle,{nan,0}, {},1),std::invalid_argument);
    REQUIRE_THROWS_AS(SweepCircle(circle,{}, {nan,0},1),std::invalid_argument);
    REQUIRE_THROWS_AS(SweepCircle(circle,{}, {},1,{nan,0}),std::invalid_argument);
    REQUIRE_THROWS_AS(SweepCircle(circle,{}, {},1,{},nan),std::invalid_argument);
    REQUIRE_THROWS_AS(SweepCircleAll(empty,{nan,0}, {},1),std::invalid_argument);
    REQUIRE_THROWS_AS(SweepCircleNearest(empty,{}, {nan,0},1),std::invalid_argument);
    auto body=QueryBody(circle);body->orientation=nan;empty.addBody(body);
    REQUIRE_THROWS_AS(SweepCircleAll(empty,{}, {1,0},1),std::invalid_argument);
    REQUIRE_THROWS_AS(SweepCircleNearest(empty,{}, {1,0},1),std::invalid_argument);
    REQUIRE(SweepCircleAll(empty,{}, {1,0},1,QueryFilter{0,0}).empty());
}
