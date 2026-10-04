#include "catch_amalgamated.hpp"
#include "physics/core/collisions/continuous_collision.h"
#include "physics/core/shape.h"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace PhysicsEngine;
namespace {
void CheckSweepVector(Vector2 actual,Vector2 expected,double margin=2e-6) {
    REQUIRE(std::isfinite(actual.x));REQUIRE(std::isfinite(actual.y));
    REQUIRE(actual.x==Catch::Approx(expected.x).epsilon(0).margin(margin));
    REQUIRE(actual.y==Catch::Approx(expected.y).epsilon(0).margin(margin));
}
}

TEST_CASE("CCD circle classification avoids enormous quadratic cancellation", "[ccd-numerics]") {
    REQUIRE_FALSE(SweepCircleCircle({-1e10f,3},{2e10f,0},1,{}, {},1).hit);
    REQUIRE_FALSE(SweepCircleCircle({-1e10f,std::nextafter(2.0f,3.0f)},{2e10f,0},1,{}, {},1).hit);
    auto tangent=SweepCircleCircle({-1e10f,2},{2e10f,0},1,{}, {},1);
    REQUIRE(tangent.hit);REQUIRE(tangent.fraction==0.5f);
    CheckSweepVector(tangent.normal,{0,-1});CheckSweepVector(tangent.point,{0,1});
    const auto crossing=SweepCircleCircle({-1e10f,1},{2e10f,0},1,{}, {},1);
    REQUIRE(crossing.hit);
    CheckSweepVector(crossing.normal,{static_cast<float>(std::sqrt(3.0)/2),-0.5f});
    CheckSweepVector(crossing.point,{static_cast<float>(-std::sqrt(3.0)/2),0.5f});
    const auto below=SweepCircleCircle({-1e10f,std::nextafter(2.0f,1.0f)},{2e10f,0},1,{}, {},1);
    REQUIRE(below.hit);REQUIRE(below.normal.x>0);
}

TEST_CASE("CCD circle sweeps preserve reversed pair and moving relative geometry", "[ccd-numerics]") {
    const auto a=SweepCircleCircle({-10,1},{20,0},1,{}, {},1);
    const auto reversed=SweepCircleCircle({}, {},1,{-10,1},{20,0},1);
    REQUIRE(a.hit);REQUIRE(reversed.hit);REQUIRE(a.fraction==reversed.fraction);
    CheckSweepVector(reversed.normal,a.normal*-1);CheckSweepVector(reversed.point,a.point);
    const auto moving=SweepCircleCircle({-10,1},{23,-2},1,{}, {3,-2},1);
    REQUIRE(moving.hit);REQUIRE(moving.fraction==a.fraction);CheckSweepVector(moving.normal,a.normal);
    const double time=(10-std::sqrt(3.0))/20;
    CheckSweepVector(moving.point,{static_cast<float>(a.point.x+3*time),static_cast<float>(a.point.y-2*time)});
    REQUIRE_FALSE(SweepCircleCircle({-1e10f,3},{1e10f,0},1,{}, {-1e10f,0},1).hit);
}

TEST_CASE("CCD circle helpers keep geometry when relative inputs exceed float range", "[ccd-numerics]") {
    const float maximum=std::numeric_limits<float>::max();
    const auto opposing=SweepCircleCircle({-maximum,0},{maximum,0},1,{maximum,0},{-maximum,0},1);
    REQUIRE(opposing.hit);REQUIRE(opposing.fraction==1);
    CheckSweepVector(opposing.normal,{1,0});CheckSweepVector(opposing.point,{});
    const auto hit=SweepCircleCircle({-maximum,0},{maximum,0},1,{}, {},1);
    REQUIRE(hit.hit);REQUIRE(hit.fraction==1);
    CheckSweepVector(hit.normal,{1,0});CheckSweepVector(hit.point,{-1,0});
    // Relative endpoints retain a target's small offset after huge travel cancels.
    REQUIRE_FALSE(SweepCircleCircle({-maximum,0},{maximum,0},1,{10,0},{},1).hit);
    REQUIRE(SweepCircleCircle({3,0},{maximum,0},1,{maximum,0},{},1).hit);
    REQUIRE_FALSE(SweepCircleCircle({-3,0},{maximum,0},1,{maximum,0},{},1).hit);
    const auto endpointBeyondRange=SweepCircleCircle({maximum/2,0},{maximum,0},maximum/8,
        {maximum,0},{},maximum/8);
    REQUIRE(endpointBeyondRange.hit);REQUIRE(endpointBeyondRange.fraction==0.25f);
    REQUIRE(endpointBeyondRange.point.x/maximum==Catch::Approx(0.875).epsilon(0).margin(1e-7));
    const auto sum=SweepCircleCircle({-maximum,maximum},{maximum,0},maximum,{}, {},maximum);
    REQUIRE(sum.hit);REQUIRE(sum.fraction==0);REQUIRE(std::isfinite(sum.normal.x));
    for (float scale : {1e-30f,1e30f}) {
        const auto small=SweepCircleCircle({-5*scale,0},{10*scale,0},scale,{5*scale,0},{-10*scale,0},scale);
        REQUIRE(small.hit);REQUIRE(small.fraction==Catch::Approx(0.4).epsilon(0).margin(3e-8));
        CheckSweepVector(small.normal,{1,0});
        REQUIRE(small.point.x/scale==Catch::Approx(0).epsilon(0).margin(3e-7));
    }
}

TEST_CASE("CCD circle initial overlap and coincident fallback retain existing conventions", "[ccd-numerics]") {
    auto hit=SweepCircleCircle({}, {},2,{1,0},{},1);
    REQUIRE(hit.hit);REQUIRE(hit.fraction==0);CheckSweepVector(hit.normal,{1,0});CheckSweepVector(hit.point,{2,0});
    hit=SweepCircleCircle({1,2},{3,4},2,{1,2},{-3,-4},1);
    REQUIRE(hit.hit);REQUIRE(hit.fraction==0);CheckSweepVector(hit.normal,{1,0});CheckSweepVector(hit.point,{3,2});
    REQUIRE_FALSE(SweepCircleCircle({}, {},1,{3,0},{},1).hit);
    hit=SweepCircleCircle({}, {},1,{2,0},{},1);REQUIRE(hit.hit);REQUIRE(hit.fraction==0);
}

TEST_CASE("CCD circle helpers validate finite inputs and reject unrepresentable contact", "[ccd-numerics]") {
    const float nan=std::numeric_limits<float>::quiet_NaN(),inf=std::numeric_limits<float>::infinity();
    for (float radius : {0.0f,-1.0f,nan,inf}) {
        REQUIRE_THROWS_AS(SweepCircleCircle({}, {},radius,{}, {},1),std::invalid_argument);
        REQUIRE_THROWS_AS(SweepCircleCircle({}, {},1,{}, {},radius),std::invalid_argument);
    }
    for (Vector2 bad : {Vector2{nan,0},Vector2{0,inf}}) {
        REQUIRE_THROWS_AS(SweepCircleCircle(bad,{},1,{}, {},1),std::invalid_argument);
        REQUIRE_THROWS_AS(SweepCircleCircle({},bad,1,{}, {},1),std::invalid_argument);
        REQUIRE_THROWS_AS(SweepCircleCircle({}, {},1,bad,{},1),std::invalid_argument);
        REQUIRE_THROWS_AS(SweepCircleCircle({}, {},1,{},bad,1),std::invalid_argument);
    }
    const float maximum=std::numeric_limits<float>::max();
    REQUIRE_THROWS_AS(SweepCircleCircle({maximum,0},{},maximum,{maximum,0},{},1),std::overflow_error);
}

TEST_CASE("CCD polygon corner classification avoids enormous quadratic false hits", "[ccd-numerics]") {
    auto box=Polygon::MakeBox(2,2).getVertices();
    for (bool reverse : {false,true}) {
        if (reverse) std::reverse(box.begin(),box.end());
        REQUIRE_FALSE(SweepCirclePolygon({-1e10f,3},{2e10f,0},0.5f,box).hit);
        REQUIRE_FALSE(SweepCirclePolygon({-1e10f,std::nextafter(1.5f,2.0f)},{2e10f,0},0.5f,box).hit);
        const auto tangent=SweepCirclePolygon({-1e10f,1.5f},{2e10f,0},0.5f,box);
        REQUIRE(tangent.hit);REQUIRE(tangent.fraction==0.5f);
        CheckSweepVector(tangent.normal,{0,-1});CheckSweepVector(tangent.point,{-1,1});
        const auto crossing=SweepCirclePolygon({-1e10f,0},{2e10f,0},0.5f,box);
        REQUIRE(crossing.hit);CheckSweepVector(crossing.normal,{1,0});CheckSweepVector(crossing.point,{-1,0});
        // Expanded half-plane square corners would falsely intersect this path.
        REQUIRE_FALSE(SweepCirclePolygon({-2,-0.8f},{1.2f,-1.2f},0.5f,box).hit);
    }
}

TEST_CASE("CCD polygon faces corners and translated frames retain physical contacts", "[ccd-numerics]") {
    const auto box=Polygon::MakeBox(2,2).getVertices();
    const auto face=SweepCirclePolygon({-5,0},{10,0},0.5f,box);
    REQUIRE(face.hit);REQUIRE(face.fraction==Catch::Approx(0.35).epsilon(0).margin(3e-8));
    CheckSweepVector(face.normal,{1,0});CheckSweepVector(face.point,{-1,0});
    const auto corner=SweepCirclePolygon({-5,-5},{10,10},1,box);
    REQUIRE(corner.hit);
    REQUIRE(corner.fraction==Catch::Approx((4-std::sqrt(0.5))/10).epsilon(0).margin(3e-8));
    CheckSweepVector(corner.normal,{std::sqrt(0.5f),std::sqrt(0.5f)});CheckSweepVector(corner.point,{-1,-1});
    const auto moving=SweepCirclePolygon({-5,0},{13,-2},0.5f,box,{3,-2});
    REQUIRE(moving.hit);REQUIRE(moving.fraction==face.fraction);CheckSweepVector(moving.normal,face.normal);
    CheckSweepVector(moving.point,{0.05f,-0.7f});
    const auto relative=SweepCirclePolygon({-5,0},{10,0},0.5f,box,{3,0});
    REQUIRE(relative.hit);REQUIRE(relative.fraction==0.5f);CheckSweepVector(relative.point,{0.5f,0});
    std::vector<Vector2> triangle{{2,3},{6,3},{2,6}};
    const auto offset=SweepCirclePolygon({4,0},{0,5},0.5f,triangle);
    REQUIRE(offset.hit);REQUIRE(offset.fraction==0.5f);CheckSweepVector(offset.normal,{0,1});CheckSweepVector(offset.point,{4,3});
    triangle={{0,0},{4,0},{0,3}};
    const auto slanted=SweepCirclePolygon({5,1},{-5,0},0.5f,triangle);
    REQUIRE(slanted.hit);REQUIRE(slanted.fraction==Catch::Approx(0.3).epsilon(0).margin(3e-8));
    CheckSweepVector(slanted.normal,{-0.6f,-0.8f});CheckSweepVector(slanted.point,{3.2f,0.6f});
}

TEST_CASE("CCD polygon initial overlap retains closest point and inward inside normal", "[ccd-numerics]") {
    const auto box=Polygon::MakeBox(2,2).getVertices();
    auto hit=SweepCirclePolygon({}, {},0.5f,box);
    REQUIRE(hit.hit);REQUIRE(hit.fraction==0);CheckSweepVector(hit.point,{0,-1});CheckSweepVector(hit.normal,{0,1});
    hit=SweepCirclePolygon({1.25f,0},{-10,0},0.5f,box);
    REQUIRE(hit.hit);REQUIRE(hit.fraction==0);CheckSweepVector(hit.point,{1,0});CheckSweepVector(hit.normal,{-1,0});
    hit=SweepCirclePolygon({1.5f,0},{1,0},0.5f,box);
    REQUIRE(hit.hit);REQUIRE(hit.fraction==0);CheckSweepVector(hit.point,{1,0});CheckSweepVector(hit.normal,{-1,0});
    hit=SweepCirclePolygon({1,0},{},0.5f,box);
    REQUIRE(hit.hit);CheckSweepVector(hit.point,{1,0});CheckSweepVector(hit.normal,{1,0});
    REQUIRE_FALSE(SweepCirclePolygon({2,0},{},0.5f,box).hit);
    REQUIRE_FALSE(SweepCirclePolygon({2,0},{3,0},0.5f,box,{3,0}).hit);
}

TEST_CASE("CCD polygon geometry preserves tiny enormous and overflowing relative scales", "[ccd-numerics]") {
    for (float scale : {1e-30f,1e30f}) {
        auto box=Polygon::MakeBox(2*scale,2*scale).getVertices();
        for (bool reverse : {false,true}) {
            if (reverse) std::reverse(box.begin(),box.end());
            const auto face=SweepCirclePolygon({-5*scale,0},{10*scale,0},0.5f*scale,box);
            REQUIRE(face.hit);REQUIRE(face.fraction==Catch::Approx(0.35).epsilon(0).margin(3e-8));
            CheckSweepVector(face.normal,{1,0});REQUIRE(face.point.x/scale==Catch::Approx(-1).epsilon(0).margin(2e-7));
            REQUIRE_FALSE(SweepCirclePolygon({-5*scale,3*scale},{10*scale,0},0.5f*scale,box).hit);
            const auto corner=SweepCirclePolygon({-5*scale,-5*scale},{10*scale,10*scale},scale,box);
            REQUIRE(corner.hit);REQUIRE(corner.point.x/scale==Catch::Approx(-1).epsilon(0).margin(2e-7));
            CheckSweepVector(corner.normal,{std::sqrt(0.5f),std::sqrt(0.5f)});
        }
    }
    const float maximum=std::numeric_limits<float>::max();
    const auto box=Polygon::MakeBox(2,2).getVertices();
    auto hit=SweepCirclePolygon({-maximum,0},{maximum,0},0.5f,box);
    REQUIRE(hit.hit);REQUIRE(hit.fraction==1);CheckSweepVector(hit.normal,{1,0});CheckSweepVector(hit.point,{-1,0});
    hit=SweepCirclePolygon({-maximum,0},{maximum,0},0.5f,box,{-maximum,0});
    REQUIRE(hit.hit);REQUIRE(hit.fraction==0.5f);CheckSweepVector(hit.normal,{1,0});
    REQUIRE(std::isfinite(hit.point.x));REQUIRE(std::isfinite(hit.point.y));
    const std::vector<Vector2> shifted{{10,-1},{12,-1},{12,1},{10,1}};
    REQUIRE_FALSE(SweepCirclePolygon({-maximum,0},{maximum,0},0.5f,shifted).hit);
    const std::vector<Vector2> huge{{maximum*0.75f,-1},{maximum*0.875f,-1},{maximum*0.875f,1},{maximum*0.75f,1}};
    hit=SweepCirclePolygon({maximum/2,0},{maximum,0},maximum/16,huge);
    REQUIRE(hit.hit);REQUIRE(hit.point.x/maximum==Catch::Approx(0.75).epsilon(0).margin(1e-7));
}

TEST_CASE("CCD polygon helpers reject invalid shapes inputs and unrepresentable contacts", "[ccd-numerics]") {
    auto box=Polygon::MakeBox(2,2).getVertices();
    const float nan=std::numeric_limits<float>::quiet_NaN(),inf=std::numeric_limits<float>::infinity();
    for (float radius : {0.0f,-1.0f,nan,inf}) REQUIRE_THROWS_AS(SweepCirclePolygon({}, {},radius,box),std::invalid_argument);
    REQUIRE_THROWS_AS(SweepCirclePolygon({nan,0},{},1,box),std::invalid_argument);
    REQUIRE_THROWS_AS(SweepCirclePolygon({}, {0,inf},1,box),std::invalid_argument);
    REQUIRE_THROWS_AS(SweepCirclePolygon({}, {},1,box,{nan,0}),std::invalid_argument);
    REQUIRE_THROWS_AS(SweepCirclePolygon({}, {},1,{}),std::invalid_argument);
    REQUIRE_THROWS_AS(SweepCirclePolygon({}, {},1,{{0,0},{1,0},{0.5f,0.1f},{0,1}}),std::invalid_argument);
    box[0].x=nan;REQUIRE_THROWS_AS(SweepCirclePolygon({}, {},1,box),std::invalid_argument);
    const float maximum=std::numeric_limits<float>::max();
    box={{maximum/4,-1},{maximum/2,-1},{maximum/2,1},{maximum/4,1}};
    REQUIRE_THROWS_AS(SweepCirclePolygon({maximum,0},{maximum/2,0},maximum/8,box,{maximum,0}),std::overflow_error);
}
