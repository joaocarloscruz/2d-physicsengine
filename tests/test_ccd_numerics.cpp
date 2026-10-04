#include "catch_amalgamated.hpp"
#include "physics/core/collisions/continuous_collision.h"
#include "physics/core/shape.h"
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
