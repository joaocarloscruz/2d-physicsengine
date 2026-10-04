#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include "physics/core/collisions/narrow_phase/collision_circle_polygon.h"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace PhysicsEngine;
namespace {
void FiniteUnit(Vector2 n) {
    REQUIRE(std::isfinite(n.x)); REQUIRE(std::isfinite(n.y));
    REQUIRE(std::hypot(double(n.x),double(n.y)) == Catch::Approx(1).epsilon(2e-7));
}
void Reversed(RigidBody& circle, RigidBody& polygon, const CollisionManifold& hit) {
    const auto reverse=CollisionCirclePolygon(&polygon,&circle);
    REQUIRE(reverse.hasCollision==hit.hasCollision);
    if(hit.hasCollision) {
        REQUIRE(reverse.normal.x==-hit.normal.x); REQUIRE(reverse.normal.y==-hit.normal.y);
        REQUIRE(reverse.penetration==hit.penetration); REQUIRE(reverse.contactPoint==hit.contactPoint);
        REQUIRE(reverse.contacts[0].featureId==hit.contacts[0].featureId);
    }
}
}

TEST_CASE("Circle polygon corner axes remain meaningful at every supported scale", "[circle-polygon-numerics]") {
    for(float scale:{1e-30f,1e-25f,1e-4f,1.0f,1e20f,1e30f}) {
        CAPTURE(scale);
        RigidBody polygon(Polygon::MakeBox(2*scale,2*scale),Material{}, {},true);
        RigidBody circle(Circle(scale),Material{}, {1.8f*scale,1.8f*scale},true);
        REQUIRE_FALSE(CollisionCirclePolygon(&circle,&polygon).hasCollision);
        circle.SetPosition({1.5f*scale,1.5f*scale});
        auto hit=CollisionCirclePolygon(&circle,&polygon);
        REQUIRE(hit.hasCollision); REQUIRE(hit.contactCount==1); FiniteUnit(hit.normal);
        REQUIRE(double(hit.penetration)/scale == Catch::Approx(1-std::sqrt(0.5)).epsilon(1e-6));
        REQUIRE(hit.normal.x == Catch::Approx(-1/std::sqrt(2.0)).epsilon(2e-7));
        REQUIRE(hit.normal.y == Catch::Approx(hit.normal.x));
        REQUIRE(double(hit.contactPoint.x)/scale == Catch::Approx(1.5-1/std::sqrt(2.0)).epsilon(1e-6));
        Reversed(circle,polygon,hit);
        circle.SetPosition({1.5f*scale,0}); hit=CollisionCirclePolygon(&circle,&polygon);
        REQUIRE(hit.hasCollision); REQUIRE(hit.normal==Vector2(-1,0));
        REQUIRE(double(hit.penetration)/scale == Catch::Approx(0.5).epsilon(1e-6));
        Reversed(circle,polygon,hit);
        circle.SetPosition({2*scale,0}); REQUIRE_FALSE(CollisionCirclePolygon(&circle,&polygon).hasCollision);
        circle.SetPosition({}); hit=CollisionCirclePolygon(&circle,&polygon);
        REQUIRE(hit.hasCollision); REQUIRE(double(hit.penetration)/scale == Catch::Approx(2).epsilon(1e-6));
        FiniteUnit(hit.normal);
    }
}

TEST_CASE("Circle polygon SAT agrees with a local rectangle distance oracle", "[circle-polygon-numerics][differential]") {
    for(float scale:{1e-25f,1e-5f,1.0f,1e10f,1e25f}) for(bool reverse:{false,true}) for(float angle:{0.0f,0.37f}) {
        CAPTURE(scale,reverse,angle);
        const auto box=Polygon::MakeBox(2*scale,1.4f*scale);
        auto vertices=box.getVertices(); if(reverse) std::reverse(vertices.begin(),vertices.end());
        RigidBody polygon(Polygon(vertices),Material{}, {3*scale,-2*scale},true);
        polygon.SetOrientation(angle);
        RigidBody circle(Circle(0.3f*scale),Material{}, {},true);
        const double c=std::cos(double(angle)),s=std::sin(double(angle));
        for(int ix=-7;ix<=7;++ix) for(int iy=-5;iy<=5;++iy) {
            const double x=ix*0.22*scale,y=iy*0.22*scale;
            circle.SetPosition({float(polygon.position.x+c*x-s*y),float(polygon.position.y+s*x+c*y)});
            // Independent inverse transform + rectangle signed distance, not SAT.
            const double dx=double(circle.position.x)-polygon.position.x,dy=double(circle.position.y)-polygon.position.y;
            const double qx=std::abs(c*dx+s*dy)-scale;
            const double qy=std::abs(-s*dx+c*dy)-double(0.7f*scale);
            const double distance=std::hypot(std::max(qx,0.0),std::max(qy,0.0))+std::min(std::max(qx,qy),0.0);
            const double penetration=double(circle.shape->GetRadius())-distance;
            if(std::abs(penetration)<2e-6*scale) continue;
            const auto hit=CollisionCirclePolygon(&circle,&polygon);
            REQUIRE(hit.hasCollision==(penetration>0));
            if(hit.hasCollision) {
                REQUIRE(double(hit.penetration)/scale == Catch::Approx(penetration/scale).epsilon(2e-5).margin(2e-6));
                FiniteUnit(hit.normal);
            }
        }
    }
}

TEST_CASE("Circle polygon transforms retain offset local geometry and exact vertex centers", "[circle-polygon-numerics]") {
    RigidBody polygon(Polygon::MakeTriangle({10,10},{14,10},{10,14}),Material{}, {-10,-10},true);
    RigidBody circle(Circle(0.5f),Material{}, {0,0},true);
    auto hit=CollisionCirclePolygon(&circle,&polygon);
    REQUIRE(hit.hasCollision); REQUIRE(hit.penetration==0.5f); FiniteUnit(hit.normal); Reversed(circle,polygon,hit);
    circle.SetPosition({2.1f,2.1f}); hit=CollisionCirclePolygon(&circle,&polygon);
    REQUIRE(hit.hasCollision); FiniteUnit(hit.normal);
    REQUIRE(hit.normal.x == Catch::Approx(-1/std::sqrt(2.0)).epsilon(2e-7));
    REQUIRE(hit.normal.y == Catch::Approx(hit.normal.x));
    circle.SetPosition({2.5f,2.5f}); REQUIRE_FALSE(CollisionCirclePolygon(&circle,&polygon).hasCollision);
}

TEST_CASE("Circle polygon manifolds reject invalid inputs and nonrepresentable output", "[circle-polygon-numerics][validation]") {
    RigidBody circle(Circle(1),Material{}, {},true), other(Circle(1),Material{}, {},true);
    RigidBody polygon(Polygon::MakeBox(2,2),Material{}, {},true);
    REQUIRE_THROWS_AS(CollisionCirclePolygon(nullptr,&polygon),std::invalid_argument);
    REQUIRE_THROWS_AS(CollisionCirclePolygon(&circle,&other),std::invalid_argument);
    REQUIRE_THROWS_AS(CollisionCirclePolygon(&polygon,&polygon),std::invalid_argument);
    polygon.orientation=std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(CollisionCirclePolygon(&circle,&polygon),std::invalid_argument);
    polygon.orientation=0; circle.position.x=std::numeric_limits<float>::infinity();
    REQUIRE_THROWS_AS(CollisionCirclePolygon(&circle,&polygon),std::invalid_argument);
    const float largest=std::numeric_limits<float>::max();
    RigidBody hugeCircle(Circle(1e31f),Material{}, {largest,0},true);
    RigidBody hugeBox(Polygon::MakeBox(2e31f,4e31f),Material{}, {largest,0},true);
    REQUIRE_THROWS_AS(CollisionCirclePolygon(&hugeCircle,&hugeBox),std::overflow_error);
    REQUIRE(hugeCircle.position==Vector2(largest,0)); REQUIRE(hugeBox.position==Vector2(largest,0));
}
