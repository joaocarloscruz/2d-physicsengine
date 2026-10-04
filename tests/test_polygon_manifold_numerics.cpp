#include "catch_amalgamated.hpp"
#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
using namespace PhysicsEngine;
namespace {
class InvalidOutline : public Polygon {
public:
    InvalidOutline() : Polygon({{-1,-1},{1,-1},{1,1},{-1,1}}) { vertices[1]=vertices[0]; }
    std::unique_ptr<Shape> Clone() const override { return std::make_unique<InvalidOutline>(*this); }
};
struct D { double x, y; };
// Independent interval SAT: translate to A's position and project both full
// outlines onto unnormalized edge perpendiculars. No reference/incident face
// selection or clipping is shared with the production manifold.
double OracleGap(const RigidBody& a, const RigidBody& b, double scale) {
    auto outline = [&](const RigidBody& body) {
        std::vector<D> result;
        const double c = std::cos(double(body.orientation)), s = std::sin(double(body.orientation));
        for (auto p : static_cast<const Polygon&>(*body.shape).getVertices())
            result.push_back({(double(body.position.x)-a.position.x+c*p.x-s*p.y)/scale,
                              (double(body.position.y)-a.position.y+s*p.x+c*p.y)/scale});
        return result;
    };
    const auto av = outline(a), bv = outline(b);
    double gap = -std::numeric_limits<double>::max();
    for (const auto* outlineVertices : {&av, &bv}) {
        const auto& v = *outlineVertices;
        for (std::size_t i=0; i<v.size(); ++i) {
            const D edge{v[(i+1)%v.size()].x-v[i].x, v[(i+1)%v.size()].y-v[i].y};
            auto interval = [&](const std::vector<D>& points) {
                double lo = std::numeric_limits<double>::max(), hi = -lo;
                for (auto p : points) { const double d = -edge.y*p.x+edge.x*p.y; lo=std::min(lo,d); hi=std::max(hi,d); }
                return std::array<double,2>{lo,hi};
            };
            const auto ai=interval(av), bi=interval(bv);
            gap=std::max(gap,std::max(bi[0]-ai[1],ai[0]-bi[1])/std::hypot(edge.x,edge.y));
        }
    }
    return gap;
}
void CheckReversal(RigidBody& a, RigidBody& b) {
    const auto forward=CollisionPolygonPolygon(&a,&b), reverse=CollisionPolygonPolygon(&b,&a);
    REQUIRE(forward.hasCollision == reverse.hasCollision);
    REQUIRE(forward.contactCount == reverse.contactCount);
    REQUIRE(forward.normal.x == -reverse.normal.x);
    REQUIRE(forward.normal.y == -reverse.normal.y);
    for (std::uint8_t i=0;i<forward.contactCount;++i) {
        REQUIRE(forward.contacts[i].featureId == reverse.contacts[i].featureId);
        REQUIRE(forward.contacts[i].position.x == reverse.contacts[i].position.x);
        REQUIRE(forward.contacts[i].position.y == reverse.contacts[i].position.y);
        REQUIRE(forward.contacts[i].penetration == reverse.contacts[i].penetration);
    }
}
}
TEST_CASE("Polygon clipping preserves analytical face patches across scale", "[polygon-numerics]") {
    std::array<std::uint32_t,2> features{};
    for (float scale : {1.f,1e-25f,1e25f,1e-35f,1e35f}) {
        auto shape=Polygon::MakeBox(2*scale,2*scale);
        RigidBody a(shape,{}, {},true), b(shape,{}, {1.5f*scale,0},true);
        const auto hit=CollisionPolygonPolygon(&a,&b);
        REQUIRE(hit.hasCollision); REQUIRE(hit.contactCount==2);
        REQUIRE(hit.normal.x==1); REQUIRE(hit.normal.y==0);
        const double expected=2*double(scale)-b.position.x;
        for (int i=0;i<2;++i) {
            REQUIRE(double(hit.contacts[i].penetration)/scale == Catch::Approx(expected/scale).epsilon(0).margin(1e-6));
            REQUIRE(double(hit.contacts[i].position.x)/scale == Catch::Approx(double(b.position.x)/scale-1).epsilon(0).margin(1e-6));
            REQUIRE(std::abs(double(hit.contacts[i].position.y)/scale)==Catch::Approx(1).epsilon(0).margin(1e-6));
            if (scale==1) features[i]=hit.contacts[i].featureId;
            else REQUIRE(hit.contacts[i].featureId==features[i]);
        }
        CheckReversal(a,b);
        b.SetPosition({2*scale,0}); REQUIRE_FALSE(CollisionPolygonPolygon(&a,&b).hasCollision);
        b.SetPosition({2.001f*scale,0}); REQUIRE_FALSE(CollisionPolygonPolygon(&a,&b).hasCollision);
        b.SetPosition({1.999f*scale,0}); REQUIRE(CollisionPolygonPolygon(&a,&b).hasCollision);
    }
}
TEST_CASE("Polygon SAT differential grid covers rotation winding corners and containment", "[polygon-numerics]") {
    int overlaps=0, misses=0;
    for (float scale : {1e-25f,1.f,1e25f}) for (bool clockwise : {false,true}) {
        std::vector<Vector2> vertices{{-scale,-scale},{scale,-scale},{scale,scale},{-scale,scale}};
        if (clockwise) std::reverse(vertices.begin(),vertices.end());
        Polygon shape(vertices);
        auto small=Polygon::MakeBox(.6f*scale,.8f*scale);
        RigidBody a(shape,{}, {3*scale,-2*scale},true), b(small,{}, {},true);
        a.SetOrientation(.27f);
        for (float angle : {-.6f,0.f,.43f}) for (int x=-5;x<=5;++x) for (int y=-5;y<=5;++y) {
            b.SetOrientation(angle); b.SetPosition({(3+.4f*x)*scale,(-2+.4f*y)*scale});
            const double gap=OracleGap(a,b,scale);
            if (std::abs(gap)<.002) continue; // independent oracle boundary margin
            const auto hit=CollisionPolygonPolygon(&a,&b);
            INFO("scale="<<scale<<" angle="<<angle<<" grid="<<x<<","<<y<<" gap="<<gap);
            REQUIRE(hit.hasCollision==(gap<0));
            if (gap<0) {
                ++overlaps; REQUIRE(hit.contactCount>=1); REQUIRE(hit.contactCount<=2);
                REQUIRE(std::hypot(hit.normal.x,hit.normal.y)==Catch::Approx(1).epsilon(0).margin(1e-7));
                CheckReversal(a,b);
            } else ++misses;
        }
    }
    REQUIRE(overlaps>300); REQUIRE(misses>300);
}
TEST_CASE("Polygon original local frames and huge common translations preserve edges", "[polygon-numerics]") {
    Polygon offset({{9,-1},{11,-1},{11,1},{9,1}});
    auto centered=Polygon::MakeBox(2,2);
    RigidBody a(offset,{}, {-10,0},true), b(centered,{}, {1.5f,0},true);
    REQUIRE(CollisionPolygonPolygon(&a,&b).penetration==.5f);
    a.SetOrientation(.4f); b.SetOrientation(.4f);
    a.SetPosition({float(-10*std::cos(.4)),float(-10*std::sin(.4))});
    b.SetPosition({float(1.5*std::cos(.4)),float(1.5*std::sin(.4))});
    REQUIRE(CollisionPolygonPolygon(&a,&b).penetration==Catch::Approx(.5).epsilon(0).margin(1e-6));
    RigidBody c(centered,{}, {1e30f,1e30f},true), d(centered,{}, {1e30f,1e30f},true);
    const auto hit=CollisionPolygonPolygon(&c,&d);
    REQUIRE(hit.hasCollision); REQUIRE(hit.contactCount==2); REQUIRE(hit.penetration==2);
    REQUIRE(std::isfinite(hit.contactPoint.x));
}
TEST_CASE("Thin polygon contact allowance scales with thickness", "[polygon-numerics]") {
    for (float aspect : {1.f,100.f,10000.f,1000000.f}) for (float scale : {1e-25f,1.f,1e25f}) {
        auto shape=Polygon::MakeBox(aspect*scale,scale);
        RigidBody a(shape,{}, {},true), b(shape,{}, {0,.75f*scale},true);
        const auto hit=CollisionPolygonPolygon(&a,&b);
        REQUIRE(hit.hasCollision); REQUIRE(hit.contactCount==2);
        for (int i=0;i<2;++i) {
            const double distance=double(hit.contacts[i].position.y)-.5*scale;
            REQUIRE(distance/scale==Catch::Approx(-.25).epsilon(0).margin(1e-6));
            REQUIRE(distance<=1e-5*double(scale));
        }
        b.SetPosition({0,1.01f*scale}); REQUIRE_FALSE(CollisionPolygonPolygon(&a,&b).hasCollision);
    }
}
TEST_CASE("Polygon manifold rejects invalid transforms types and unrepresentable output", "[polygon-numerics]") {
    auto shape=Polygon::MakeBox(2,2); Circle circle(1);
    RigidBody a(shape,{}, {},true), b(shape,{}, {},true), c(circle,{}, {},true);
    REQUIRE_THROWS_AS(CollisionPolygonPolygon(nullptr,&a),std::invalid_argument);
    REQUIRE_THROWS_AS(CollisionPolygonPolygon(&a,&c),std::invalid_argument);
    InvalidOutline invalid;
    RigidBody malformed(invalid,{}, {},true);
    REQUIRE_THROWS_AS(CollisionPolygonPolygon(&a,&malformed),std::invalid_argument);
    b.orientation=std::numeric_limits<float>::infinity();
    REQUIRE_THROWS_AS(CollisionPolygonPolygon(&a,&b),std::invalid_argument);
    b.orientation=0; b.position.x=std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(CollisionPolygonPolygon(&a,&b),std::invalid_argument);
    auto huge=Polygon::MakeBox(3e38f,3e38f);
    RigidBody h(huge,{}, {3e38f,3e38f},true), j(huge,{}, {3e38f,3e38f},true);
    REQUIRE_THROWS_AS(CollisionPolygonPolygon(&h,&j),std::overflow_error);
}
TEST_CASE("Rotated thin polygon contacts stay within their reference-plane allowance", "[polygon-numerics]") {
    for (float aspect : {1.f,100.f,10000.f}) for (float scale : {1e-25f,1.f,1e25f}) {
        auto shape=Polygon::MakeBox(aspect*scale,scale);
        RigidBody a(shape,{}, {},true), b(shape,{}, {},true);
        a.SetOrientation(.37f); b.SetOrientation(.37f);
        b.SetPosition({float(-.75*scale*std::sin(double(.37f))),float(.75*scale*std::cos(double(.37f)))});
        const auto hit=CollisionPolygonPolygon(&a,&b);
        REQUIRE(hit.hasCollision); REQUIRE(hit.contactCount==2);
        const bool referenceB=(hit.contacts[0].featureId & 0x40000000u)!=0;
        const auto& reference=referenceB?b:a;
        const std::size_t face=(hit.contacts[0].featureId>>16)&0x3fffu;
        const auto& v=static_cast<const Polygon&>(*reference.shape).getVertices();
        const double c=std::cos(double(reference.orientation)), s=std::sin(double(reference.orientation));
        const double ex=double(v[(face+1)%v.size()].x)-v[face].x;
        const double ey=double(v[(face+1)%v.size()].y)-v[face].y;
        const double length=std::hypot(ex,ey);
        const double nx=(c*ey+s*ex)/length, ny=(s*ey-c*ex)/length;
        const double px=reference.position.x+c*v[face].x-s*v[face].y;
        const double py=reference.position.y+s*v[face].x+c*v[face].y;
        for (int i=0;i<2;++i) {
            const auto p=hit.contacts[i].position;
            const double distance=nx*(double(p.x)-px)+ny*(double(p.y)-py);
            // Final public float contact positions can round away more than
            // the intentional allowance on a high-aspect rotated outline.
            const double conversionBound=4*std::numeric_limits<float>::epsilon()*aspect*scale;
            REQUIRE(distance<=1e-5*double(scale)+conversionBound);
            REQUIRE(double(hit.contacts[i].penetration)/scale==Catch::Approx(.25).epsilon(0).margin(1e-6));
        }
    }
}
