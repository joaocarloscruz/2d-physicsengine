#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include "../src/physics/core/collisions/normal_contact_block.h"
#include <cmath>
#include <limits>
using namespace PhysicsEngine;
namespace {
Material MaterialFor(float e=0) { return {1,e,0,0}; }
SimulationConfig Controls() {
    SimulationConfig c; c.solverIterations=1; c.positionCorrectionFactor=0; c.warmStartFactor=0;
    c.enableLinearVelocityLimit=false; c.enableAngularVelocityLimit=false; c.restitutionVelocityThreshold=0; return c;
}
ContactConstraint Constraint(RigidBody& a,RigidBody& b,ContactImpulseCache& cache,Vector2 p0={-.5f,0},Vector2 p1={.5f,0}) {
    ContactConstraint c; c.bodyA=&a; c.bodyB=&b; c.normal={0,1}; c.tangent={-1,0}; c.pointCount=2; c.cache=&cache;
    cache.contactCount=2;
    for(int i=0;i<2;++i) {
        const auto p=i?p1:p0; c.points[i].localAnchorA=p-a.position; c.points[i].localAnchorB=p-b.position;
        c.points[i].featureId=cache.contacts[i].featureId=i+1; c.points[i].impulse=cache.contacts[i].impulse;
    }
    return c;
}
double PointNormalVelocity(const RigidBody& a,const RigidBody& b,double x) {
    return double(b.velocity.y)+b.angularVelocity*(x-b.position.x)-a.velocity.y-a.angularVelocity*(x-a.position.x);
}
double Energy(const RigidBody& a,const RigidBody& b) {
    double e=0; for(const auto* p:{&a,&b}) if(!p->IsStatic()) e+=.5*p->GetMass()*(double(p->velocity.x)*p->velocity.x+double(p->velocity.y)*p->velocity.y)+.5*p->GetInertia()*double(p->angularVelocity)*p->angularVelocity;
    return e;
}
double Angular(const RigidBody& a,const RigidBody& b) {
    double l=0; for(const auto* p:{&a,&b}) if(!p->IsStatic()) l+=p->GetMass()*(double(p->position.x)*p->velocity.y-double(p->position.y)*p->velocity.x)+p->GetInertia()*p->angularVelocity;
    return l;
}
}
TEST_CASE("Symmetric two-point box impact stops or rebounds without rotation in one World iteration", "[contact-block][physical]") {
    const float restitution=GENERATE(0.f,.5f,1.f);
    World world(Controls()); auto floor=std::make_shared<RigidBody>(Polygon::MakeBox(10,1),MaterialFor(restitution),Vector2{0,-.5f},true);
    auto box=std::make_shared<RigidBody>(Polygon::MakeBox(1,1),MaterialFor(restitution),Vector2{0,.499f});
    box->SetMass(1); box->SetVelocity({0,-1}); world.addBody(floor); world.addBody(box); world.step(0);
    REQUIRE(box->velocity.y==Catch::Approx(restitution).epsilon(0).margin(2e-7)); REQUIRE(std::abs(box->angularVelocity)<2e-7);
    REQUIRE(box->velocity.x==0);
}
TEST_CASE("Spinning offcenter contact selects the physical single-active end", "[contact-block][physical]") {
    const float spin=GENERATE(-4.f,4.f);
    RigidBody a(Polygon::MakeBox(10,1),MaterialFor(),{0,-.5f},true),b(Polygon::MakeBox(1,1),MaterialFor(),{.1f,.499f});
    b.SetMass(1); b.SetVelocity({0,-1}); b.SetAngularVelocity(spin);
    ContactImpulseCache cache; auto c=Constraint(a,b,cache);
    const double left=.5+.1,right=.5-.1;
    const double lever=spin>0?-left:right;
    const double initial=-1+spin*lever,impulse=-initial/(1+6*lever*lever);
    REQUIRE(ContactSolverDetail::SolveNormalBlock(c));
    const int active=spin>0?0:1,inactive=1-active;
    REQUIRE(c.points[active].impulse.normal==Catch::Approx(impulse).epsilon(2e-7)); REQUIRE(c.points[inactive].impulse.normal==0);
    REQUIRE(b.velocity.y==Catch::Approx(-1+impulse).epsilon(0).margin(2e-7));
    REQUIRE(b.angularVelocity==Catch::Approx(spin+6*lever*impulse).epsilon(0).margin(5e-7));
    REQUIRE(std::abs(PointNormalVelocity(a,b,active?.5:-.5))<5e-7);
    REQUIRE(PointNormalVelocity(a,b,inactive?.5:-.5)>0);
}
TEST_CASE("Two dynamic contact endpoints conserve momentum angular momentum and elastic energy", "[contact-block][physical]") {
    const float e=GENERATE(0.f,1.f);
    World world(Controls()); auto a=std::make_shared<RigidBody>(Polygon::MakeBox(1,1),MaterialFor(e),Vector2{0,-.499f});
    auto b=std::make_shared<RigidBody>(Polygon::MakeBox(1,1),MaterialFor(e),Vector2{0,.499f});
    a->SetMass(2); b->SetMass(3); a->SetVelocity({.3f,1}); b->SetVelocity({-.2f,-2});
    const double initialEnergy=Energy(*a,*b),initialAngular=Angular(*a,*b),momentum=2*1+3*(-2);
    world.addBody(a); world.addBody(b); world.step(0);
    const double common=momentum/5;
    REQUIRE(a->velocity.y==Catch::Approx(common-3./5*e*3).epsilon(0).margin(5e-7));
    REQUIRE(b->velocity.y==Catch::Approx(common+2./5*e*3).epsilon(0).margin(5e-7));
    REQUIRE(2*double(a->velocity.y)+3*double(b->velocity.y)==Catch::Approx(momentum).epsilon(0).margin(1e-6));
    REQUIRE(Angular(*a,*b)==Catch::Approx(initialAngular).epsilon(0).margin(1e-7));
    REQUIRE(std::abs(a->angularVelocity)<1e-7); REQUIRE(std::abs(b->angularVelocity)<1e-7);
    if(e==1) REQUIRE(Energy(*a,*b)==Catch::Approx(initialEnergy).epsilon(2e-7)); else REQUIRE(Energy(*a,*b)<initialEnergy);
}
TEST_CASE("Accumulated normal solve removes a symmetric warm impulse without splitting the patch", "[contact-block][warmstart]") {
    RigidBody a(Polygon::MakeBox(10,1),MaterialFor(),{0,-.5f},true),b(Polygon::MakeBox(1,1),MaterialFor(),{0,.499f});
    b.SetMass(1); b.SetVelocity({0,-.2f}); // Initial -1 plus already applied .4+.4.
    ContactImpulseCache cache; cache.contacts[0].impulse.normal=.4; cache.contacts[1].impulse.normal=.4;
    auto c=Constraint(a,b,cache); REQUIRE(ContactSolverDetail::SolveNormalBlock(c));
    REQUIRE(c.points[0].impulse.normal==Catch::Approx(.5).epsilon(0).margin(1e-8)); REQUIRE(c.points[1].impulse.normal==Catch::Approx(.5).epsilon(0).margin(1e-8));
    REQUIRE(std::abs(b.velocity.y)<1e-7); REQUIRE(std::abs(b.angularVelocity)<1e-7);
    REQUIRE(cache.contacts[0].impulse.normal==c.points[0].impulse.normal); REQUIRE(cache.contacts[1].impulse.normal==c.points[1].impulse.normal);
    b.SetVelocity({0,2}); REQUIRE(ContactSolverDetail::SolveNormalBlock(c));
    REQUIRE(c.points[0].impulse.normal==0); REQUIRE(c.points[1].impulse.normal==0); REQUIRE(b.velocity.y==Catch::Approx(1).epsilon(0).margin(1e-7));
}
TEST_CASE("Singular duplicate contact features use finite nonnegative accumulated impulses", "[contact-block][singular]") {
    RigidBody a(Polygon::MakeBox(10,1),MaterialFor(),{0,-.5f},true),b(Polygon::MakeBox(1,1),MaterialFor(),{0,.499f});
    b.SetMass(1); b.SetVelocity({0,-1}); ContactImpulseCache cache;
    auto c=Constraint(a,b,cache,{0,0},{0,0}); c.points[1].featureId=c.points[0].featureId;
    REQUIRE(ContactSolverDetail::SolveNormalBlock(c)); REQUIRE(b.velocity.y==0); REQUIRE(b.angularVelocity==0);
    REQUIRE(c.points[0].impulse.normal==1); REQUIRE(c.points[1].impulse.normal==0);
}
TEST_CASE("Two-point normal correction overflow preserves both endpoint states and both caches", "[contact-block][transaction]") {
    RigidBody a(Polygon::MakeBox(1,1),MaterialFor(1),{0,-.499f}),b(Polygon::MakeBox(1,1),MaterialFor(1),{0,.499f});
    a.SetMass(2); b.SetMass(1); a.SetVelocity({.3f,0}); b.SetVelocity({-.2f,-3e38f});
    ContactImpulseCache cache; cache.contacts[0].impulse.normal=.25; cache.contacts[1].impulse.normal=.75;
    auto c=Constraint(a,b,cache,{.25f,0},{.5f,0});
    // Elastic bias from the actual incident velocity. The closer single active
    // contact gives finite A updates but B's angular result exceeds float range.
    c.points[0].velocityBias=-double(b.velocity.y); c.points[1].velocityBias=-double(b.velocity.y);
    const auto av=a.velocity,bv=b.velocity; const auto aw=a.angularVelocity,bw=b.angularVelocity;
    REQUIRE_THROWS_AS(ContactSolverDetail::SolveNormalBlock(c),std::overflow_error);
    REQUIRE(a.velocity==av); REQUIRE(b.velocity==bv); REQUIRE(a.angularVelocity==aw); REQUIRE(b.angularVelocity==bw);
    REQUIRE(c.points[0].impulse.normal==.25); REQUIRE(c.points[1].impulse.normal==.75);
    REQUIRE(cache.contacts[0].impulse.normal==.25); REQUIRE(cache.contacts[1].impulse.normal==.75);
}
TEST_CASE("Two-point conditioning and admissibility retain tiny and huge physical masses", "[contact-block][scale]") {
    for(float mass:{1e-30f,1.f,1e30f}) {
        RigidBody a(Polygon::MakeBox(10,1),MaterialFor(),{0,-.5f},true),b(Polygon::MakeBox(1,1),MaterialFor(),{0,.499f});
        b.SetMass(mass); b.SetVelocity({0,-1}); ContactImpulseCache cache; auto c=Constraint(a,b,cache);
        REQUIRE(ContactSolverDetail::SolveNormalBlock(c)); REQUIRE(std::abs(b.velocity.y)<2e-7); REQUIRE(std::abs(b.angularVelocity)<2e-7);
        REQUIRE(c.points[0].impulse.normal==Catch::Approx(.5/double(b.inverseMass)).epsilon(2e-7));
        REQUIRE(c.points[1].impulse.normal==Catch::Approx(.5/double(b.inverseMass)).epsilon(2e-7));
    }
}
TEST_CASE("Offcenter unequal dynamic bodies retain physical impulse invariants for both active contacts", "[contact-block][physical]") {
    const float e=GENERATE(0.f,1.f);
    RigidBody a(Polygon::MakeBox(1,1),MaterialFor(e),{-.2f,-.499f}),b(Polygon::MakeBox(1,1),MaterialFor(e),{.3f,.499f});
    a.SetMass(2); b.SetMass(3); a.SetVelocity({.3f,1}); b.SetVelocity({-.2f,-2}); a.SetAngularVelocity(.4f); b.SetAngularVelocity(-.6f);
    const double left=PointNormalVelocity(a,b,-.5),right=PointNormalVelocity(a,b,.5),energy=Energy(a,b),angular=Angular(a,b);
    ContactImpulseCache cache; auto c=Constraint(a,b,cache); c.points[0].velocityBias=-e*left; c.points[1].velocityBias=-e*right;
    REQUIRE(ContactSolverDetail::SolveNormalBlock(c)); REQUIRE(c.points[0].impulse.normal>0); REQUIRE(c.points[1].impulse.normal>0);
    REQUIRE(PointNormalVelocity(a,b,-.5)==Catch::Approx(-e*left).epsilon(0).margin(1e-6));
    REQUIRE(PointNormalVelocity(a,b,.5)==Catch::Approx(-e*right).epsilon(0).margin(1e-6));
    REQUIRE(2*double(a.velocity.y)+3*double(b.velocity.y)==Catch::Approx(-4).epsilon(0).margin(1e-6));
    REQUIRE(Angular(a,b)==Catch::Approx(angular).epsilon(0).margin(1e-6));
    if(e==1) REQUIRE(Energy(a,b)==Catch::Approx(energy).epsilon(3e-7)); else REQUIRE(Energy(a,b)<energy);
}
TEST_CASE("Nearly coincident two-point levers avoid ill-conditioned inversion", "[contact-block][singular]") {
    RigidBody a(Polygon::MakeBox(10,1),MaterialFor(),{0,-.5f},true),b(Polygon::MakeBox(1,1),MaterialFor(),{0,.499f});
    b.SetMass(1); b.SetVelocity({0,-1}); b.SetAngularVelocity(1); ContactImpulseCache cache;
    auto c=Constraint(a,b,cache,{-1e-8f,0},{1e-8f,0}); REQUIRE(ContactSolverDetail::SolveNormalBlock(c));
    REQUIRE(c.points[0].impulse.normal>=0); REQUIRE(c.points[1].impulse.normal>=0);
    REQUIRE(std::isfinite(b.velocity.y)); REQUIRE(std::isfinite(b.angularVelocity));
    REQUIRE(PointNormalVelocity(a,b,-1e-8)>-2e-7); REQUIRE(PointNormalVelocity(a,b,1e-8)>-2e-7);
}
TEST_CASE("World two-point warm starting retains a symmetric one-iteration support impulse", "[contact-block][warmstart]") {
    const float factor=GENERATE(.8f,1.f); auto controls=Controls(); controls.warmStartFactor=factor;
    World world(controls); auto floor=std::make_shared<RigidBody>(Polygon::MakeBox(10,1),MaterialFor(),Vector2{0,-.5f},true);
    auto box=std::make_shared<RigidBody>(Polygon::MakeBox(1,1),MaterialFor(),Vector2{0,.499f});
    box->SetMass(1); world.addBody(floor); world.addBody(box);
    for(int i=0;i<3;++i) {
        box->SetVelocity({0,-1}); world.step(0);
        REQUIRE(std::abs(box->velocity.y)<2e-7); REQUIRE(std::abs(box->angularVelocity)<2e-7);
    }
}
TEST_CASE("Duplicate feature identifiers retain distinct one-to-one cached point impulses", "[contact-block][singular][warmstart]") {
    ContactImpulseCache cache; cache.contactCount=2;
    cache.contacts[0]={7,{.25,.1}}; cache.contacts[1]={7,{.75,-.2}};
    CollisionManifold manifold; manifold.hasCollision=true; manifold.contactCount=2;
    manifold.contacts[0].featureId=7; manifold.contacts[1].featureId=7;
    ContactSolverDetail::SynchronizeFeatureCache(manifold,cache);
    REQUIRE(cache.contacts[0].impulse.normal==.25); REQUIRE(cache.contacts[1].impulse.normal==.75);
    REQUIRE(cache.contacts[0].impulse.tangent==.1); REQUIRE(cache.contacts[1].impulse.tangent==-.2);
}
