#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include <limits>

using namespace PhysicsEngine;
namespace {
RigidBodyPtr SliderBody(Vector2 p={}, bool fixed=false) {
    auto body=std::make_shared<RigidBody>(Circle(0.1f),Material{1,0},p,fixed);
    body->SetCollisionMaskBits(0); if (!fixed) body->SetMass(1); return body;
}
SimulationConfig SliderConfig(int iterations=20) {
    SimulationConfig c; c.solverIterations=iterations;
    c.enableLinearVelocityLimit=false; c.enableAngularVelocityLimit=false; return c;
}
double AngularMomentum(const RigidBody& a, const RigidBody& b) {
    return double(a.mass)*(double(a.position.x)*a.velocity.y-double(a.position.y)*a.velocity.x)
        +double(b.mass)*(double(b.position.x)*b.velocity.y-double(b.position.y)*b.velocity.x)
        +double(a.inertia)*a.angularVelocity+double(b.inertia)*b.angularVelocity;
}
}
TEST_CASE("Prismatic base permits analytical axial motion", "[prismatic]") {
    World world(SliderConfig()); auto a=SliderBody({},true), b=SliderBody({2,0});
    b->SetVelocity({3,0}); auto joint=std::make_shared<PrismaticJoint>(a,b);
    world.addBody(a); world.addBody(b); world.addJoint(joint);
    for (int i=0;i<100;++i) world.step(0.01f);
    REQUIRE(joint->getTranslation()==Catch::Approx(5).margin(2e-5));
    REQUIRE(joint->getTranslationSpeed()==Catch::Approx(3));
    REQUIRE(joint->getTransverseError()==0); REQUIRE(joint->getAngle()==0);
}
TEST_CASE("Prismatic base couples long levers and rotated axes", "[prismatic]") {
    World world(SliderConfig(1)); auto a=SliderBody({},true), b=SliderBody({0,2});
    a->SetOrientation(1.57079632679f); b->SetOrientation(0.3f);
    b->SetVelocity({4,3}); b->SetAngularVelocity(2);
    auto joint=std::make_shared<PrismaticJoint>(a,b,Vector2{1,0},Vector2{},Vector2{20,-10});
    world.addBody(a); world.addBody(b); world.addJoint(joint); world.step(0);
    REQUIRE(b->angularVelocity==Catch::Approx(0).margin(1e-6));
    REQUIRE(b->velocity.x==Catch::Approx(0).margin(1e-6));
    REQUIRE(b->velocity.y==Catch::Approx(3).margin(1e-6));
    REQUIRE(std::abs(joint->getAngle())<0.006);
}
TEST_CASE("Prismatic impulses preserve isolated pair momentum", "[prismatic]") {
    World world(SliderConfig(1)); auto a=SliderBody({-1,0}), b=SliderBody({2,0});
    a->SetMass(2); b->SetMass(5); a->SetVelocity({3,-1}); b->SetVelocity({-2,4});
    a->SetAngularVelocity(1.3f); b->SetAngularVelocity(-0.7f);
    auto joint=std::make_shared<PrismaticJoint>(a,b,Vector2{1,0},Vector2{0,1},Vector2{0,1});
    const Vector2 before=a->velocity*a->mass+b->velocity*b->mass;
    const double angular=AngularMomentum(*a,*b);
    world.addBody(a); world.addBody(b); world.addJoint(joint); world.step(0);
    const Vector2 after=a->velocity*a->mass+b->velocity*b->mass;
    REQUIRE(after.x==Catch::Approx(before.x).margin(2e-6));
    REQUIRE(after.y==Catch::Approx(before.y).margin(2e-6));
    REQUIRE(AngularMomentum(*a,*b)==Catch::Approx(angular).margin(1e-5));
    REQUIRE(a->angularVelocity==Catch::Approx(b->angularVelocity).margin(1e-6));
    const auto e=joint->getAxis(); const Vector2 dv=b->velocity-a->velocity;
    // The line follows A: transverse relative speed includes axis rotation.
    REQUIRE(-e.y*dv.x+e.x*dv.y==Catch::Approx(3*a->angularVelocity).margin(2e-6));
}
TEST_CASE("Prismatic base controls transverse and angular error under loads", "[prismatic]") {
    World world(SliderConfig(30)); auto a=SliderBody({},true), b=SliderBody({1,1});
    a->SetOrientation(0.78539816339f); b->SetOrientation(0.2f);
    auto joint=std::make_shared<PrismaticJoint>(a,b,Vector2{1,0},Vector2{},Vector2{0.2f,0.1f});
    world.addBody(a); world.addBody(b); world.addJoint(joint);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0,-9.81f}));
    b->SetOrientation(0.6f);
    for (int i=0;i<500;++i) { world.step(1.f/120); REQUIRE(std::abs(joint->getTransverseError())<0.006); REQUIRE(std::abs(joint->getAngle())<0.006); }
}
TEST_CASE("Prismatic reference angle follows the principal model", "[prismatic]") {
    auto a=SliderBody(), b=SliderBody(); a->SetOrientation(3); b->SetOrientation(-3);
    PrismaticJoint joint(a,b); REQUIRE(joint.getAngle()==0);
    REQUIRE(joint.getReferenceAngle()==Catch::Approx(0.283185307179586));
    a->SetOrientation(3+0.2f); b->SetOrientation(-3+0.2f);
    REQUIRE(joint.getAngle()==Catch::Approx(0).margin(3e-7));
    b->SetOrientation(b->orientation+0.1f); REQUIRE(joint.getAngle()==Catch::Approx(0.1).margin(3e-7));
}
TEST_CASE("Prismatic construction validates axis anchors and endpoints", "[prismatic][validation]") {
    auto a=SliderBody(),b=SliderBody(); const float inf=std::numeric_limits<float>::infinity();
    REQUIRE_THROWS_AS(PrismaticJoint(a,b,{0,0}),std::invalid_argument);
    REQUIRE_THROWS_AS(PrismaticJoint(a,b,{inf,1}),std::invalid_argument);
    REQUIRE_THROWS_AS(PrismaticJoint(a,b,{1,0},{inf,0}),std::invalid_argument);
    REQUIRE_THROWS_AS(PrismaticJoint(a,a),std::invalid_argument);
    REQUIRE_THROWS_AS(PrismaticJoint(nullptr,b),std::invalid_argument);
    REQUIRE_THROWS_AS(PrismaticJoint(SliderBody({},true),SliderBody({},true)),std::invalid_argument);
    SECTION("large finite axis") { PrismaticJoint joint(a,b,{3e38f,3e38f}); REQUIRE(joint.getLocalAxis().x==Catch::Approx(std::sqrt(0.5))); }
    SECTION("tiny finite axis") { PrismaticJoint joint(a,b,{std::numeric_limits<float>::denorm_min(),0}); REQUIRE(joint.getLocalAxis().x==1); }
}
TEST_CASE("Prismatic base uses existing island ownership and sleep", "[prismatic][sleep]") {
    auto c=SliderConfig(); c.enableSleeping=true; c.sleepTimeThreshold=0.1f;
    World world(c); auto a=SliderBody(),b=SliderBody({1,0});
    auto joint=std::make_shared<PrismaticJoint>(a,b); world.addBody(a); world.addBody(b); world.addJoint(joint);
    for(int i=0;i<20;++i) world.step();
    REQUIRE_FALSE(a->IsAwake()); REQUIRE_FALSE(b->IsAwake());
    a->ApplyForce({1,0}); world.step(); REQUIRE(a->IsAwake()); REQUIRE(b->IsAwake());
    world.removeBody(a); REQUIRE(world.getJoints().empty()); REQUIRE(b->IsAwake());
    REQUIRE(joint->getBodyA()==a); // Joint retains endpoints after world removal.
}
