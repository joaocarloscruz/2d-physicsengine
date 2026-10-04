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
    REQUIRE(joint->getTranslation()==Catch::Approx(5).epsilon(0).margin(2e-5));
    REQUIRE(joint->getTranslationSpeed()==Catch::Approx(3));
    REQUIRE(joint->getTransverseError()==0); REQUIRE(joint->getAngle()==0);
}
TEST_CASE("Prismatic base couples long levers and rotated axes", "[prismatic]") {
    World world(SliderConfig(1)); auto a=SliderBody({},true), b=SliderBody({0,2});
    a->SetOrientation(1.57079632679f); b->SetOrientation(0.3f);
    b->SetVelocity({4,3}); b->SetAngularVelocity(2);
    auto joint=std::make_shared<PrismaticJoint>(a,b,Vector2{1,0},Vector2{},Vector2{20,-10});
    world.addBody(a); world.addBody(b); world.addJoint(joint); world.step(0);
    REQUIRE(b->angularVelocity==Catch::Approx(0).epsilon(0).margin(1e-6));
    REQUIRE(b->velocity.x==Catch::Approx(0).epsilon(0).margin(1e-6));
    REQUIRE(b->velocity.y==Catch::Approx(3).epsilon(0).margin(1e-6));
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
    REQUIRE(after.x==Catch::Approx(before.x).epsilon(0).margin(2e-6));
    REQUIRE(after.y==Catch::Approx(before.y).epsilon(0).margin(2e-6));
    REQUIRE(AngularMomentum(*a,*b)==Catch::Approx(angular).epsilon(0).margin(1e-5));
    REQUIRE(a->angularVelocity==Catch::Approx(b->angularVelocity).epsilon(0).margin(1e-6));
    const auto e=joint->getAxis(); const Vector2 dv=b->velocity-a->velocity;
    // The line follows A: transverse relative speed includes axis rotation.
    REQUIRE(-e.y*dv.x+e.x*dv.y==Catch::Approx(3*a->angularVelocity).epsilon(0).margin(2e-6));
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
    REQUIRE(joint.getAngle()==Catch::Approx(0).epsilon(0).margin(3e-7));
    b->SetOrientation(b->orientation+0.1f); REQUIRE(joint.getAngle()==Catch::Approx(0.1).epsilon(0).margin(3e-7));
}
TEST_CASE("Prismatic construction validates axis anchors and endpoints", "[prismatic][validation]") {
    auto a=SliderBody(),b=SliderBody(); const float inf=std::numeric_limits<float>::infinity();
    REQUIRE_THROWS_AS(PrismaticJoint(a,b,{0,0}),std::invalid_argument);
    REQUIRE_THROWS_AS(PrismaticJoint(a,b,{inf,1}),std::invalid_argument);
    REQUIRE_THROWS_AS(PrismaticJoint(a,b,{std::numeric_limits<float>::quiet_NaN(),1}),std::invalid_argument);
    REQUIRE_THROWS_AS(PrismaticJoint(a,b,{1,0},{inf,0}),std::invalid_argument);
    REQUIRE_THROWS_AS(PrismaticJoint(a,b,{1,0},{},{0,inf}),std::invalid_argument);
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
TEST_CASE("Prismatic motor caps total impulse across solver iterations", "[prismatic][motor]") {
    for (int iterations : {1,10,40}) for (float sign : {-1.f,1.f}) {
        World world(SliderConfig(iterations)); auto a=SliderBody({},true),b=SliderBody(); b->SetMass(2);
        auto joint=std::make_shared<PrismaticJoint>(a,b,Vector2{1,0},Vector2{0,20},Vector2{0,20});
        joint->setMotor(true,sign*100,3); world.addBody(a); world.addBody(b); world.addJoint(joint);
        world.step(0.1f);
        REQUIRE(b->velocity.x==Catch::Approx(sign*0.15).epsilon(0).epsilon(0).margin(2e-8));
        REQUIRE(b->velocity.y==Catch::Approx(0).epsilon(0).epsilon(0).margin(1e-7));
        REQUIRE(b->angularVelocity==Catch::Approx(0).epsilon(0).epsilon(0).margin(1e-7));
        REQUIRE(std::abs(joint->getMotorForce())<=3);
        REQUIRE(joint->getMotorForce()==Catch::Approx(sign*3).epsilon(0).epsilon(0).margin(1e-12));
        world.step(0); REQUIRE(joint->getMotorForce()==0);
    }
}
TEST_CASE("Prismatic unsaturated motor solves coupled offcenter rows", "[prismatic][motor]") {
    for(int iterations : {1,30}) {
        World world(SliderConfig(iterations)); auto a=SliderBody({-1,0}),b=SliderBody({2,0});
        a->SetMass(2); b->SetMass(5);
        auto joint=std::make_shared<PrismaticJoint>(a,b,Vector2{1,0},Vector2{0,20},Vector2{0,20});
        joint->setMotor(true,2,1000); world.addBody(a); world.addBody(b); world.addJoint(joint);
        world.step(0.01f);
        REQUIRE(joint->getTranslationSpeed()==Catch::Approx(2).epsilon(0).epsilon(0).margin(5e-6));
        REQUIRE(a->angularVelocity==Catch::Approx(b->angularVelocity).epsilon(0).epsilon(0).margin(1e-6));
        REQUIRE(AngularMomentum(*a,*b)==Catch::Approx(0).epsilon(0).epsilon(0).margin(1e-5));
        REQUIRE(a->mass*a->velocity.x+b->mass*b->velocity.x==Catch::Approx(0).epsilon(0).epsilon(0).margin(2e-6));
        REQUIRE(a->mass*a->velocity.y+b->mass*b->velocity.y==Catch::Approx(0).epsilon(0).epsilon(0).margin(2e-6));
    }
}
TEST_CASE("Prismatic drive preserves unequal mass Newton momentum", "[prismatic][motor]") {
    World world(SliderConfig()); auto a=SliderBody(),b=SliderBody({2,0}); a->SetMass(1); b->SetMass(3);
    auto joint=std::make_shared<PrismaticJoint>(a,b); joint->setMotor(true,100,2);
    world.addBody(a); world.addBody(b); world.addJoint(joint); world.step(0.1f);
    REQUIRE(a->velocity.x==Catch::Approx(-0.2).epsilon(0).epsilon(0).margin(2e-8));
    REQUIRE(b->velocity.x==Catch::Approx(0.2/3).epsilon(0).epsilon(0).margin(1e-8));
    REQUIRE(joint->getTranslationSpeed()==Catch::Approx(0.2*4/3).epsilon(0).epsilon(0).margin(4e-8));
    REQUIRE(a->mass*a->velocity.x+b->mass*b->velocity.x==Catch::Approx(0).epsilon(0).epsilon(0).margin(3e-8));
}
TEST_CASE("Prismatic motor handles zero force braking and disabling", "[prismatic][motor]") {
    World world(SliderConfig()); auto a=SliderBody({},true),b=SliderBody(); b->SetVelocity({2,0});
    auto joint=std::make_shared<PrismaticJoint>(a,b); world.addBody(a); world.addBody(b); world.addJoint(joint);
    joint->setMotor(true,0,0); world.step(0.1f); REQUIRE(b->velocity.x==2); REQUIRE(joint->getMotorForce()==0);
    joint->setMotor(false,100,5); world.step(0.1f); REQUIRE(b->velocity.x==2);
    joint->setMotor(true,0,5); world.step(0.1f);
    REQUIRE(b->velocity.x==Catch::Approx(1.5).epsilon(0).epsilon(0).margin(1e-7));
    REQUIRE(joint->getMotorForce()==Catch::Approx(-5).epsilon(0).epsilon(0).margin(1e-12));
}
TEST_CASE("Prismatic motor displacement converges under timestep refinement", "[prismatic][motor]") {
    double errors[3]; int k=0;
    for(int steps : {10,20,40}) {
        World world(SliderConfig()); auto a=SliderBody({},true),b=SliderBody(); b->SetMass(2);
        auto joint=std::make_shared<PrismaticJoint>(a,b); joint->setMotor(true,100,4);
        world.addBody(a); world.addBody(b); world.addJoint(joint);
        for(int i=0;i<steps;++i) world.step(1.f/steps);
        REQUIRE(b->velocity.x==Catch::Approx(2).epsilon(0).epsilon(0).margin(2e-6));
        const double discrete=double(steps-1)/steps;
        REQUIRE(joint->getTranslation()==Catch::Approx(discrete).epsilon(0).epsilon(0).margin(2e-6));
        errors[k++]=std::abs(joint->getTranslation()-1);
    }
    REQUIRE(errors[1]<errors[0]*0.51); REQUIRE(errors[2]<errors[1]*0.51);
}
TEST_CASE("Prismatic travel limits stop outward motion and release inward", "[prismatic][limits]") {
    for(float sign : {-1.f,1.f}) {
        World world(SliderConfig(30)); auto a=SliderBody({},true),b=SliderBody({sign,0});
        auto joint=std::make_shared<PrismaticJoint>(a,b); joint->setLimits(true,-1,1);
        b->SetVelocity({sign*10,0}); world.addBody(a); world.addBody(b); world.addJoint(joint);
        world.step(0.01f);
        REQUIRE(joint->getTranslation()==Catch::Approx(sign).epsilon(0).epsilon(0).margin(1e-6));
        REQUIRE(b->velocity.x==Catch::Approx(0).epsilon(0).epsilon(0).margin(1e-6));
        b->SetVelocity({-sign,0}); world.step(0.1f);
        REQUIRE(joint->getTranslation()==Catch::Approx(sign*0.9).epsilon(0).epsilon(0).margin(1e-6));
        REQUIRE(b->velocity.x==Catch::Approx(-sign).epsilon(0).epsilon(0).margin(1e-6));
    }
}
TEST_CASE("Prismatic equal stops lock rotated offcenter anchors", "[prismatic][limits]") {
    World world(SliderConfig(30)); auto a=SliderBody({},true),b=SliderBody({0,2});
    a->SetOrientation(1.57079632679f);
    auto joint=std::make_shared<PrismaticJoint>(a,b,Vector2{1,0},Vector2{3,4},Vector2{2,3});
    joint->setLimits(true,0.5f,0.5f); joint->setMotor(true,10,4);
    b->SetVelocity({2,3}); b->SetAngularVelocity(1);
    world.addBody(a); world.addBody(b); world.addJoint(joint);
    for(int i=0;i<50;++i) world.step(0.01f);
    REQUIRE(joint->getTranslation()==Catch::Approx(0.5).epsilon(0).epsilon(0).margin(1e-6));
    REQUIRE(joint->getTransverseError()==Catch::Approx(0).epsilon(0).epsilon(0).margin(1e-6));
    REQUIRE(joint->getTranslationSpeed()==Catch::Approx(0).epsilon(0).epsilon(0).margin(1e-6));
    REQUIRE(joint->getAngle()==Catch::Approx(0).epsilon(0).epsilon(0).margin(1e-6));
    REQUIRE(std::abs(joint->getMotorForce())<=4);
}
TEST_CASE("Prismatic predictive stops prevent crossing with driven carriage", "[prismatic][limits]") {
    World world(SliderConfig()); auto a=SliderBody({},true),b=SliderBody();
    auto joint=std::make_shared<PrismaticJoint>(a,b); joint->setLimits(true,-0.4f,0.4f); joint->setMotor(true,5,100);
    world.addBody(a); world.addBody(b); world.addJoint(joint);
    for(int i=0;i<100;++i) {
        world.step(0.01f);
        REQUIRE(joint->getTranslation()<=double(joint->getUpperLimit())+1e-6);
        REQUIRE(joint->getTranslation()>=double(joint->getLowerLimit())-1e-6);
        REQUIRE(std::abs(joint->getMotorForce())<=100);
    }
    REQUIRE(joint->getTranslation()==Catch::Approx(0.4).epsilon(0).epsilon(0).margin(1e-6));
    joint->setMotor(true,-5,100);
    for(int i=0;i<100;++i) world.step(0.01f);
    REQUIRE(joint->getTranslation()==Catch::Approx(-0.4).epsilon(0).epsilon(0).margin(1e-6));
}
TEST_CASE("Prismatic motor wakes and holds active islands awake", "[prismatic][sleep]") {
    auto c=SliderConfig(); c.enableSleeping=true; c.sleepTimeThreshold=0.05f;
    World world(c); auto a=SliderBody(),b=SliderBody(); auto joint=std::make_shared<PrismaticJoint>(a,b);
    world.addBody(a); world.addBody(b); world.addJoint(joint);
    for(int i=0;i<20;++i) world.step(); REQUIRE_FALSE(a->IsAwake()); REQUIRE_FALSE(b->IsAwake());
    joint->setLimits(true,0,0); REQUIRE(a->IsAwake()); REQUIRE(b->IsAwake());
    joint->setMotor(true,1,2);
    for(int i=0;i<30;++i) world.step(); REQUIRE(a->IsAwake()); REQUIRE(b->IsAwake());
    joint->setMotor(false,1,2);
    for(int i=0;i<30;++i) world.step(); REQUIRE_FALSE(a->IsAwake()); REQUIRE_FALSE(b->IsAwake());
    joint->setMotor(true,0,2);
    for(int i=0;i<30;++i) world.step(); REQUIRE_FALSE(a->IsAwake()); REQUIRE_FALSE(b->IsAwake());
}
TEST_CASE("Prismatic settings reject invalid values atomically", "[prismatic][validation]") {
    auto a=SliderBody(),b=SliderBody(); PrismaticJoint joint(a,b);
    const float inf=std::numeric_limits<float>::infinity(), nan=std::numeric_limits<float>::quiet_NaN();
    joint.setMotor(true,2,3); joint.setLimits(true,-2,4);
    REQUIRE_THROWS_AS(joint.setMotor(false,nan,3),std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setMotor(false,0,inf),std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setMotor(false,0,-1),std::invalid_argument);
    REQUIRE(joint.isMotorEnabled()); REQUIRE(joint.getMotorSpeed()==2); REQUIRE(joint.getMaxMotorForce()==3);
    REQUIRE_THROWS_AS(joint.setLimits(false,5,4),std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setLimits(false,-inf,4),std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setLimits(false,0,nan),std::invalid_argument);
    REQUIRE(joint.areLimitsEnabled()); REQUIRE(joint.getLowerLimit()==-2); REQUIRE(joint.getUpperLimit()==4);
}
TEST_CASE("Prismatic overflow stages both endpoint corrections", "[prismatic][validation]") {
    World world(SliderConfig(1)); auto a=SliderBody(),b=SliderBody({0.1f,0});
    a->SetVelocity({0,-3e38f}); b->SetVelocity({0,3e38f});
    auto joint=std::make_shared<PrismaticJoint>(a,b);
    const auto av=a->velocity,bv=b->velocity; const float aw=a->angularVelocity,bw=b->angularVelocity;
    world.addBody(a); world.addBody(b); world.addJoint(joint);
    REQUIRE_THROWS_AS(world.step(0),std::overflow_error);
    REQUIRE(a->velocity==av); REQUIRE(b->velocity==bv);
    REQUIRE(a->angularVelocity==aw); REQUIRE(b->angularVelocity==bw);
}

TEST_CASE("Prismatic simultaneous angle and line solve retains small linear speeds", "[prismatic][prismatic-numerics]") {
    for (float lever : {1e10f,1e20f,1e30f}) for (float mass : {1e-30f,1.f,1e30f})
    for (float angle : {0.f,.37f}) {
        World world(SliderConfig(1));
        auto a=SliderBody({},true), b=SliderBody();
        a->SetOrientation(angle); b->SetOrientation(angle); b->SetMass(mass);
        const double c=std::cos(double(angle)), s=std::sin(double(angle));
        b->SetVelocity({float(3*c-s),float(3*s+c)}); b->SetAngularVelocity(1);
        const double axial=c*b->velocity.x+s*b->velocity.y;
        auto joint=std::make_shared<PrismaticJoint>(a,b,Vector2{1,0},Vector2{lever,0},Vector2{lever,0});
        world.addBody(a); world.addBody(b); world.addJoint(joint); world.step(0);
        REQUIRE(b->velocity.x==Catch::Approx(c*axial).epsilon(0).margin(5e-7));
        REQUIRE(b->velocity.y==Catch::Approx(s*axial).epsilon(0).margin(5e-7));
        REQUIRE(std::abs(b->angularVelocity)<1e-7);
        REQUIRE(a->velocity==Vector2{}); REQUIRE(a->angularVelocity==0);
    }
}

TEST_CASE("Prismatic observers retain relative local anchors at huge common translations", "[prismatic][prismatic-numerics]") {
    for (float offset : {0.f,1e30f,-1e30f}) {
        auto a=SliderBody({offset,offset},true), b=SliderBody({offset,offset});
        PrismaticJoint joint(a,b,{1,0},{},{1,2});
        REQUIRE(joint.getTranslation()==1); REQUIRE(joint.getTransverseError()==2);
        a->SetOrientation(.37f); b->SetOrientation(.37f);
        REQUIRE(joint.getTranslation()==Catch::Approx(1).epsilon(0).margin(1e-14));
        REQUIRE(joint.getTransverseError()==Catch::Approx(2).epsilon(0).margin(1e-14));
    }
}

TEST_CASE("Prismatic shared anchors preserve isolated unequal mass momentum and energy", "[prismatic][prismatic-numerics]") {
    for (float lever : {0.f,1e20f,1e30f}) {
        World world(SliderConfig(1));
        auto a=std::make_shared<RigidBody>(Circle(1),Material{},Vector2{-3,0});
        auto b=std::make_shared<RigidBody>(Circle(1),Material{},Vector2{1,0});
        a->SetMass(1); b->SetMass(3); a->SetCollisionMaskBits(0); b->SetCollisionMaskBits(0);
        a->SetVelocity({0,-1}); b->SetVelocity({0,1});
        auto joint=std::make_shared<PrismaticJoint>(a,b,Vector2{1,0},Vector2{lever,0},Vector2{lever,0});
        world.addBody(a); world.addBody(b); world.addJoint(joint); world.step(0);
        // COM speed=1/2, total inertia about COM=14, L=6, so omega=3/7.
        REQUIRE(a->velocity.y==Catch::Approx(-11./14).epsilon(0).margin(1e-7));
        REQUIRE(b->velocity.y==Catch::Approx(13./14).epsilon(0).margin(1e-7));
        REQUIRE(a->angularVelocity==Catch::Approx(3./7).epsilon(0).margin(1e-7));
        REQUIRE(b->angularVelocity==Catch::Approx(3./7).epsilon(0).margin(1e-7));
        REQUIRE(double(a->mass)*a->velocity.y+double(b->mass)*b->velocity.y==Catch::Approx(2).epsilon(0).margin(2e-7));
        REQUIRE(AngularMomentum(*a,*b)==Catch::Approx(6).epsilon(0).margin(3e-7));
        const double energy=.5*(double(a->mass)*a->velocity.y*a->velocity.y+double(b->mass)*b->velocity.y*b->velocity.y
            +double(a->inertia)*a->angularVelocity*a->angularVelocity+double(b->inertia)*b->angularVelocity*b->angularVelocity);
        REQUIRE(energy==Catch::Approx(25./14).epsilon(0).margin(3e-7));
    }
}

TEST_CASE("Prismatic long shared anchors retain coupled stops and force caps", "[prismatic][prismatic-numerics]") {
    SECTION("stops remove outward and permit inward velocity") {
        for (float sign : {-1.f,1.f}) for (bool locked : {false,true}) {
            World world(SliderConfig(1)); auto a=SliderBody({},true), b=SliderBody();
            auto joint=std::make_shared<PrismaticJoint>(a,b,Vector2{1,0},Vector2{1e30f,0},Vector2{1e30f,0});
            const float lower=locked || sign<0 ? 0.f : -1.f;
            const float upper=locked || sign>0 ? 0.f : 1.f;
            joint->setLimits(true,lower,upper);
            b->SetVelocity({sign*2,1}); b->SetAngularVelocity(1);
            world.addBody(a); world.addBody(b); world.addJoint(joint); world.step(0);
            REQUIRE(b->velocity.x==0); REQUIRE(b->velocity.y==0); REQUIRE(b->angularVelocity==0);
            b->SetVelocity({-sign,0}); world.step(0);
            REQUIRE(b->velocity.x==(locked ? 0.f : -sign));
        }
    }
    SECTION("motor force is a per-step cap with huge opposing angular impulse") {
        for (float lever : {1e20f,1e30f}) for (int iterations : {1,20}) {
            World world(SliderConfig(iterations)); auto a=SliderBody({},true), b=SliderBody();
            auto joint=std::make_shared<PrismaticJoint>(a,b,Vector2{1,0},Vector2{0,lever},Vector2{0,lever});
            joint->setMotor(true,10,3); b->SetVelocity({0,1});
            world.addBody(a); world.addBody(b); world.addJoint(joint); world.step(.125f);
            REQUIRE(b->velocity.x==Catch::Approx(.375).epsilon(0).margin(1e-7));
            REQUIRE(b->velocity.y==0); REQUIRE(b->angularVelocity==0);
            REQUIRE(joint->getMotorForce()==3);
        }
    }
}

TEST_CASE("Prismatic solvers reject corrupted static inverse properties before correction", "[prismatic][prismatic-numerics]") {
    for (bool inertia : {false,true}) {
        World world(SliderConfig(1)); auto a=SliderBody({},true), b=SliderBody();
        auto joint=std::make_shared<PrismaticJoint>(a,b);
        world.addBody(a); world.addBody(b); world.addJoint(joint);
        b->SetVelocity({1,2}); b->SetAngularVelocity(3);
        if (inertia) a->inverseInertia=1; else a->inverseMass=1;
        REQUIRE_THROWS_AS(world.step(0),std::invalid_argument);
        REQUIRE(b->velocity==Vector2{1,2}); REQUIRE(b->angularVelocity==3);
        REQUIRE(a->velocity==Vector2{}); REQUIRE(a->angularVelocity==0);
    }
}
