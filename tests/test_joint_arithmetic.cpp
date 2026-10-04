#include "catch_amalgamated.hpp"
#include "physics/core/world.h"
#include <cmath>
#include <limits>

using namespace PhysicsEngine;
namespace {
SimulationConfig JointConfig() {
    SimulationConfig c;
    c.solverIterations = 1;
    c.enableLinearVelocityLimit = false;
    c.enableAngularVelocityLimit = false;
    return c;
}
RigidBodyPtr JointBody(Vector2 p = {}, bool fixed = false) {
    auto b = std::make_shared<RigidBody>(Circle(1), Material{}, p, fixed);
    b->SetCollisionMaskBits(0);
    return b;
}
void FiniteJointBody(const RigidBody &b) {
    REQUIRE(std::isfinite(b.velocity.x));
    REQUIRE(std::isfinite(b.velocity.y));
    REQUIRE(std::isfinite(b.angularVelocity));
    REQUIRE(std::isfinite(b.position.x));
    REQUIRE(std::isfinite(b.position.y));
    REQUIRE(std::isfinite(b.orientation));
}
} // namespace
TEST_CASE("Revolute point constraint stops a heavy center-anchored body", "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody({}, true), b = JointBody({1, 0});
    b->SetMass(1e30f);
    b->SetVelocity({10, 0});
    w.addBody(a);
    w.addBody(b);
    w.addJoint(std::make_shared<RevoluteJoint>(a, b, Vector2{1, 0}));
    w.step(0);
    FiniteJointBody(*b);
    REQUIRE(b->velocity.x == Catch::Approx(0).epsilon(0).margin(1e-6));
    REQUIRE(b->velocity.y == 0);
}
TEST_CASE("Distance constraint retains physical impulse above float range", "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody({}, true), b = JointBody({1, 0});
    b->SetMass(1e38f);
    b->SetVelocity({10, 0});
    w.addBody(a);
    w.addBody(b);
    w.addJoint(std::make_shared<DistanceJoint>(a, b, 1));
    w.step(0);
    FiniteJointBody(*b);
    REQUIRE(b->velocity.x == Catch::Approx(0).epsilon(0).margin(1e-6));
}
TEST_CASE("Nonzero tiny vertical distance retains its physical axis", "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody({}, true), b = JointBody({0, 1e-10f});
    b->SetMass(1);
    b->SetVelocity({0, 1});
    w.addBody(a);
    w.addBody(b);
    w.addJoint(std::make_shared<DistanceJoint>(a, b, 1e-10f));
    w.step(0);
    REQUIRE(b->velocity.y == Catch::Approx(0).epsilon(0).margin(1e-6));
    REQUIRE(b->velocity.x == 0);
    REQUIRE(b->position.y == 1e-10f);
}
TEST_CASE("Tiny vertical distance projection follows the same axis", "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody({}, true), b = JointBody({0, 2e-10f});
    w.addBody(a);
    w.addBody(b);
    w.addJoint(std::make_shared<DistanceJoint>(a, b, 1e-10f));
    w.step(0);
    REQUIRE(b->position.x == 0);
    REQUIRE(b->position.y == Catch::Approx(1e-10).epsilon(0).margin(1e-17));
}
TEST_CASE("Heavy hinge angular stops retain impulses above float range", "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody({}, true), b = JointBody();
    b->SetMass(1e38f);
    auto joint = std::make_shared<RevoluteJoint>(a, b);
    SECTION("equal") {
        joint->setLimits(true, 0, 0);
        b->SetAngularVelocity(10);
    }
    SECTION("lower") {
        joint->setLimits(true, 0, 1);
        b->SetAngularVelocity(-10);
    }
    SECTION("upper") {
        joint->setLimits(true, -1, 0);
        b->SetAngularVelocity(10);
    }
    w.addBody(a);
    w.addBody(b);
    w.addJoint(joint);
    w.step(0);
    FiniteJointBody(*b);
    REQUIRE(b->angularVelocity == Catch::Approx(0).epsilon(0).margin(1e-6));
    REQUIRE(a->angularVelocity == 0);
}
TEST_CASE("Tiny mass joint constraints remain finite", "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody({}, true), b = JointBody({1, 1});
    b->SetMass(1e-38f);
    b->SetVelocity({1, -1});
    w.addBody(a);
    w.addBody(b);
    SECTION("distance") {
        a->SetPosition({0, 1});
        w.addJoint(std::make_shared<DistanceJoint>(a, b, 1));
        w.step(0);
        REQUIRE(b->velocity.x == Catch::Approx(0).epsilon(0).margin(1e-6));
        REQUIRE(b->velocity.y == -1);
    }
    SECTION("off-center point") {
        w.addJoint(std::make_shared<RevoluteJoint>(a, b, Vector2{}, Vector2{-1, -1}));
        w.step(0);
        const double m = b->inverseMass, i = b->inverseInertia, denominator = m + 2 * i;
        REQUIRE(b->velocity.x == Catch::Approx(1 - m / denominator).epsilon(0).margin(1e-6));
        REQUIRE(b->velocity.y == Catch::Approx(-1 + m / denominator).epsilon(0).margin(1e-6));
        REQUIRE(b->angularVelocity == Catch::Approx(-2 * i / denominator).epsilon(0).margin(1e-6));
    }
    FiniteJointBody(*b);
}
TEST_CASE("Long lever point response preserves the rotational numerator", "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody({}, true), b = JointBody();
    b->SetMass(1);
    b->SetVelocity({1, -1});
    const float r = 1e10f;
    w.addBody(a);
    w.addBody(b);
    w.addJoint(std::make_shared<RevoluteJoint>(a, b, Vector2{r, r}, Vector2{r, r}));
    w.step(0);
    const double rate = 2 * double(b->inverseInertia) * r /
                        (double(b->inverseMass) + 2 * double(b->inverseInertia) * r * r);
    REQUIRE(b->angularVelocity == Catch::Approx(rate).epsilon(0).margin(1e-17));
    REQUIRE(double(b->velocity.x) - r * double(b->angularVelocity) ==
            Catch::Approx(0).epsilon(0).margin(2e-7));
    REQUIRE(double(b->velocity.y) + r * double(b->angularVelocity) ==
            Catch::Approx(0).epsilon(0).margin(2e-7));
    FiniteJointBody(*b);
}
TEST_CASE("Long lever coupled motor obeys its analytic inertia and impulse cap",
          "[joint-arithmetic]") {
    for (int iterations : {1, 20}) {
        auto config = JointConfig();
        config.solverIterations = iterations;
        World w(config);
        auto a = JointBody(), b = JointBody();
        a->SetMass(1);
        b->SetMass(1);
        const float r = 1e5f;
        auto j = std::make_shared<RevoluteJoint>(a, b, Vector2{r, r}, Vector2{r, r});
        // Equal body angular velocities are +/-0.5; each center travels at r/2
        // along both axes. Required angular impulse is (I + 2*m*r*r)/2.
        const double required = (double(a->inertia) + 2 * double(r) * r) / 2;
        SECTION("free target") {
            j->setMotor(true, 1, 1e15f);
            w.addBody(a);
            w.addBody(b);
            w.addJoint(j);
            w.step(0.01f);
            REQUIRE(b->angularVelocity - a->angularVelocity ==
                    Catch::Approx(1).epsilon(0).margin(2e-6));
            REQUIRE(j->getMotorTorque() * double(0.01f) == Catch::Approx(required).epsilon(2e-6));
        }
        SECTION("force limited") {
            j->setMotor(true, 1, 1e10f);
            w.addBody(a);
            w.addBody(b);
            w.addJoint(j);
            w.step(0.01f);
            const double cap = double(1e10f) * double(0.01f);
            REQUIRE(std::abs(j->getMotorTorque()) <= double(1e10f));
            REQUIRE(b->angularVelocity - a->angularVelocity ==
                    Catch::Approx(cap / required).epsilon(0).margin(2e-7));
        }
        REQUIRE(double(a->velocity.x) + b->velocity.x == 0);
        REQUIRE(double(a->velocity.y) + b->velocity.y == 0);
        REQUIRE(double(b->velocity.x) - a->velocity.x -
                    r * double(b->angularVelocity - a->angularVelocity) ==
                Catch::Approx(0).epsilon(0).margin(0.02));
        REQUIRE(double(b->velocity.y) - a->velocity.y +
                    r * double(b->angularVelocity - a->angularVelocity) ==
                Catch::Approx(0).epsilon(0).margin(0.02));
        FiniteJointBody(*a);
        FiniteJointBody(*b);
    }
}
TEST_CASE("Coupled hinge preserves small net torque between enormous cancelling impulses",
          "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody(), b = JointBody();
    a->SetMass(1);
    b->SetMass(1);
    const float r = 1e10f;
    auto j = std::make_shared<RevoluteJoint>(a, b, Vector2{r, r}, Vector2{r, r});
    SECTION("motor") {
        j->setMotor(true, 1, 1e25f);
        w.addBody(a);
        w.addBody(b);
        w.addJoint(j);
        w.step(0.01f);
        REQUIRE(a->angularVelocity == Catch::Approx(-0.5).epsilon(0).margin(1e-6));
        REQUIRE(b->angularVelocity == Catch::Approx(0.5).epsilon(0).margin(1e-6));
    }
    SECTION("equal angular stop") {
        j->setLimits(true, 0, 0);
        b->SetVelocity({r, -r});
        b->SetAngularVelocity(1);
        w.addBody(a);
        w.addBody(b);
        w.addJoint(j);
        w.step(0);
        REQUIRE(a->angularVelocity == Catch::Approx(0.5).epsilon(0).margin(1e-6));
        REQUIRE(b->angularVelocity == Catch::Approx(0.5).epsilon(0).margin(1e-6));
    }
    SECTION("capped motor") {
        j->setMotor(true, 1, 1e20f);
        w.addBody(a);
        w.addBody(b);
        w.addJoint(j);
        w.step(0.01f);
        const double cap = double(1e20f) * double(0.01f), inertia = double(r) * r + 0.25;
        REQUIRE(b->angularVelocity - a->angularVelocity ==
                Catch::Approx(cap / inertia).epsilon(0).margin(2e-8));
        REQUIRE(std::abs(j->getMotorTorque()) <= double(1e20f));
        REQUIRE(b->velocity.x == Catch::Approx(double(r) * cap / inertia / 2).epsilon(0).margin(8));
        REQUIRE(b->velocity.y ==
                Catch::Approx(-double(r) * cap / inertia / 2).epsilon(0).margin(8));
    }
}
TEST_CASE("Joint pair retains velocities on genuine angular overflow", "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody({1, -1});
    a->SetMass(1);
    auto b = std::make_shared<RigidBody>(Circle(1e-19f), Material{1e38f, 0}, Vector2{1, 0});
    b->SetCollisionMaskBits(0);
    b->SetMass(1);
    b->SetVelocity({0, std::numeric_limits<float>::max()});
    w.addBody(a);
    w.addBody(b);
    w.addJoint(std::make_shared<DistanceJoint>(a, b, 1, Vector2{}, Vector2{1e-19f, 0}));
    const Vector2 va = a->velocity, vb = b->velocity;
    REQUIRE_THROWS_AS(w.step(0), std::overflow_error);
    REQUIRE(a->velocity == va);
    REQUIRE(b->velocity == vb);
    REQUIRE(a->angularVelocity == 0);
    REQUIRE(b->angularVelocity == 0);
}
TEST_CASE("Joint pair retains poses on genuine position angular overflow", "[joint-arithmetic]") {
    auto config = JointConfig();
    config.maxPositionCorrection = std::numeric_limits<float>::max();
    World w(config);
    auto a = JointBody({0, -2e38f});
    a->SetMass(1);
    auto b = std::make_shared<RigidBody>(Circle(1e-19f), Material{1e38f, 0}, Vector2{0, 2e38f});
    b->SetCollisionMaskBits(0);
    b->SetMass(1);
    w.addBody(a);
    w.addBody(b);
    w.addJoint(std::make_shared<DistanceJoint>(a, b, 1, Vector2{}, Vector2{1e-19f, 0}));
    const Vector2 pa = a->position, pb = b->position;
    REQUIRE_THROWS_AS(w.step(0), std::overflow_error);
    REQUIRE(a->position == pa);
    REQUIRE(b->position == pb);
    REQUIRE(a->orientation == 0);
    REQUIRE(b->orientation == 0);
}
TEST_CASE("Failed coupled hinge correction retains both velocities and motor accounting",
          "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody(), b = JointBody();
    a->SetMass(1);
    b->SetMass(1);
    const float maximum = std::numeric_limits<float>::max();
    a->SetVelocity({maximum, 0});
    b->SetVelocity({maximum, 0});
    auto j = std::make_shared<RevoluteJoint>(a, b, Vector2{0, 1}, Vector2{0, 1});
    j->setMotor(true, maximum, maximum);
    w.addBody(a);
    w.addBody(b);
    w.addJoint(j);
    REQUIRE_THROWS_AS(w.step(1), std::overflow_error);
    REQUIRE(a->velocity == Vector2(maximum, 0));
    REQUIRE(b->velocity == Vector2(maximum, 0));
    REQUIRE(a->angularVelocity == 0);
    REQUIRE(b->angularVelocity == 0);
    REQUIRE(j->getMotorTorque() == 0);
}
TEST_CASE("Double solver anchors may exceed public float anchor representation",
          "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody({2e38f, 0}, true), b = JointBody({2e38f, 0});
    b->SetVelocity({2, 0});
    auto j = std::make_shared<DistanceJoint>(a, b, 1, Vector2{2e38f, 0}, Vector2{2e38f, 0});
    REQUIRE_THROWS_AS(j->getAnchorA(), std::overflow_error);
    REQUIRE_THROWS_AS(j->getAnchorB(), std::overflow_error);
    w.addBody(a);
    w.addBody(b);
    w.addJoint(j);
    w.step(0);
    REQUIRE(b->velocity.x == Catch::Approx(0).epsilon(0).margin(1e-6));
    FiniteJointBody(*b);
}
TEST_CASE("Off-center joint impulses conserve linear and angular momentum", "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody({-1, 0}), b = JointBody({1, 0});
    a->SetMass(2);
    b->SetMass(4);
    a->SetVelocity({4, 1});
    b->SetVelocity({-2, 3});
    a->SetAngularVelocity(2);
    b->SetAngularVelocity(-1);
    auto momentum = [&]() {
        return std::pair<double, double>{
            double(a->mass) * a->velocity.x + double(b->mass) * b->velocity.x,
            double(a->mass) * a->velocity.y + double(b->mass) * b->velocity.y};
    };
    auto angular = [&]() {
        return double(a->mass) *
                   (double(a->position.x) * a->velocity.y - double(a->position.y) * a->velocity.x) +
               double(a->inertia) * a->angularVelocity +
               double(b->mass) *
                   (double(b->position.x) * b->velocity.y - double(b->position.y) * b->velocity.x) +
               double(b->inertia) * b->angularVelocity;
    };
    const auto p = momentum();
    const double l = angular();
    JointPtr j;
    SECTION("distance") {
        j = std::make_shared<DistanceJoint>(a, b, 2, Vector2{0, 1}, Vector2{0, 1});
    }
    SECTION("revolute") {
        j = std::make_shared<RevoluteJoint>(a, b, Vector2{1, 1}, Vector2{-1, 1});
    }
    w.addBody(a);
    w.addBody(b);
    w.addJoint(j);
    w.step(0);
    REQUIRE(momentum().first == Catch::Approx(p.first).epsilon(0).margin(3e-6));
    REQUIRE(momentum().second == Catch::Approx(p.second).epsilon(0).margin(3e-6));
    REQUIRE(angular() == Catch::Approx(l).epsilon(0).margin(3e-6));
    const Vector2 relative =
        b->GetVelocityAtPoint(j->getAnchorB()) - a->GetVelocityAtPoint(j->getAnchorA());
    REQUIRE(relative.x == Catch::Approx(0).epsilon(0).margin(1e-6));
    if (dynamic_cast<RevoluteJoint *>(j.get()))
        REQUIRE(relative.y == Catch::Approx(0).epsilon(0).margin(1e-6));
}
TEST_CASE("Joints reject invalid consumed legacy body values", "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody({}, true), b = JointBody({1, 0});
    auto j = std::make_shared<DistanceJoint>(a, b, 1);
    w.addBody(a);
    w.addBody(b);
    w.addJoint(j);
    const float nan = std::numeric_limits<float>::quiet_NaN(),
                inf = std::numeric_limits<float>::infinity();
    SECTION("position") {
        b->position.x = nan;
    }
    SECTION("orientation") {
        b->orientation = inf;
    }
    SECTION("linear velocity") {
        b->velocity.y = nan;
    }
    SECTION("angular velocity") {
        b->angularVelocity = inf;
    }
    SECTION("inverse mass zero") {
        b->inverseMass = 0;
    }
    SECTION("inverse mass negative") {
        b->inverseMass = -1;
    }
    SECTION("inverse inertia infinity") {
        b->inverseInertia = inf;
    }
    SECTION("inverse inertia zero") {
        b->inverseInertia = 0;
    }
    SECTION("static inverse mass") {
        a->inverseMass = 1;
    }
    SECTION("static inverse inertia") {
        a->inverseInertia = 1;
    }
    REQUIRE_THROWS_AS(w.step(0), std::invalid_argument);
}
TEST_CASE("Static support velocity participates without moving the support", "[joint-arithmetic]") {
    World w(JointConfig());
    auto a = JointBody({}, true), b = JointBody({1, 0});
    a->velocity = {1, 2};
    a->angularVelocity = 3;
    w.addBody(a);
    w.addBody(b);
    w.addJoint(std::make_shared<RevoluteJoint>(a, b, Vector2{1, 0}));
    w.step(0);
    REQUIRE(b->velocity == Vector2(1, 5));
    REQUIRE(a->velocity == Vector2(1, 2));
    REQUIRE(a->angularVelocity == 3);
    REQUIRE(a->position == Vector2{});
    REQUIRE(a->orientation == 0);
}
TEST_CASE("Locked hinge retains center velocity below an enormous shared rotational speed", "[joint-arithmetic][hinge-rhs]") {
    for(float r:{1e10f,1e20f,1e30f}) for(float angle:{0.f,.37f}) for(float mass:{1e-30f,1.f,1e30f}) {
        World w(JointConfig()); auto a=JointBody({1e30f,0},true),b=JointBody({1e30f,0});
        a->SetOrientation(angle); b->SetOrientation(angle); b->SetMass(mass);
        b->SetVelocity({1,-2}); b->SetAngularVelocity(1);
        auto j=std::make_shared<RevoluteJoint>(a,b,Vector2{r,0},Vector2{r,0}); j->setLimits(true,0,0);
        w.addBody(a); w.addBody(b); w.addJoint(j); w.step(0);
        REQUIRE(b->velocity.x==Catch::Approx(0).epsilon(0).margin(2e-7));
        REQUIRE(b->velocity.y==Catch::Approx(0).epsilon(0).margin(4e-7));
        REQUIRE(b->angularVelocity==0); REQUIRE(a->angularVelocity==0);
    }
}
TEST_CASE("One-sided long-anchor stops lock outward motion and retain tiny inward release", "[joint-arithmetic][hinge-rhs]") {
    const float direction=GENERATE(-1.f,1.f);
    for(float r:{1e10f,1e20f,1e30f}) {
        World w(JointConfig()); auto a=JointBody({},true),b=JointBody(); b->SetMass(1);
        auto j=std::make_shared<RevoluteJoint>(a,b,Vector2{r,0},Vector2{r,0});
        j->setLimits(true,direction>0?-1:0,direction>0?0:1);
        b->SetVelocity({0,-direction}); b->SetAngularVelocity(direction);
        w.addBody(a); w.addBody(b); w.addJoint(j); w.step(0);
        REQUIRE(std::abs(b->velocity.y)<2e-7); REQUIRE(b->angularVelocity==0);
        b->SetVelocity({0,direction}); b->SetAngularVelocity(direction); w.step(0);
        const double inverse=b->inverseInertia,denominator=1+inverse*double(r)*r;
        const double expectedSpin=direction*(1-inverse*r)/denominator;
        const double expectedLinear=direction*(inverse*double(r)*r-r)/denominator;
        REQUIRE(b->velocity.y==Catch::Approx(expectedLinear).epsilon(0).margin(2e-7));
        REQUIRE(b->angularVelocity==Catch::Approx(expectedSpin).epsilon(0).margin(std::abs(expectedSpin)*3e-7));
        REQUIRE(double(b->velocity.y)+r*double(b->angularVelocity)==Catch::Approx(0).epsilon(0).margin(3e-7));
    }
}
TEST_CASE("Capped hinge motor retains the independent center velocity in the point response", "[joint-arithmetic][hinge-rhs]") {
    constexpr float dt=.125f;
    for(float torque:{0.f,1e20f}) {
        World w(JointConfig()); auto a=JointBody({},true),b=JointBody({0,-dt}); b->SetMass(1);
        // Arrive at identical orientations/positions at the velocity phase.
        b->SetOrientation(-dt); b->SetVelocity({0,1}); b->SetAngularVelocity(1);
        const float r=1e20f; auto j=std::make_shared<RevoluteJoint>(a,b,Vector2{r,0},Vector2{r,0});
        j->setMotor(true,0,torque); w.addBody(a); w.addBody(b); w.addJoint(j); w.step(dt);
        const double cap=double(torque)*dt,inverse=b->inverseInertia,denominator=1+inverse*double(r)*r;
        const double expectedSpin=(1-inverse*r+inverse*cap)/denominator;
        const double expectedLinear=(inverse*double(r)*r-r-inverse*r*cap)/denominator;
        REQUIRE(b->velocity.y==Catch::Approx(expectedLinear).epsilon(0).margin(2e-7));
        REQUIRE(b->angularVelocity==Catch::Approx(expectedSpin).epsilon(0).margin(std::abs(expectedSpin)*3e-7));
        REQUIRE(j->getMotorTorque()==Catch::Approx(double(torque)).epsilon(2e-7));
        REQUIRE(double(b->velocity.y)+r*double(b->angularVelocity)==Catch::Approx(0).epsilon(0).margin(3e-7));
    }
}
TEST_CASE("Unsaturated long-anchor hinge motor reaches its small prescribed angular target", "[joint-arithmetic][hinge-rhs]") {
    constexpr float dt=.125f,r=1e20f,target=1e-20f;
    World w(JointConfig()); auto a=JointBody({},true),b=JointBody({0,-dt}); b->SetMass(1); b->SetVelocity({0,1});
    auto j=std::make_shared<RevoluteJoint>(a,b,Vector2{r,0},Vector2{r,0}); j->setMotor(true,target,1e25f);
    w.addBody(a); w.addBody(b); w.addJoint(j); w.step(dt);
    const double expectedLinear=-double(r)*target,expectedImpulse=target/double(b->inverseInertia)+double(r)*(1+double(r)*target);
    REQUIRE(b->velocity.y==Catch::Approx(expectedLinear).epsilon(0).margin(2e-7));
    REQUIRE(b->angularVelocity==Catch::Approx(double(target)).epsilon(0).margin(3e-27));
    REQUIRE(j->getMotorTorque()==Catch::Approx(expectedImpulse/dt).epsilon(3e-7));
    REQUIRE(std::abs(j->getMotorTorque())<1e25);
}
TEST_CASE("Common long-anchor locked hinge retains unequal-mass momentum and expected energy loss", "[joint-arithmetic][hinge-rhs]") {
    World w(JointConfig()); auto a=JointBody(),b=JointBody(); a->SetMass(2); b->SetMass(4);
    a->SetVelocity({4,1}); b->SetVelocity({-2,3}); a->SetAngularVelocity(3); b->SetAngularVelocity(-1);
    auto j=std::make_shared<RevoluteJoint>(a,b,Vector2{1e30f,0},Vector2{1e30f,0}); j->setLimits(true,0,0);
    w.addBody(a); w.addBody(b); w.addJoint(j); w.step(0);
    REQUIRE(a->velocity.x==Catch::Approx(0).epsilon(0).margin(2e-7)); REQUIRE(b->velocity.x==Catch::Approx(0).epsilon(0).margin(2e-7));
    REQUIRE(a->velocity.y==Catch::Approx(7./3).epsilon(0).margin(3e-7)); REQUIRE(b->velocity.y==Catch::Approx(7./3).epsilon(0).margin(3e-7));
    REQUIRE(a->angularVelocity==Catch::Approx(1./3).epsilon(0).margin(4e-8)); REQUIRE(b->angularVelocity==Catch::Approx(1./3).epsilon(0).margin(4e-8));
    REQUIRE(2*double(a->velocity.y)+4*double(b->velocity.y)==Catch::Approx(14).epsilon(0).margin(2e-6));
    REQUIRE(double(a->inertia)*a->angularVelocity+double(b->inertia)*b->angularVelocity==Catch::Approx(1).epsilon(0).margin(1e-7));
    const double energy=.5*2*double(a->velocity.y)*a->velocity.y+.5*4*double(b->velocity.y)*b->velocity.y
        +.5*a->inertia*double(a->angularVelocity)*a->angularVelocity+.5*b->inertia*double(b->angularVelocity)*b->angularVelocity;
    REQUIRE(energy==Catch::Approx(16.5).epsilon(0).margin(5e-6));
}
TEST_CASE("Inactive hinge retains the original point-only long-anchor response", "[joint-arithmetic][hinge-rhs]") {
    for(bool limits:{false,true}) {
        World w(JointConfig()); auto a=JointBody({},true),b=JointBody(); b->SetMass(1); b->SetVelocity({0,1});
        constexpr float r=1e20f; auto j=std::make_shared<RevoluteJoint>(a,b,Vector2{r,0},Vector2{r,0});
        j->setLimits(limits,-1,1); w.addBody(a); w.addBody(b); w.addJoint(j); w.step(0);
        const double expected=-double(b->inverseInertia)*r/(1+double(b->inverseInertia)*r*r);
        REQUIRE(b->velocity.y==Catch::Approx(1).epsilon(0).margin(2e-7));
        REQUIRE(b->angularVelocity==Catch::Approx(expected).epsilon(0).margin(std::abs(expected)*3e-7));
        REQUIRE(double(b->velocity.y)+r*double(b->angularVelocity)==Catch::Approx(0).epsilon(0).margin(3e-7));
    }
}
TEST_CASE("Reduced hinge motor overflow retains both velocity states and unaccepted drive cache", "[joint-arithmetic][hinge-rhs]") {
    World w(JointConfig());
    auto a=std::make_shared<RigidBody>(Circle(1e-19f),Material{1e38f,0}),b=std::make_shared<RigidBody>(Circle(1e-19f),Material{1e38f,0});
    a->SetCollisionMaskBits(0); b->SetCollisionMaskBits(0); a->SetMass(1); b->SetMass(1);
    b->SetVelocity({0,std::numeric_limits<float>::max()});
    auto j=std::make_shared<RevoluteJoint>(a,b,Vector2{},Vector2{1e-19f,0}); j->setMotor(true,0,std::numeric_limits<float>::max());
    w.addBody(a); w.addBody(b); w.addJoint(j); const auto va=a->velocity,vb=b->velocity;
    REQUIRE_THROWS_AS(w.step(.125f),std::overflow_error);
    REQUIRE(a->velocity==va); REQUIRE(b->velocity==vb); REQUIRE(a->angularVelocity==0); REQUIRE(b->angularVelocity==0); REQUIRE(j->getMotorTorque()==0);
}
TEST_CASE("Offcenter unequal hinge motor matches independent rational block and cap solutions", "[joint-arithmetic][hinge-rhs]") {
    const bool capped=GENERATE(false,true); constexpr float dt=.125f;
    World w(JointConfig()); auto a=JointBody({-1.5f,-.125f}),b=JointBody({1.25f,-.375f});
    a->SetMass(2); b->SetMass(4); a->SetVelocity({4,1}); b->SetVelocity({-2,3});
    a->SetAngularVelocity(2); b->SetAngularVelocity(-1); a->SetOrientation(-.25f); b->SetOrientation(.125f);
    auto j=std::make_shared<RevoluteJoint>(a,b,Vector2{1,.5f},Vector2{-1,.5f}); j->setMotor(true,.75f,capped?2:100);
    w.addBody(a); w.addBody(b); w.addJoint(j); w.step(dt);
    // At the velocity phase rA=(1,.5), rB=(-1,.5), inv masses=.5/.25,
    // inv inertias=1/.5. Independent rational matrix:
    // [9/8,-1/4,-3/4; -1/4,9/4,1/2; -3/4,1/2,3/2] j = [9/2,-1,15/4].
    // Its exact solution is (17/2,-27/25,711/100). Fixing j_angle=1/4
    // gives the separate point-row solution (657/158,-3/79).
    const double px=capped?657./158:17./2,py=capped?-3./79:-27./25,q=capped?.25:711./100;
    const double avx=4-.5*px,avy=1-.5*py,bvx=-2+.25*px,bvy=3+.25*py;
    const double aw=2-(py-.5*px+q),bw=-1+.5*(-py-.5*px+q);
    REQUIRE(a->velocity.x==Catch::Approx(avx).epsilon(0).margin(6e-7)); REQUIRE(a->velocity.y==Catch::Approx(avy).epsilon(0).margin(6e-7));
    REQUIRE(b->velocity.x==Catch::Approx(bvx).epsilon(0).margin(6e-7)); REQUIRE(b->velocity.y==Catch::Approx(bvy).epsilon(0).margin(6e-7));
    REQUIRE(a->angularVelocity==Catch::Approx(aw).epsilon(0).margin(4e-7)); REQUIRE(b->angularVelocity==Catch::Approx(bw).epsilon(0).margin(4e-7));
    REQUIRE(j->getMotorTorque()==Catch::Approx(q/dt).epsilon(0).margin(2e-6));
    REQUIRE(2*double(a->velocity.x)+4*double(b->velocity.x)==Catch::Approx(0).epsilon(0).margin(2e-6));
    REQUIRE(2*double(a->velocity.y)+4*double(b->velocity.y)==Catch::Approx(14).epsilon(0).margin(3e-6));
    const double angular=-2*double(a->velocity.y)+4*double(b->velocity.y)+a->angularVelocity+2*double(b->angularVelocity);
    REQUIRE(angular==Catch::Approx(10).epsilon(0).margin(3e-6));
    const double energy=double(a->velocity.x)*a->velocity.x+double(a->velocity.y)*a->velocity.y
        +2*(double(b->velocity.x)*b->velocity.x+double(b->velocity.y)*b->velocity.y)
        +.5*double(a->angularVelocity)*a->angularVelocity+double(b->angularVelocity)*b->angularVelocity;
    const double expected=avx*avx+avy*avy+2*(bvx*bvx+bvy*bvy)+.5*aw*aw+bw*bw;
    REQUIRE(energy==Catch::Approx(expected).epsilon(0).margin(6e-6));
}
