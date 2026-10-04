#include "catch_amalgamated.hpp"
#include "../benchmarks/rigid_contact_metrics.h"

using namespace RigidContactDiagnostic;

TEST_CASE("Rigid contact impact matrix matches independent Newtonian impulse oracles", "[contact-oracle]") {
    for(const auto& fixture:Fixtures(true))if(fixture.impact)
        for(int iterations:{1,10}) {
            const auto r=Run(fixture,{1.0f/60,iterations,0.8f,0.1});
            INFO(fixture.name<<" iterations="<<iterations);
            REQUIRE(r.begins==1);REQUIRE(r.positionalProjection==0);REQUIRE(r.angularProjection==0);
            if(!fixture.offCenter) {
                // Independent closed form from momentum and Newton restitution,
                // separate from the benchmark's effective-mass impulse formula.
                const double m=fixture.massRatio,e=fixture.restitution;
                const double a=2*(1-e*m)/(1+m),b=2*(1+e)/(1+m);
                REQUIRE(r.states[0][3]==Catch::Approx(a).epsilon(0).margin(3e-6));
                REQUIRE(r.states[1][3]==Catch::Approx(b).epsilon(0).margin(3e-6));
                const double energy=2-2*m*(1-e*e)/(1+m);
                REQUIRE(r.final.kinetic==Catch::Approx(energy).epsilon(0).margin(3e-6));
                REQUIRE(r.expectedA==Catch::Approx(a).epsilon(0).margin(1e-15));
                REQUIRE(r.expectedB==Catch::Approx(b).epsilon(0).margin(1e-15));
            } else {
                // Circle m=1,u=3; square m=2,I=1/3; horizontal lever y=.3f.
                const double y=0.3f,j=4.5/(1.5+3*y*y);
                REQUIRE(r.states[0][3]==Catch::Approx(3-j).epsilon(0).margin(3e-6));
                REQUIRE(r.states[1][3]==Catch::Approx(j/2).epsilon(0).margin(3e-6));
                REQUIRE(r.states[1][5]==Catch::Approx(-3*y*j).epsilon(0).margin(3e-6));
            }
            // Eight float ulps at incident speed bound scalar response rounding.
            REQUIRE(r.velocityError<3e-6);
            REQUIRE(r.final.px==Catch::Approx(r.initial.px).epsilon(0).margin(3e-6));
            REQUIRE(r.final.py==Catch::Approx(r.initial.py).epsilon(0).margin(3e-6));
            REQUIRE(r.final.angular==Catch::Approx(r.initial.angular).epsilon(0).margin(3e-6));
            REQUIRE(r.energyError==Catch::Approx(0).epsilon(0).margin(3e-6));
            REQUIRE(r.final.kinetic<=r.initial.kinetic+3e-6);
        }
}

TEST_CASE("Rigid contact diagnostic geometry measures independently transformed shapes", "[contact-oracle]") {
    RigidBody a(Polygon::MakeBox(2,2),Material{}),b(Polygon::MakeBox(2,2),Material{},Vector2{1.5f,0});
    REQUIRE(Penetration(a,b)==Catch::Approx(0.5).epsilon(0).margin(1e-12));
    b.SetPosition({2,0});REQUIRE(Penetration(a,b)==0);
    RigidBody circle(Circle(0.5f),Material{},Vector2{1.25f,0});
    REQUIRE(Penetration(a,circle)==0.25);
    circle.SetPosition({1.4f,1.4f});REQUIRE(Penetration(a,circle)==0);
    circle.SetPosition({});REQUIRE(Penetration(a,circle)==1.5);
    auto measured=std::make_shared<RigidBody>(Circle(1),Material{},Vector2{3,4});
    measured->SetMass(2);measured->SetVelocity({5,-2});measured->SetAngularVelocity(0.5f);
    const auto m=Measure({measured},9.81);
    REQUIRE(m.px==10);REQUIRE(m.py==-4);REQUIRE(m.angular==-51.5);
    REQUIRE(m.kinetic==29.125);REQUIRE(m.potential==Catch::Approx(78.48).epsilon(0).margin(1e-12));
}

TEST_CASE("Rigid contact benchmark controls and deterministic observations are validated", "[contact-diagnostic]") {
    const Controls controls{1.0f/120,10,0.8f,0.1};
    const auto first=Run(Fixtures(true)[1],controls),repeat=Run(Fixtures(true)[1],controls);
    REQUIRE(first.states==repeat.states);REQUIRE(first.featureChanges==repeat.featureChanges);
    REQUIRE(first.integrated==repeat.integrated);REQUIRE(first.begins==repeat.begins);REQUIRE(first.persists==repeat.persists);
    REQUIRE(first.steps==12);REQUIRE(first.bodies==4);REQUIRE(first.finalPersistent<=6); // At most one entry per unordered body pair.
    REQUIRE_THROWS_AS(Validate({0,10,0.8f,2}),std::invalid_argument);
    REQUIRE_THROWS_AS(Validate({1.0f/60,65,0.8f,2}),std::invalid_argument);
    REQUIRE_THROWS_AS(Validate({1.0f/60,10,1.01f,2}),std::invalid_argument);
    REQUIRE_THROWS_AS(Validate({1.0f/60,10,0.8f,9}),std::invalid_argument);
}
