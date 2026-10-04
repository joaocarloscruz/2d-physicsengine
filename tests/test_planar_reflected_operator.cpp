#include "catch_amalgamated.hpp"
#include "../benchmarks/planar_reflected_operator.h"
#include <limits>
using namespace PlanarReflection;
using PhysicsEngine::SphKernelFamily;
namespace {
Vec Rotate(Vec v,double angle) { return {v.x*std::cos(angle)-v.y*std::sin(angle),v.x*std::sin(angle)+v.y*std::cos(angle)}; }
Plane Advance(Plane plane,double dt) {
    plane.point=plane.center+Rotate(plane.point-plane.center,plane.angularVelocity*dt)+plane.velocity*dt;
    plane.normal=Rotate(plane.normal,plane.angularVelocity*dt);
    plane.center=plane.center+plane.velocity*dt;
    return plane;
}
std::vector<Particle> State() {
    return {{{-.17,.13},{.4,-.3},2,997,2300},{{.28,.19},{-.2,.7},3,1012,-120},{{.08,.37},{.1,-.5},4,1008,4200}};
}
}
TEST_CASE("Reflected source velocity differentiates arbitrary translating rotating plane geometry", "[fluid][planar-reflection][oracle]") {
    Plane plane; plane.point={.11,-.2}; plane.center={-.3,.17}; plane.normal=Rotate({0,1},.31);
    plane.velocity={.27,-.36}; plane.angularVelocity=.73;
    Particle p; p.position={.38,.52}; p.velocity={-.41,.26};
    const double dt=1e-6;
    auto plus=p,minus=p; plus.position=p.position+p.velocity*dt; minus.position=p.position-p.velocity*dt;
    const Vec derivative=(Advance(plane,dt).ReflectPosition(plus.position)-Advance(plane,-dt).ReflectPosition(minus.position))*(.5/dt);
    REQUIRE((derivative-plane.GhostVelocity(p)).norm()<2e-10);
    const Vec incomplete=plane.ReflectVector(p.velocity)+plane.normal*(2*plane.normal.dot(plane.VelocityAt(plane.Projection(p.position))));
    REQUIRE((derivative-incomplete).norm()>.1);
}
TEST_CASE("Reflected pressure adjoint includes source self accumulation and complete wall wrench", "[fluid][planar-reflection][oracle]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    Plane plane; plane.velocity={.2,-.1}; plane.angularVelocity=.7; plane.center={-.3,.1};
    const auto p=State(); const auto out=Evaluate(p,plane,1,family);
    REQUIRE(out.totalForce.norm()<2e-12);
    REQUIRE(std::abs(out.totalTorque)<2e-12);
    REQUIRE(std::abs(out.workResidual)<2e-12);
    REQUIRE(std::abs(out.wallCouple)>1e-4);
    REQUIRE(std::abs(out.wallTorque-out.wallProjectionTorque)>1e-4);
    const auto single=Evaluate({p[0]},plane,1,family);
    const Vec g=Gradient(p[0].position-plane.ReflectPosition(p[0].position),1,family);
    const double dual=p[0].mass*p[0].pressure/(p[0].density*p[0].density);
    REQUIRE((single.forces[0]-g*(-2*dual*p[0].mass)).norm()<1e-12);
    REQUIRE(std::abs(single.workResidual)<1e-12);
}
TEST_CASE("Planar reflected continuity has tangent and joint rigid null modes", "[fluid][planar-reflection][oracle]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    Plane plane; auto p=State();
    for(auto& a:p) a.velocity={.4,0};
    auto out=Evaluate(p,plane,1,family);
    for(double rate:out.densityRates) REQUIRE(rate==0);
    plane.velocity={.21,-.13}; plane.angularVelocity=.71; plane.center={.12,-.17};
    for(auto& a:p) a.velocity=plane.VelocityAt(a.position);
    out=Evaluate(p,plane,1,family);
    for(double rate:out.densityRates) REQUIRE(std::abs(rate)<1e-12);
    plane.angularVelocity=0;
    for(auto& a:p) a.velocity=plane.velocity;
    out=Evaluate(p,plane,1,family);
    for(double rate:out.densityRates) REQUIRE(std::abs(rate)<1e-12);
}
TEST_CASE("Cubic reflected continuity differentiates owned density quadrature", "[fluid][planar-reflection][oracle]") {
    Plane plane; plane.velocity={.2,-.1}; plane.angularVelocity=.7; plane.center={-.3,.1};
    const auto p=State(); const auto out=Evaluate(p,plane,1,SphKernelFamily::CubicSpline);
    auto plus=p,minus=p; const double dt=1e-6;
    for(std::size_t i=0;i<p.size();++i) {
        plus[i].position=p[i].position+p[i].velocity*dt;
        minus[i].position=p[i].position-p[i].velocity*dt;
    }
    const auto a=Evaluate(plus,Advance(plane,dt),1,SphKernelFamily::CubicSpline);
    const auto b=Evaluate(minus,Advance(plane,-dt),1,SphKernelFamily::CubicSpline);
    for(std::size_t i=0;i<p.size();++i)
        REQUIRE((a.summedDensities[i]-b.summedDensities[i])/(2*dt)==Catch::Approx(out.densityRates[i]).epsilon(2e-8));
    const auto legacy=Evaluate(p,plane,1,SphKernelFamily::Poly6Spiky);
    const auto la=Evaluate(plus,Advance(plane,dt),1,SphKernelFamily::Poly6Spiky);
    const auto lb=Evaluate(minus,Advance(plane,-dt),1,SphKernelFamily::Poly6Spiky);
    REQUIRE(std::abs((la.summedDensities[0]-lb.summedDensities[0])/(2*dt)-legacy.densityRates[0])>1);
}
TEST_CASE("Planar pressure dual closes independent velocity basis probes", "[fluid][planar-reflection][oracle]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    auto p=State(); Plane plane; plane.center={-.3,.1};
    for(auto& a:p) a.velocity={};
    for(std::size_t k=0;k<2*p.size()+3;++k) {
        auto probe=p; auto wall=plane;
        if(k<2*p.size()) { if(k%2) probe[k/2].velocity.y=1; else probe[k/2].velocity.x=1; }
        else if(k==2*p.size()) wall.velocity.x=1;
        else if(k==2*p.size()+1) wall.velocity.y=1;
        else wall.angularVelocity=1;
        const auto out=Evaluate(probe,wall,1,family);
        REQUIRE(std::abs(out.workResidual)<1e-12);
    }
}
TEST_CASE("Cubic pressure forces and wall wrench differentiate an independent frozen dual potential", "[fluid][planar-reflection][oracle]") {
    const auto p=State(); Plane plane; plane.center={-.3,.1};
    const auto out=Evaluate(p,plane,1,SphKernelFamily::CubicSpline);
    auto potential=[&](const std::vector<Particle>& state,const Plane& wall) {
        double value=0;
        for(std::size_t i=0;i<p.size();++i) for(std::size_t j=0;j<p.size();++j) {
            const double dual=(p[i].mass/p[i].density)*(p[i].pressure/p[i].density);
            value+=dual*p[j].mass*(Weight(state[i].position-state[j].position,1,SphKernelFamily::CubicSpline)
                +Weight(state[i].position-wall.ReflectPosition(state[j].position),1,SphKernelFamily::CubicSpline));
        }
        return value;
    };
    const double dt=1e-6;
    for(std::size_t k=0;k<2*p.size()+3;++k) {
        auto plus=p,minus=p; auto wall=plane; double force;
        if(k<2*p.size()) {
            const Vec delta=k%2?Vec{0,dt}:Vec{dt,0};
            plus[k/2].position=plus[k/2].position+delta;
            minus[k/2].position=minus[k/2].position-delta;
            force=k%2?out.forces[k/2].y:out.forces[k/2].x;
        } else if(k==2*p.size()) { wall.velocity={1,0}; force=out.wallForce.x; }
        else if(k==2*p.size()+1) { wall.velocity={0,1}; force=out.wallForce.y; }
        else { wall.angularVelocity=1; force=out.wallTorque; }
        const double derivative=(potential(plus,Advance(wall,dt))-potential(minus,Advance(wall,-dt)))/(2*dt);
        REQUIRE(std::abs(derivative+force)<1e-8);
    }
}
TEST_CASE("Independent planar kernel formulas agree with public kernels", "[fluid][planar-reflection][oracle]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    for(double radius:{0.,.12,.51,.99,1.}) {
        const Vec r{radius*.6,radius*.8};
        PhysicsEngine::Vector2 f{static_cast<float>(r.x),static_cast<float>(r.y)};
        REQUIRE(Weight(r,1,family)==Catch::Approx(PhysicsEngine::SphKernels2D::DensityWeight(f,1,family)).margin(1e-6));
        const auto actual=PhysicsEngine::SphKernels2D::PressureGradient(f,1,family);
        REQUIRE((Gradient(r,1,family)-Vec{actual.x,actual.y}).norm()<2e-6);
    }
}
TEST_CASE("Planar prototype rejects invalid and excessive input", "[fluid][planar-reflection][validation]") {
    auto p=State(); Plane plane;
    REQUIRE_THROWS_AS(Evaluate(p,plane,0,SphKernelFamily::CubicSpline),std::invalid_argument);
    plane.normal={0,2}; REQUIRE_THROWS_AS(Evaluate(p,plane,1,SphKernelFamily::CubicSpline),std::invalid_argument);
    plane.normal={0,1}; p[0].position.y=0;
    REQUIRE_THROWS_AS(Evaluate(p,plane,1,SphKernelFamily::CubicSpline),std::invalid_argument);
    REQUIRE_THROWS_AS(Evaluate(std::vector<Particle>(1025),plane,1,SphKernelFamily::CubicSpline),std::length_error);
    p=State(); p[0].mass=std::numeric_limits<double>::max(); p[0].pressure=0;
    REQUIRE_THROWS_AS(Evaluate(p,plane,1,SphKernelFamily::CubicSpline),std::overflow_error);
}

TEST_CASE("Owned reflection balances a regular lattice and preserves common tangential phase", "[fluid][planar-reflection][quadrature]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    std::vector<Particle> p;
    for(int y=0;y<9;++y) for(int x=-4;x<=4;++x) p.push_back({{x*.1,(y+.5)*.1},{},10,1000,1000});
    Plane plane; const auto out=Evaluate(p,plane,.25,family);
    REQUIRE(out.forces[4].norm()<1e-12);
    auto shifted=p; for(auto& a:shifted) a.position.x+=.037;
    const auto phase=Evaluate(shifted,plane,.25,family);
    REQUIRE((phase.forces[4]-out.forces[4]).norm()<1e-12);
    REQUIRE(phase.summedDensities[4]==Catch::Approx(out.summedDensities[4]).epsilon(1e-13));
    for(auto& a:p) a.velocity=a.position*(-.1);
    const auto compression=Evaluate(p,plane,.25,family);
    const double expected=family==SphKernelFamily::CubicSpline?.2002092372038099:.18496156225689975;
    REQUIRE(compression.densityRates[4]/1000==Catch::Approx(expected).epsilon(1e-12));
    REQUIRE(compression.densityRates[4]>0);
}

TEST_CASE("Source ownership does not remove alternating row phase quadrature error", "[fluid][planar-reflection][quadrature]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    auto make=[](double dx) {
        std::vector<Particle> p;
        for(int y=0;y<9;++y) for(int x=-4;x<=4;++x)
            p.push_back({{(x+.5*(y%2))*dx,(y+.5)*dx},{},1000*dx*dx,1000,1000});
        return p;
    };
    Plane plane; const auto coarse=Evaluate(make(.1),plane,.25,family);
    const auto fine=Evaluate(make(.05),plane,.125,family);
    const double expected=family==SphKernelFamily::CubicSpline?-.023055152274654205:.0043488616243053;
    REQUIRE(coarse.forces[4].y/100==Catch::Approx(expected).epsilon(1e-11));
    REQUIRE(std::abs(coarse.forces[4].y)>0.01);
    REQUIRE(fine.forces[4].y/2.5==Catch::Approx(2*coarse.forces[4].y/10).epsilon(1e-12));
}
