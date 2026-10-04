#include "catch_amalgamated.hpp"
#include "physics/core/fluids/dfsph_solver.h"
#include "physics/core/fluids/wcsph_solver.h"
#include "../benchmarks/fluid_disorder_operator.h"
#include "../benchmarks/planar_reflected_operator.h"
#include <cmath>
#include <limits>

using namespace PhysicsEngine;
namespace {
constexpr auto Wendland = SphKernelFamily::WendlandC2;
constexpr double Pi = 3.1415926535897932384626433832795;
// Expanded polynomial oracle, independent of the factored production shape.
double Polynomial(double q) {
    return q>=1 ? 0 : 1+q*q*(-10+q*(20+q*(-15+4*q)));
}
std::vector<FluidParticle> Pair() {
    FluidParticleProperties p;
    p.mass=1;p.restDensity=1;p.smoothingLength=1;p.viscosity=0;
    return {FluidParticle({0,0},{.1f,0},p),FluidParticle({.5f,0},{-.1f,0},p)};
}
}

TEST_CASE("Wendland C2 has the normalized 2D weight moment and matched derivative",
          "[fluid][sph][kernel][wendland]") {
    for(float h:{.25f,1.f,4.f}) {
        double integral=0,secondMoment=0;
        constexpr int bins=20000;
        for(int i=0;i<bins;++i) {
            const float r=float((i+.5)*h/bins);
            const double shell=2*Pi*r*SphKernels2D::DensityWeight({r,0},h,Wendland)*h/bins;
            integral+=shell;secondMoment+=shell*r*r;
        }
        REQUIRE(integral==Catch::Approx(1).epsilon(0).margin(2e-7));
        REQUIRE(secondMoment/(double(h)*h)==Catch::Approx(5./36).epsilon(0).margin(3e-8));
        for(float ratio:{0.f,.125f,.25f,.5f,.75f,.875f,1.f,1.125f}) {
            const float r=ratio*h;
            const double q=double(r)/h,c=7/(Pi*h*h);
            const double derivative=q>=1?0:c/h*q*(-20+q*(60+q*(-60+20*q)));
            const auto w=SphKernels2D::DensityWeight({r,0},h,Wendland);
            REQUIRE(w==Catch::Approx(c*Polynomial(q)).epsilon(2e-7));
            REQUIRE(SphKernels2D::PressureWeight({r,0},h,Wendland)==w);
            REQUIRE(SphKernels2D::PressureGradient({r,0},h,Wendland).x==
                    Catch::Approx(derivative).epsilon(2e-7));
            if(q>0 && q<1) {
                const float delta=h/4096;
                const double difference=(double(SphKernels2D::DensityWeight({r+delta,0},h,Wendland))-
                    SphKernels2D::DensityWeight({r-delta,0},h,Wendland))/(2*delta);
                REQUIRE(difference==Catch::Approx(derivative).epsilon(.0002));
            }
        }
    }
}

TEST_CASE("Wendland scale calibration support symmetry and checked range agree",
          "[fluid][sph][kernel][wendland]") {
    const Vector2 r{.3f,-.4f};
    const double w=SphKernels2D::DensityWeight(r,1,Wendland);
    const auto g=SphKernels2D::PressureGradient(r,1,Wendland);
    REQUIRE(SphKernels2D::DensityWeight(r*-1,1,Wendland)==w);
    REQUIRE(SphKernels2D::PressureGradient(r*-1,1,Wendland)==g*-1);
    for(float scale:{1e-10f,.25f,2.f,1e10f}) {
        const double area=double(scale)*scale;
        const auto gs=SphKernels2D::PressureGradient(r*scale,scale,Wendland);
        REQUIRE(SphKernels2D::DensityWeight(r*scale,scale,Wendland)*area==Catch::Approx(w));
        REQUIRE(gs.x*area*scale==Catch::Approx(g.x));
        REQUIRE(gs.y*area*scale==Catch::Approx(g.y));
    }
    const double latticeSum=7/(4*Pi)*(1+4*Polynomial(.5)+4*Polynomial(std::sqrt(.5)));
    for(float dx:{1e-30f,1e-15f,1.f,1e15f,1e30f})
        REQUIRE(SphKernels2D::SquareLatticeMassScale(dx,2*dx,Wendland)==Catch::Approx(1/latticeSum));
    REQUIRE(SphKernels2D::PressureGradient({},1e-20f,Wendland)==Vector2{});
    REQUIRE_THROWS_AS(SphKernels2D::DensityWeight({},1e-20f,Wendland),std::overflow_error);
    REQUIRE_THROWS_AS(SphKernels2D::PressureGradient({.5e-20f,0},1e-20f,Wendland),std::overflow_error);
    const auto tiny=SphKernels2D::PressureGradient({1e-30f,0},1e-10f,Wendland);
    REQUIRE(tiny.x==Catch::Approx(-140/Pi*1e10).epsilon(1e-6));
    REQUIRE(tiny.y==0);
    const float big=std::numeric_limits<float>::max(),nan=std::numeric_limits<float>::quiet_NaN();
    REQUIRE(SphKernels2D::DensityWeight({big,big},1,Wendland)==0);
    REQUIRE(SphKernels2D::PressureGradient({big,big},1,Wendland)==Vector2{});
    REQUIRE_THROWS_AS(SphKernels2D::DensityWeight({nan,0},1,Wendland),std::invalid_argument);
    REQUIRE_THROWS_AS(SphKernels2D::PressureWeight({},0,Wendland),std::invalid_argument);
    REQUIRE_THROWS_AS(SphKernels2D::SquareLatticeMassScale(1,1e6f,Wendland),std::length_error);
}

TEST_CASE("Wendland selection reaches both solvers diagnostics and continuity",
          "[fluid][family][wendland]") {
    const double density=133/(16*Pi),rate=7/(4*Pi);
    auto p=Pair();WcsphConfig wc;wc.externalAcceleration={};wc.kernelFamily=Wendland;
    WcsphSolver w(1,wc);w.prepare(p);
    REQUIRE(p[0].density==Catch::Approx(density));
    REQUIRE(p[1].density==Catch::Approx(density));
    REQUIRE(w.getDiagnostics().maximumCompressionRate==Catch::Approx(rate));
    REQUIRE(MeasureFluidDiagnostics(p,{{0,1}},Wendland).maximumCompressionRate==Catch::Approx(rate));
    DfsphConfig dc;dc.externalAcceleration={};dc.kernelFamily=Wendland;
    DfsphSolver d(1,dc);p=Pair();d.step(p,0);
    REQUIRE(p[0].density==Catch::Approx(density));
    REQUIRE(d.getDiagnostics().maximumCompressionRate==Catch::Approx(rate));
    wc.densityMode=WcsphDensityMode::Continuity;wc.densityDiffusion=0;
    WcsphSolver continuity(1,wc);p=Pair();continuity.prepare(p);
    REQUIRE(p[0].densityRate==Catch::Approx(rate));
    REQUIRE(p[1].densityRate==Catch::Approx(rate));
    for(auto& a:p)a.velocity={.2f,.3f};
    continuity.prepare(p);REQUIRE(p[0].densityRate==0);REQUIRE(p[1].densityRate==0);
}

TEST_CASE("Wendland sampled walls use matched density pressure and mirror rates",
          "[fluid][boundary][wendland]") {
    FluidParticleProperties properties;properties.mass=10;properties.smoothingLength=1;properties.viscosity=0;
    std::vector<FluidParticle> p{FluidParticle({0,0},{-.1f,0},properties)};
    const std::vector<FluidBoundaryParticle> wall{{{-.5f,0},{},.2f,{},1}};
    const double c=7/Pi,weight=21/(16*Pi),gradient=-35/(4*Pi);
    WcsphConfig config;config.externalAcceleration={};config.kernelFamily=Wendland;
    WcsphSolver summation(1,config);summation.prepare(p,wall);
    REQUIRE(p[0].density==Catch::Approx(10*c+1000*.2*weight));
    config.densityMode=WcsphDensityMode::Continuity;config.densityDiffusion=0;p[0].density=1100;
    WcsphSolver continuity(1,config);continuity.prepare(p,wall);
    const double rate=1100*.2*2*(-.1)*gradient;
    REQUIRE(p[0].densityRate==Catch::Approx(rate));
    REQUIRE(continuity.getDiagnostics().maximumCompressionRate==Catch::Approx(rate/1000));
    REQUIRE(p[0].force.x==Catch::Approx(-p[0].volume*.2*2*p[0].pressure*gradient));
    REQUIRE(p[0].force.y==0);REQUIRE(p[0].mass==10);REQUIRE(p[0].restDensity==1000);
}

TEST_CASE("Wendland retains the checkerboard linear null mode and independent pressure work",
          "[fluid][disorder][wendland]") {
    using namespace FluidDisorder;
    for(double ratio:{2.,2.5,4.,8.}) {
        const auto row=LatticeRow(.1,.1*ratio,.02,Wendland);
        REQUIRE(row.linearSymbol.norm()<2e-12);
        REQUIRE(row.shifted==Catch::Approx(LatticeRow(1,ratio,.02,Wendland).shifted).epsilon(0).margin(4e-15));
    }
    auto p=Block(9,.1f,.2f,1);
    for(auto& a:p) {
        a.position=a.position*.96f;a.viscosity=0;
        a.velocity={float(.3*a.position.x+.1*std::sin(4*a.position.y)),
                    float(-.2*a.position.y+.07*std::cos(3*a.position.x))};
    }
    const auto s=FromParticles(p,Wendland);const auto op=Build(s);
    REQUIRE(std::abs(op.mechanicalWork+op.trueEnergyRate)<2e-12*std::abs(op.mechanicalWork));
    const double e1=std::abs(EnergyDifference(s,.001)-op.trueEnergyRate);
    const double e2=std::abs(EnergyDifference(s,.0005)-op.trueEnergyRate);
    const double e3=std::abs(EnergyDifference(s,.00025)-op.trueEnergyRate);
    REQUIRE(e1/e2>3.8);REQUIRE(e1/e2<4.2);REQUIRE(e2/e3>3.8);REQUIRE(e2/e3<4.2);
    WcsphConfig c;c.externalAcceleration={};c.kernelFamily=Wendland;c.speedOfSound=15;
    WcsphSolver solver(.2f,c);solver.prepare(p);
    double scale=0,error=0;
    for(std::size_t i=0;i<p.size();++i) {
        scale=std::max(scale,op.forces[i].norm());
        error=std::max(error,(op.forces[i]-D2{p[i].force.x,p[i].force.y}).norm());
    }
    REQUIRE(error<2e-5*scale);
    // The reflected-source prototype has independent formulas for two families
    // only. Never silently interpret a newly recognized enum as cubic.
    REQUIRE_THROWS_AS(PlanarReflection::Weight({.25,0},1,Wendland),std::invalid_argument);
    REQUIRE_THROWS_AS(PlanarReflection::Gradient({},1,Wendland),std::invalid_argument);
    REQUIRE_THROWS_AS(PlanarReflection::Evaluate({}, {}, 1,Wendland),std::invalid_argument);
}
