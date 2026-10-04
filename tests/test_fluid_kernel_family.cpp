#include "catch_amalgamated.hpp"
#include "../benchmarks/fluid_consistency_metrics.h"
#include "physics/core/fluids/wcsph_solver.h"
#include "physics/core/fluids/dfsph_solver.h"
#include <cmath>
#include <limits>
using namespace PhysicsEngine;
namespace {
constexpr auto Cubic = SphKernelFamily::CubicSpline;
constexpr double Pi = 3.14159265358979323846;
std::vector<FluidParticle> Pair() {
    FluidParticleProperties p;
    p.mass=1; p.restDensity=1; p.smoothingLength=1; p.viscosity=0;
    return {FluidParticle({0,0},{0.1f,0},p),FluidParticle({0.5f,0},{-0.1f,0},p)};
}
}

TEST_CASE("Both fluid solvers and diagnostics use the selected cubic family", "[fluid][family]") {
    const double c=40/(7*Pi), weight=c*0.25, gradient=-1.5*c;
    auto p=Pair();
    WcsphConfig w; w.externalAcceleration={}; w.kernelFamily=Cubic;
    WcsphSolver wc(1,w); wc.prepare(p);
    REQUIRE(p[0].density == Catch::Approx(c+weight));
    REQUIRE(p[1].density == Catch::Approx(c+weight));
    const double expectedRate=0.2*(-gradient);
    REQUIRE(wc.getDiagnostics().maximumCompressionRate == Catch::Approx(expectedRate));
    const std::vector<FluidParticleSpatialGrid::ParticlePair> pairs{{0,1}};
    REQUIRE(MeasureFluidDiagnostics(p,pairs,Cubic).maximumCompressionRate == Catch::Approx(expectedRate));
    REQUIRE(MeasureFluidDiagnostics(p,pairs).maximumCompressionRate != Catch::Approx(expectedRate));
    DfsphConfig d; d.externalAcceleration={}; d.kernelFamily=Cubic;
    DfsphSolver df(1,d); p=Pair(); df.step(p,0);
    REQUIRE(p[0].density == Catch::Approx(c+weight));
    REQUIRE(df.getDiagnostics().maximumCompressionRate == Catch::Approx(expectedRate));
    w.densityMode=WcsphDensityMode::Continuity; w.densityDiffusion=0;
    p=Pair(); WcsphSolver continuity(1,w); continuity.prepare(p);
    REQUIRE(p[0].densityRate == Catch::Approx(expectedRate));
    REQUIRE(p[1].densityRate == Catch::Approx(expectedRate));
    for(auto& particle:p) particle.velocity={0.2f,0.3f};
    continuity.prepare(p);
    REQUIRE(p[0].densityRate == 0);
    REQUIRE(p[1].densityRate == 0);
}

TEST_CASE("Cubic WCSPH wall density pressure and mirror divergence share the same family", "[fluid][family][boundary]") {
    FluidParticleProperties properties; properties.mass=10; properties.smoothingLength=1; properties.viscosity=0;
    std::vector<FluidParticle> p{FluidParticle({0,0},{-0.1f,0},properties)};
    const std::vector<FluidBoundaryParticle> wall{{{-0.5f,0},{},0.2f,{},1}};
    const double c=40/(7*Pi), weight=c*0.25, gx=-1.5*c;
    WcsphConfig w; w.externalAcceleration={}; w.kernelFamily=Cubic;
    WcsphSolver summation(1,w); summation.prepare(p,wall);
    REQUIRE(p[0].density == Catch::Approx(10*c+1000*0.2*weight));
    w.densityMode=WcsphDensityMode::Continuity; w.densityDiffusion=0;
    p[0].density=1100;
    WcsphSolver continuity(1,w); continuity.prepare(p,wall);
    const double expectedRate=1100*0.2*2*(-0.1)*gx;
    REQUIRE(p[0].densityRate == Catch::Approx(expectedRate));
    REQUIRE(continuity.getDiagnostics().maximumCompressionRate == Catch::Approx(expectedRate/1000));
    REQUIRE(p[0].force.x == Catch::Approx(-p[0].volume*0.2*2*p[0].pressure*gx));
    REQUIRE(p[0].force.y == 0);
    REQUIRE(p[0].mass == 10);
    REQUIRE(p[0].restDensity == 1000);
}

TEST_CASE("Cubic compression retains EOS response and nominal input masses", "[fluid][family][compression]") {
    FluidParticleProperties properties; properties.mass=10; properties.smoothingLength=0.2f; properties.viscosity=0;
    std::vector<FluidParticle> p;
    for(int y=0;y<9;++y) for(int x=0;x<9;++x) p.emplace_back(Vector2{x*0.1f,y*0.1f},Vector2{},properties);
    WcsphConfig w; w.externalAcceleration={}; w.kernelFamily=Cubic; w.speedOfSound=15;
    WcsphSolver solver(0.2f,w); solver.prepare(p);
    const float density=p[40].density, pressure=p[40].pressure;
    REQUIRE(density/1000 == Catch::Approx(1.0008618328).margin(1e-6));
    for(auto& particle:p) particle.position=particle.position*0.98f;
    solver.prepare(p);
    REQUIRE(p[40].density > density*1.02f);
    REQUIRE(p[40].pressure > pressure);
    REQUIRE(p[40].mass == 10);
    REQUIRE(p[40].restDensity == 1000);
}

TEST_CASE("Kernel configs reject unknown families and legacy defaults agree exactly", "[fluid][family][validation]") {
    WcsphConfig w; DfsphConfig d;
    REQUIRE(w.kernelFamily == SphKernelFamily::Poly6Spiky);
    REQUIRE(d.kernelFamily == SphKernelFamily::Poly6Spiky);
    w.externalAcceleration={}; auto a=Pair(),b=a;
    WcsphSolver first(1,w); w.kernelFamily=SphKernelFamily::Poly6Spiky; WcsphSolver second(1,w);
    first.step(a,0.001f); second.step(b,0.001f);
    for(std::size_t i=0;i<a.size();++i) {
        REQUIRE(a[i].position == b[i].position); REQUIRE(a[i].velocity == b[i].velocity);
        REQUIRE(a[i].density == b[i].density); REQUIRE(a[i].pressure == b[i].pressure);
    }
    w.kernelFamily=static_cast<SphKernelFamily>(99); d.kernelFamily=w.kernelFamily;
    REQUIRE_THROWS_AS(WcsphSolver(1,w),std::invalid_argument);
    REQUIRE_THROWS_AS(DfsphSolver(1,d),std::invalid_argument);
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics(a,{},Cubic,{1}),std::invalid_argument);
}

TEST_CASE("Cubic central pressure preserves linear and angular momentum", "[fluid][family][momentum]") {
    const bool projected=GENERATE(false,true);
    auto p=Pair(); p[0].mass=1.3f; p[1].mass=0.7f;
    p[0].position={-0.2f,-0.1f}; p[1].position={0.2f,0.2f};
    p[0].velocity={0.03f,0.02f}; p[1].velocity={-0.01f,0.04f};
    const auto moment=[](const auto& particles) {
        double px=0,py=0,l=0;
        for(const auto& x:particles) {
            px+=x.mass*static_cast<double>(x.velocity.x);
            py+=x.mass*static_cast<double>(x.velocity.y);
            l+=x.mass*(static_cast<double>(x.position.x)*x.velocity.y-static_cast<double>(x.position.y)*x.velocity.x);
        }
        return std::vector<double>{px,py,l};
    };
    const auto before=moment(p);
    if(projected) {
        DfsphConfig config; config.kernelFamily=Cubic; config.externalAcceleration={};
        DfsphSolver solver(1,config); solver.step(p,0.001f);
    } else {
        WcsphConfig config; config.kernelFamily=Cubic; config.externalAcceleration={}; config.speedOfSound=1;
        WcsphSolver solver(1,config); solver.step(p,0.001f);
    }
    const auto after=moment(p);
    for(std::size_t i=0;i<before.size();++i) REQUIRE(after[i] == Catch::Approx(before[i]).margin(0.0001));
    REQUIRE(p[0].mass == 1.3f); REQUIRE(p[1].mass == 0.7f);
    REQUIRE(p[0].restDensity == 1); REQUIRE(p[1].restDensity == 1);
}

TEST_CASE("Cubic viscosity remains stable and dissipative under the independent Muller bound", "[fluid][family][viscosity]") {
    const bool projected=GENERATE(false,true);
    auto p=Pair();
    for(auto& x:p) { x.mass=0.001f; x.restDensity=1; x.density=1; x.viscosity=1; }
    p[0].velocity={0,1}; p[1].velocity={0,-1};
    const auto energy=[](const auto& particles) {
        double e=0; for(const auto& x:particles) e+=0.5*x.mass*(static_cast<double>(x.velocity.x)*x.velocity.x+static_cast<double>(x.velocity.y)*x.velocity.y); return e;
    };
    const double before=energy(p);
    FluidDiagnostics diagnostics;
    if(projected) {
        DfsphConfig c; c.kernelFamily=Cubic; c.externalAcceleration={};
        DfsphSolver s(1,c); s.step(p,0.001f); diagnostics=s.getDiagnostics();
    } else {
        WcsphConfig c; c.kernelFamily=Cubic; c.externalAcceleration={}; c.speedOfSound=0.1f;
        WcsphSolver s(1,c); s.step(p,0.001f); diagnostics=s.getDiagnostics();
    }
    REQUIRE(energy(p) < before); REQUIRE(diagnostics.substeps > 1);
    REQUIRE(p[0].pressure == 0); REQUIRE(p[1].pressure == 0);
    REQUIRE((p[0].velocity+p[1].velocity).magnitude() < 1e-6f);
}

TEST_CASE("Production cubic lattice moments reproduce the independent candidate", "[fluid][family][moments]") {
    for(float ratio:{1.5f,2.0f,2.5f,4.0f,8.0f}) {
        const auto m=FluidConsistency::MeasureKernelMoments(0.1f,ratio*0.1f);
        REQUIRE(m.cubicDensity == Catch::Approx(m.candidateCubicDensity).margin(2e-7));
        REQUIRE(m.cubicGradientXX == Catch::Approx(m.candidateCubicGradientXX).margin(2e-7));
    }
}

TEST_CASE("Nominal mass cubic rest blocks meet unchanged acceptance under scale and dt refinement", "[fluid][family][rest]") {
    const float dx=GENERATE(0.1f,0.05f,0.025f);
    const float dt=GENERATE(1.0f/240,1.0f/480,1.0f/960);
    FluidParticleProperties properties; properties.mass=1000*dx*dx; properties.smoothingLength=2*dx; properties.viscosity=0.05f;
    std::vector<FluidParticle> p;
    for(int y=0;y<21;++y) for(int x=0;x<21;++x) p.emplace_back(Vector2{x*dx,y*dx},Vector2{},properties);
    const auto initial=p;
    WcsphConfig c; c.kernelFamily=Cubic; c.externalAcceleration={}; c.speedOfSound=15; c.maximumTimeStep=dt;
    WcsphSolver solver(2*dx,c); solver.prepare(p);
    REQUIRE(std::abs(p[220].density/1000-1) < 0.01f);
    const int steps=static_cast<int>(std::lround(0.1f/dt));
    for(int step=0;step<steps;++step) solver.step(p,dt);
    double speed=0,displacement=0,px=0,py=0,angular=0;
    for(std::size_t i=0;i<p.size();++i) {
        speed=std::max(speed,std::hypot(static_cast<double>(p[i].velocity.x),p[i].velocity.y));
        displacement=std::max(displacement,std::hypot(static_cast<double>(p[i].position.x)-initial[i].position.x,
            static_cast<double>(p[i].position.y)-initial[i].position.y));
        px+=p[i].mass*static_cast<double>(p[i].velocity.x); py+=p[i].mass*static_cast<double>(p[i].velocity.y);
        angular+=p[i].mass*(static_cast<double>(p[i].position.x)*p[i].velocity.y-static_cast<double>(p[i].position.y)*p[i].velocity.x);
        REQUIRE(p[i].mass == initial[i].mass); REQUIRE(p[i].restDensity == initial[i].restDensity);
    }
    INFO("dx=" << dx << " dt=" << dt << " speed=" << speed << " displacement=" << displacement);
    REQUIRE(speed < 0.05); REQUIRE(displacement < 0.005); REQUIRE(std::hypot(px,py) < 1e-3);
    REQUIRE(std::abs(angular) < 1e-3);
}

TEST_CASE("Cubic gradient has continuous radial derivatives at branch and support", "[fluid][family][smoothness]") {
    for(float h:{0.2f,1.0f,2.0f}) {
        const float epsilon=h*0.0002f;
        const double normalization=40/(7*Pi*static_cast<double>(h)*h);
        const auto g=[h](float r) { return SphKernels2D::PressureGradient({r,0},h,Cubic).x; };
        for(float branch:{0.5f,1.0f}) {
            const float r=branch*h;
            const double left=(g(r)-g(r-epsilon))/epsilon*h*h/normalization;
            const double right=(g(r+epsilon)-g(r))/epsilon*h*h/normalization;
            const double expected=branch==0.5f ? 6 : 0;
            REQUIRE(left == Catch::Approx(expected).margin(0.01));
            REQUIRE(right == Catch::Approx(expected).margin(0.01));
        }
    }
    const float h=1e20f;
    const double area=static_cast<double>(h)*h;
    REQUIRE(SphKernels2D::DensityWeight({h*0.5f,0},h,Cubic)*area ==
        Catch::Approx(10/(7*Pi)).margin(2e-5));
}
