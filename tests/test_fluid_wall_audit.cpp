#include "catch_amalgamated.hpp"
#include "../benchmarks/fluid_wall_audit.h"
#include "physics/core/fluids/wcsph_solver.h"
using namespace PhysicsEngine;
using namespace FluidWallAudit;
namespace {
constexpr double Pi=3.14159265358979323846;
std::vector<FluidParticle> One(Vector2 position,Vector2 velocity={}) {
    FluidParticleProperties properties; properties.mass=10; properties.smoothingLength=1; properties.viscosity=0;
    std::vector<FluidParticle> p{FluidParticle(position,velocity,properties)};
    p[0].density=1000;
    return p;
}
WcsphConfig Config(SphKernelFamily family) {
    WcsphConfig config; config.kernelFamily=family; config.externalAcceleration={};
    config.densityMode=WcsphDensityMode::Continuity; config.densityDiffusion=0;
    return config;
}
}

TEST_CASE("Radial wall continuity matches a normal aligned analytic mirror", "[fluid][wall-audit][oracle]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    const double gradient=family==SphKernelFamily::CubicSpline ? -60/(7*Pi) : -7.5/Pi;
    auto p=One({0,0.5f},{0,-0.2f});
    const std::vector<FluidBoundaryParticle> wall{{{0,0},{},0.2f,{},1}};
    auto config=Config(family); WcsphSolver solver(1,config); solver.prepare(p,wall);
    const double rate=2*1000*0.2*(-0.2)*gradient;
    REQUIRE(p[0].densityRate == Catch::Approx(rate));
    const auto op=Build(p,wall,config); const auto work=MeasureWork(op,p,wall);
    REQUIRE(work.densityRates[0] == Catch::Approx(rate));
    REQUIRE(work.wallWorkResidual == 0); // rho0 gives zero pressure.
}

TEST_CASE("Oblique radial mirror is not reflection about a flat plane normal", "[fluid][wall-audit][oracle]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    const double derivative=family==SphKernelFamily::CubicSpline ? -60/(7*Pi) : -7.5/Pi;
    auto p=One({0.3f,0.4f},{0.1f,0}); // Plane y=0: strictly tangential motion.
    const std::vector<FluidBoundaryParticle> wall{{{0,0},{},0.2f,{},1}};
    auto config=Config(family); WcsphSolver solver(1,config); solver.prepare(p,wall);
    const double radialRate=2*1000*0.2*0.1*derivative*0.6;
    REQUIRE(p[0].densityRate == Catch::Approx(radialRate));
    REQUIRE(std::abs(p[0].densityRate) > 50);
    // Plane-normal reflected velocity would be unchanged, so its rate is zero.
    const Vector2 planeNormal{0,1};
    REQUIRE(p[0].velocity.dot(planeNormal) == 0);
}

TEST_CASE("Equal pressure radial wall work closes with an inferred moving reaction", "[fluid][wall-audit][work]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    auto p=One({0.3f,0.4f},{0.1f,-0.2f}); p[0].density=1010;
    std::vector<FluidBoundaryParticle> wall{{{0,0},{-0.1f,0.2f},0.2f,{},1}};
    auto config=Config(family); WcsphSolver solver(1,config); solver.prepare(p,wall);
    const auto op=Build(p,wall,config); const auto work=MeasureWork(op,p,wall);
    REQUIRE(work.wallInternalEnergyRate != 0);
    REQUIRE(work.inferredReactionWork != 0);
    REQUIRE(work.wallWorkResidual == Catch::Approx(0).margin(1e-10));
    REQUIRE(work.wallRelativeMechanicalWork == Catch::Approx(work.wallFluidWork+work.inferredReactionWork));
    REQUIRE(ToDouble(p[0].force).x == Catch::Approx(op.wallForces[0].x));
    REQUIRE(ToDouble(p[0].force).y == Catch::Approx(op.wallForces[0].y));
    REQUIRE(p[0].densityRate == Catch::Approx(work.densityRates[0]));
}

TEST_CASE("Gravity extrapolated wall pressure has the independently derived work residual", "[fluid][wall-audit][work]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    auto p=One({0,0.5f},{0,-0.2f}); p[0].density=1010;
    std::vector<FluidBoundaryParticle> wall{{{0,0},{},0.2f,{},1}};
    auto config=Config(family); config.externalAcceleration={0,-9.81f};
    WcsphSolver solver(1,config); solver.prepare(p,wall);
    const auto op=Build(p,wall,config); const auto work=MeasureWork(op,p,wall);
    const double wallPressure=p[0].pressure+p[0].density*0.5*9.81f;
    const double derivative=family==SphKernelFamily::CubicSpline ? -60/(7*Pi) : -7.5/Pi;
    const double expected=(p[0].mass/static_cast<double>(p[0].density))*0.2*(p[0].pressure-wallPressure)*(-0.2)*derivative;
    REQUIRE(op.walls[0].pressure == Catch::Approx(wallPressure));
    REQUIRE(work.wallWorkResidual == Catch::Approx(expected));
    REQUIRE(work.wallWorkResidual < 0);
    REQUIRE((ToDouble(p[0].force)-op.wallForces[0]-op.gravityForces[0]).norm() < 1e-4);
    // A virtual-boundary energy reservoir is untracked: this is a model
    // accounting residual, not a proof of unstable time integration.
}

TEST_CASE("Fluid pressure pair work and translation close independently of wall terms", "[fluid][wall-audit][work]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    auto p=One({0,0},{0.1f,0.2f}); auto second=One({0.3f,0.4f},{-0.2f,0.1f})[0];
    second.mass=15; second.density=1020; p[0].density=1010; p.push_back(second);
    auto config=Config(family); WcsphSolver solver(1,config); solver.prepare(p);
    auto op=Build(p,{},config); auto work=MeasureWork(op,p,{});
    REQUIRE(work.pairInternalEnergyRate != 0);
    REQUIRE(work.pairWorkResidual == Catch::Approx(0).margin(1e-10));
    REQUIRE((op.fluidForces[0]+op.fluidForces[1]).norm() == 0);
    REQUIRE(p[0].densityRate == Catch::Approx(work.densityRates[0]));
    REQUIRE(p[1].densityRate == Catch::Approx(work.densityRates[1]));
    for(auto& particle:p) particle.velocity={0.1f,0.2f};
    work=MeasureWork(op,p,{}); REQUIRE(work.maximumAbsoluteDensityRate == 0);
}

TEST_CASE("Tait hydrostatic EOS profile obeys body force balance", "[fluid][wall-audit][hydrostatic]") {
    for(double depth:{0.0,0.2,0.7,1.5}) {
        const double ratio=HydrostaticDensityRatio(depth);
        REQUIRE(HydrostaticPressure(depth) == Catch::Approx(1000*40*40/7*(std::pow(ratio,7)-1)));
        if(depth>0) {
            const double epsilon=1e-5;
            const double derivative=(HydrostaticPressure(depth+epsilon)-HydrostaticPressure(depth-epsilon))/(2*epsilon);
            REQUIRE(derivative == Catch::Approx(1000*ratio*9.81).epsilon(1e-6));
        }
    }
}

TEST_CASE("Symmetric oblique samples cancel aggregate flat wall tangent response", "[fluid][wall-audit][oracle]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    auto p=One({0,0.4f},{0.2f,0}); p[0].density=1010;
    const std::vector<FluidBoundaryParticle> wall{{{-0.3f,0},{},0.2f,{},1},{{0.3f,0},{},0.2f,{},1}};
    auto config=Config(family); WcsphSolver solver(1,config); solver.prepare(p,wall);
    REQUIRE(p[0].densityRate == 0); REQUIRE(p[0].force.x == 0);
    const auto work=MeasureWork(Build(p,wall,config),p,wall);
    REQUIRE(work.maximumAbsoluteDensityRate == 0);
    REQUIRE(work.wallRelativeMechanicalWork == 0);
}

TEST_CASE("Radial mirror annihilates joint rigid rotation", "[fluid][wall-audit][oracle]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    auto p=One({0.3f,0.4f},{-0.04f,0.03f}); p[0].density=1010;
    const std::vector<FluidBoundaryParticle> wall{{{0,0},{},0.2f,{},1}};
    auto config=Config(family); WcsphSolver solver(1,config); solver.prepare(p,wall);
    REQUIRE(std::abs(p[0].densityRate) < 2e-6);
    REQUIRE(MeasureWork(Build(p,wall,config),p,wall).maximumAbsoluteDensityRate < 2e-9);
}

TEST_CASE("Pressure scale changes force without scaling the current mirror density rate", "[fluid][wall-audit][work]") {
    auto p=One({0,0.5f},{0,-0.2f});p[0].density=1010;
    const std::vector<FluidBoundaryParticle> wall{{{0,0},{},0.2f,{},0.5f}};
    auto config=Config(SphKernelFamily::CubicSpline);WcsphSolver solver(1,config);solver.prepare(p,wall);
    const auto op=Build(p,wall,config);const auto work=MeasureWork(op,p,wall);
    REQUIRE(work.wallWorkResidual == Catch::Approx(work.wallInternalEnergyRate*0.5));
    const double adjointWork=op.localAdjointWallForces[0].dot(ToDouble(p[0].velocity));
    REQUIRE(adjointWork+work.wallInternalEnergyRate == Catch::Approx(0).margin(1e-10));
}

TEST_CASE("Signed hydrostatic extrapolation reproduces an affine pressure across a side sample", "[fluid][wall-audit][oracle]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    auto p=One({0.3f,0.4f});p[0].density=1010;
    const std::vector<FluidBoundaryParticle> wall{{{0,0.5f},{},0.2f,{},1}};
    auto config=Config(family);config.externalAcceleration={0,-9.81f};
    WcsphSolver solver(1,config);solver.prepare(p,wall);const auto op=Build(p,wall,config);
    const D2 displacement=ToDouble(p[0].position)-ToDouble(wall[0].position);
    const double expectedPressure=p[0].pressure-p[0].density*ToDouble(config.externalAcceleration).dot(displacement);
    REQUIRE(expectedPressure > 0);
    REQUIRE(op.walls[0].pressure == Catch::Approx(p[0].pressure)); // Current one-sided clamp.
    const double volume=p[0].mass/static_cast<double>(p[0].density);
    const D2 expectedForce=op.walls[0].gradient*(-volume*wall[0].volume*(p[0].pressure+expectedPressure));
    REQUIRE(op.signedExtrapolationWallForces[0].x == Catch::Approx(expectedForce.x));
    REQUIRE(op.signedExtrapolationWallForces[0].y == Catch::Approx(expectedForce.y));
    REQUIRE(op.signedExtrapolationWallForces[0].norm() < op.wallForces[0].norm());
}

TEST_CASE("Aligned flat wall samples cancel constant pressure while midpoint staggering retains a defect", "[fluid][wall-audit][quadrature]") {
    const auto family=GENERATE(SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline);
    const auto aligned=FlatPhaseProbe(family,1,2.5f,false),staggered=FlatPhaseProbe(family,1,2.5f,true);
    REQUIRE(std::abs(aligned.normalizedNormalForce) < 1e-6);
    REQUIRE(std::abs(staggered.normalizedNormalForce) > 1e-4);
    const auto refined=FlatPhaseProbe(family,0.5f,2.5f,true);
    REQUIRE(refined.normalizedNormalForce == Catch::Approx(staggered.normalizedNormalForce).margin(1e-6));
    REQUIRE(refined.normalAcceleration == Catch::Approx(2*staggered.normalAcceleration).margin(1e-4));
}

TEST_CASE("Wall audit rejects configurations beyond its diagnostic work bounds", "[fluid][wall-audit][validation]") {
    auto config=Config(SphKernelFamily::CubicSpline);
    REQUIRE_THROWS_AS(Build({},std::vector<FluidBoundaryParticle>(10001),config),std::length_error);
    REQUIRE_THROWS_AS(FlatPhaseProbe(SphKernelFamily::CubicSpline,0,2.5f,false),std::invalid_argument);
    config.kernelFamily=static_cast<SphKernelFamily>(99);
    REQUIRE_THROWS_AS(Build({}, {},config),std::invalid_argument);
}
