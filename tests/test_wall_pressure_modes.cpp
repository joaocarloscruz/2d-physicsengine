#include "catch_amalgamated.hpp"
#include "physics/core/fluids/wcsph_solver.h"
#include "physics/core/fluids/sph_kernels.h"
#include <cmath>
#include <limits>

using namespace PhysicsEngine;
namespace {
WcsphConfig Config(SphKernelFamily family) {
    WcsphConfig c;
    c.kernelFamily = family; c.densityMode = WcsphDensityMode::Continuity;
    c.densityDiffusion = 0; c.externalAcceleration = {};
    c.wallPressureMode = WcsphWallPressureMode::SignedBodyForce;
    return c;
}
FluidParticle Particle(Vector2 x, float mass = 1) {
    FluidParticleProperties props; props.mass = mass; props.viscosity = 0; props.smoothingLength = 1;
    FluidParticle p(x, {}, props); p.density = 1010; return p;
}
double Dot(Vector2 a, Vector2 b) { return double(a.x)*b.x + double(a.y)*b.y; }
void CheckForce(Vector2 actual, double x, double y) {
    REQUIRE(std::isfinite(actual.x)); REQUIRE(std::isfinite(actual.y));
    REQUIRE(actual.x == Catch::Approx(x).epsilon(2e-5).margin(2e-4));
    REQUIRE(actual.y == Catch::Approx(y).epsilon(2e-5).margin(2e-4));
}
double InferWallPressure(const FluidParticle& p, const FluidBoundaryParticle& b, const WcsphConfig& c) {
    const auto gradient = SphKernels2D::PressureGradient(p.position-b.position, p.smoothingLength, c.kernelFamily);
    const double forceDot = (double(p.force.x)-double(p.mass)*c.externalAcceleration.x)*gradient.x
        + (double(p.force.y)-double(p.mass)*c.externalAcceleration.y)*gradient.y;
    return -forceDot/(double(p.volume)*b.volume*Dot(gradient,gradient))-p.pressure;
}
}

TEST_CASE("Signed wall pressure retains both body-force signs and frame acceleration", "[fluid][wall-pressure]") {
    const auto family = GENERATE(SphKernelFamily::Poly6Spiky, SphKernelFamily::CubicSpline);
    const bool rotated = GENERATE(false, true);
    const float sign = GENERATE(-1.0f, 1.0f);
    const auto rotate = [rotated](Vector2 p) { return rotated ? Vector2{-p.y,p.x} : p; };
    auto c = Config(family); c.externalAcceleration = rotate({2,-9});
    std::vector<FluidParticle> p{Particle(rotate({0.3f,sign*0.1f}))};
    std::vector<FluidBoundaryParticle> b{{{}, {}, 0.2f, rotate({2,1}), 1}};
    WcsphSolver solver(1,c); solver.prepare(p,b);
    // grad(p)=rho*(g-a_wall): evaluate the local affine field at the wall.
    const double expected = p[0].pressure + double(p[0].density)*10*sign*0.1f;
    REQUIRE(expected > 0);
    REQUIRE(InferWallPressure(p[0],b[0],c) == Catch::Approx(expected).epsilon(2e-6));
    REQUIRE(solver.getConfig().wallPressureMode == WcsphWallPressureMode::SignedBodyForce);
    c.wallPressureMode = WcsphWallPressureMode::LegacyPositiveIncrement;
    WcsphSolver legacy(1,c); legacy.prepare(p,b);
    REQUIRE(InferWallPressure(p[0],b[0],c) == Catch::Approx(p[0].pressure + (sign>0 ? double(p[0].density) : 0)).epsilon(2e-6));
}

TEST_CASE("Signed wall pressure clamps after weighted extrapolation", "[fluid][wall-pressure]") {
    const auto family = GENERATE(SphKernelFamily::Poly6Spiky, SphKernelFamily::CubicSpline);
    auto c = Config(family);
    std::vector<FluidParticle> p{Particle({0.2f,0}),Particle({-0.4f,0})};
    WcsphSolver initial(1,c); initial.prepare(p);
    const double w0 = SphKernels2D::DensityWeight(p[0].position,1,family);
    const double w1 = SphKernels2D::DensityWeight(p[1].position,1,family);
    const double meanX = (w0*p[0].position.x+w1*p[1].position.x)/(w0+w1);
    c.externalAcceleration = {float(p[0].pressure/(2*p[0].density*meanX)),0};
    const double raw0 = p[0].pressure-double(p[0].density)*c.externalAcceleration.x*p[0].position.x;
    const double raw1 = p[1].pressure-double(p[1].density)*c.externalAcceleration.x*p[1].position.x;
    const double expectedWall = (w0*raw0+w1*raw1)/(w0+w1);
    REQUIRE(raw0 > 0); REQUIRE(raw1 < 0); REQUIRE(expectedWall > 0);
    REQUIRE(expectedWall == Catch::Approx(p[0].pressure/2).epsilon(2e-6));
    std::vector<FluidBoundaryParticle> b{{{}, {}, 0.2f, {}, 1}};
    auto reference = p;
    WcsphSolver solver(1,c); solver.prepare(reference); solver.prepare(p,b);
    for (std::size_t i=0;i<p.size();++i) {
        const auto gradient = SphKernels2D::PressureGradient(p[i].position,1,family);
        const double scale = -double(p[i].volume)*b[0].volume*(p[i].pressure+expectedWall);
        CheckForce(p[i].force, reference[i].force.x+scale*gradient.x, reference[i].force.y+scale*gradient.y);
    }
}

TEST_CASE("Signed wall pressure uses the selected negative-pressure policy", "[fluid][wall-pressure]") {
    const bool clamp = GENERATE(false,true);
    auto c = Config(SphKernelFamily::CubicSpline); c.clampNegativePressure=clamp; c.externalAcceleration={0,100};
    std::vector<FluidParticle> p{Particle({0,0.5f})};
    std::vector<FluidBoundaryParticle> b{{{}, {}, 0.2f, {}, 1}};
    WcsphSolver solver(1,c); solver.prepare(p,b);
    const double raw = p[0].pressure-50.0*p[0].density;
    REQUIRE(raw < 0);
    const auto gradient = SphKernels2D::PressureGradient(p[0].position,1,c.kernelFamily);
    const double scale = -double(p[0].volume)*b[0].volume*(p[0].pressure+(clamp ? 0.0 : raw));
    // Compare the forward force: inferring zero wall pressure would subtract
    // rounded gravity/pressure forces, amplifying float storage error.
    CheckForce(p[0].force,gradient.x*scale,double(p[0].mass)*100+gradient.y*scale);
}

TEST_CASE("Signed pressure keeps large intermediate pressures when stored force is finite", "[fluid][wall-pressure][numerics]") {
    auto c = Config(SphKernelFamily::CubicSpline);
    std::vector<FluidParticle> p{Particle({0,0.5f},1e-5f)};
    std::vector<FluidBoundaryParticle> b{{{}, {}, 1e-4f, {0,3e38f}, 1}};
    WcsphSolver solver(1,c); solver.prepare(p,b);
    const double wallPressure = p[0].pressure+double(p[0].density)*b[0].acceleration.y*0.5;
    REQUIRE(wallPressure > std::numeric_limits<float>::max());
    const auto gradient = SphKernels2D::PressureGradient(p[0].position,1,c.kernelFamily);
    const double scale = -double(p[0].volume)*b[0].volume*(p[0].pressure+wallPressure);
    CheckForce(p[0].force,gradient.x*scale,gradient.y*scale);
    // The actual stored force, unlike the intermediate pressure, cannot exceed float.
    b[0].volume=1e10f;
    REQUIRE_THROWS_AS(solver.prepare(p,b),std::overflow_error);
    REQUIRE(std::isfinite(p[0].force.x)); REQUIRE(std::isfinite(p[0].force.y));
}

TEST_CASE("Wall pressure selection validates configuration and inactive boundaries", "[fluid][wall-pressure][validation]") {
    WcsphConfig defaults;
    REQUIRE(defaults.wallPressureMode == WcsphWallPressureMode::LegacyPositiveIncrement);
    auto c = Config(SphKernelFamily::CubicSpline);
    std::vector<FluidParticle> empty;
    WcsphSolver solver(1,c); REQUIRE_NOTHROW(solver.prepare(empty,{}));
    std::vector<FluidParticle> p{Particle({0,0.5f})};
    solver.prepare(p); const auto before = p[0].force;
    solver.prepare(p,{{{}, {}, 1, {0,std::numeric_limits<float>::max()}, 0}});
    REQUIRE(p[0].force == before);
    solver.prepare(p,{{{2,0}, {}, 1, {}, 1}}); REQUIRE(p[0].force == before);
    c.wallPressureMode = static_cast<WcsphWallPressureMode>(99);
    REQUIRE_THROWS_AS(c.Validate(),std::invalid_argument);
    REQUIRE_THROWS_AS(WcsphSolver(1,c),std::invalid_argument);
}
