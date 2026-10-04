#include "catch_amalgamated.hpp"
#include "physics/core/fluids/dfsph_solver.h"
#include "physics/core/fluids/wcsph_solver.h"
#include "physics/core/fluids/sph_kernels.h"
#include <limits>

using namespace PhysicsEngine;
namespace {
std::vector<FluidParticle> CompressionPatch(float speed = 1, SphKernelFamily family = SphKernelFamily::Poly6Spiky) {
    constexpr float spacing = 0.1f, h = 0.25f;
    FluidParticleProperties properties;
    properties.smoothingLength = h; properties.viscosity = 0;
    properties.mass = properties.restDensity*spacing*spacing*SphKernels2D::SquareLatticeMassScale(spacing, h, family);
    std::vector<FluidParticle> particles;
    for (int y=-5; y<=5; ++y) for (int x=-5; x<=5; ++x) {
        const Vector2 position(x*spacing, y*spacing);
        particles.emplace_back(position, position*(-speed), properties);
    }
    return particles;
}
Vector2 Momentum(const std::vector<FluidParticle>& particles) {
    Vector2 result;
    for (const auto& p : particles) result = result+p.velocity*p.mass;
    return result;
}
}

TEST_CASE("DFSPH reduces compression and divergence against WCSPH", "[dfsph]") {
    const auto family = GENERATE(SphKernelFamily::Poly6Spiky, SphKernelFamily::CubicSpline, SphKernelFamily::WendlandC2);
    auto reference = CompressionPatch(1, family); auto projected = reference;
    WcsphConfig weak; weak.kernelFamily = family; weak.externalAcceleration = {}; weak.speedOfSound = GENERATE(5.0f, 20.0f);
    DfsphConfig strong; strong.kernelFamily = family; strong.externalAcceleration = {};
    strong.densityTolerance = 1e-4f; strong.divergenceTolerance = 1e-3f;
    strong.maximumIterations = 1000;
    WcsphSolver wcsph(0.25f, weak); DfsphSolver dfsph(0.25f, strong);
    IFluidSolver* referenceSolver = &wcsph; IFluidSolver* projectedSolver = &dfsph;
    referenceSolver->step(reference, 0.01f); projectedSolver->step(projected, 0.01f);
    const auto a = wcsph.getDiagnostics(), b = dfsph.getDiagnostics();
    INFO("WCSPH compression=" << a.maximumCompression << " rate=" << a.maximumCompressionRate);
    INFO("DFSPH compression=" << b.maximumCompression << " rate=" << b.maximumCompressionRate
        << " iterations=" << b.densityIterations << "/" << b.divergenceIterations);
    REQUIRE(b.maximumCompression < a.maximumCompression*0.5f);
    REQUIRE(b.maximumCompressionRate < a.maximumCompressionRate*0.1f);
    REQUIRE(b.converged);
    REQUIRE(b.densityResidual <= strong.densityTolerance);
    REQUIRE(b.divergenceResidual <= strong.divergenceTolerance);
    REQUIRE((Momentum(projected)-Momentum(reference)).magnitude() < 0.001f);
}

TEST_CASE("DFSPH tighter tolerance improves the projection", "[dfsph]") {
    const auto family = GENERATE(SphKernelFamily::Poly6Spiky, SphKernelFamily::CubicSpline, SphKernelFamily::WendlandC2);
    auto looseParticles = CompressionPatch(2, family), tightParticles = looseParticles;
    DfsphConfig loose; loose.kernelFamily = family; loose.externalAcceleration = {}; loose.divergenceTolerance = 0.1f;
    DfsphConfig tight = loose; tight.divergenceTolerance = 1e-4f;
    DfsphSolver a(0.25f, loose), b(0.25f, tight);
    a.step(looseParticles, 0.005f); b.step(tightParticles, 0.005f);
    REQUIRE(b.getDiagnostics().maximumCompressionRate <= a.getDiagnostics().maximumCompressionRate+1e-5f);
    REQUIRE(b.getDiagnostics().divergenceIterations >= a.getDiagnostics().divergenceIterations);
}

TEST_CASE("DFSPH maintains finite state and momentum over repeated steps", "[dfsph]") {
    const auto family = GENERATE(SphKernelFamily::Poly6Spiky, SphKernelFamily::CubicSpline, SphKernelFamily::WendlandC2);
    auto particles = CompressionPatch(1, family);
    for (auto& p : particles) p.velocity = p.velocity+Vector2(0.3f, -0.2f);
    const Vector2 before = Momentum(particles);
    DfsphConfig config; config.kernelFamily = family; config.externalAcceleration = {};
    DfsphSolver solver(0.25f, config);
    for (int i=0; i<100; ++i) {
        solver.step(particles, 0.005f);
        for (const auto& p : particles) {
            REQUIRE(std::isfinite(p.position.x)); REQUIRE(std::isfinite(p.position.y));
            REQUIRE(p.density > 0);
        }
    }
    REQUIRE((Momentum(particles)-before).magnitude() < 0.01f);
}

TEST_CASE("DFSPH validates inputs and exposes iteration limits", "[dfsph][validation]") {
    const auto family = GENERATE(SphKernelFamily::Poly6Spiky, SphKernelFamily::CubicSpline, SphKernelFamily::WendlandC2);
    DfsphConfig config; config.kernelFamily = family; config.externalAcceleration = {}; config.maximumIterations = 1;
    config.densityTolerance = 1e-6f; config.divergenceTolerance = 1e-6f;
    DfsphSolver solver(0.25f, config);
    auto particles = CompressionPatch(3, family);
    solver.step(particles, 0.01f);
    REQUIRE_FALSE(solver.getDiagnostics().converged);
    REQUIRE_THROWS_AS(solver.step(particles, -1), std::invalid_argument);
    particles[0].mass = 0;
    REQUIRE_THROWS_AS(solver.step(particles, 0), std::invalid_argument);
    config.relaxation = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(DfsphSolver(0.25f, config), std::invalid_argument);
    std::vector<FluidParticle> empty;
    solver.step(empty, 0.1f);
    REQUIRE(solver.getDiagnostics().substeps == 0);
    REQUIRE(solver.getDiagnostics().maximumCompression == 0);
}
