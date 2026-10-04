#include "catch_amalgamated.hpp"
#include "physics/core/fluids/dfsph_solver.h"
#include "physics/core/fluids/wcsph_solver.h"
#include "physics/core/fluids/sph_kernels.h"
#include <cmath>
#include <limits>
#include <memory>

using namespace PhysicsEngine;
namespace {
std::unique_ptr<IFluidSolver> MakeSolver(bool projected, float maximumTimeStep = 0.1f,
                                       int maximumSubsteps = 1024) {
    if (projected) {
        DfsphConfig config;
        config.externalAcceleration = {};
        config.maximumTimeStep = maximumTimeStep;
        config.maximumSubsteps = maximumSubsteps;
        return std::make_unique<DfsphSolver>(1.0f, config);
    }
    WcsphConfig config;
    config.externalAcceleration = {};
    config.speedOfSound = 0.1f;
    config.maximumTimeStep = maximumTimeStep;
    config.maximumSubsteps = maximumSubsteps;
    return std::make_unique<WcsphSolver>(1.0f, config);
}
std::vector<FluidParticle> ViscousPair(float mass = 0.001f, float viscosity = 1.0f) {
    FluidParticleProperties properties;
    properties.mass = mass;
    properties.restDensity = mass * 1000.0f;
    properties.smoothingLength = 1.0f;
    properties.viscosity = viscosity;
    return {FluidParticle({-0.2f, 0.0f}, {0.0f, 1.0f}, properties),
            FluidParticle({0.2f, 0.0f}, {0.0f, -1.0f}, properties)};
}
double Energy(const std::vector<FluidParticle>& particles) {
    double result = 0;
    for (const auto& p : particles) {
        result += 0.5 * p.mass * (static_cast<double>(p.velocity.x) * p.velocity.x
                                 + static_cast<double>(p.velocity.y) * p.velocity.y);
    }
    return result;
}
Vector2 Momentum(const std::vector<FluidParticle>& particles) {
    double x = 0, y = 0;
    for (const auto& p : particles) { x += static_cast<double>(p.mass) * p.velocity.x;
                                    y += static_cast<double>(p.mass) * p.velocity.y; }
    return Vector2(static_cast<float>(x), static_cast<float>(y));
}
}

TEST_CASE("Explicit viscosity dissipates pressure free low density pair energy", "[fluid-viscosity]") {
    const bool projected = GENERATE(false, true);
    INFO("DFSPH=" << projected);
    auto particles = ViscousPair();
    const double before = Energy(particles);
    auto solver = MakeSolver(projected);
    solver->step(particles, 0.001f);
    REQUIRE(Energy(particles) < before);
    REQUIRE(solver->getDiagnostics().substeps > 1);
    REQUIRE(Momentum(particles).magnitude() < 1e-8f);
}

TEST_CASE("Viscous diffusion preserves unequal mass momentum and dissipates repeatedly", "[fluid-viscosity]") {
    const bool projected = GENERATE(false, true);
    auto particles = ViscousPair();
    particles[1].mass = 0.004f;
    particles[1].inverseMass = 250.0f;
    particles[1].restDensity = 2.0f;
    particles[1].density = 2.0f;
    particles[1].viscosity = 0.5f;
    particles[1].smoothingLength = 0.8f;
    particles[0].velocity = Vector2(0.2f, 1.0f);
    particles[1].velocity = Vector2(0.2f, -0.25f);
    const Vector2 momentum = Momentum(particles);
    auto solver = MakeSolver(projected);
    for (int step = 0; step < 20; ++step) {
        const double before = Energy(particles);
        solver->step(particles, 0.0005f);
        REQUIRE(Energy(particles) <= before + 1e-10);
        REQUIRE((Momentum(particles)-momentum).magnitude() < 1e-8f);
        for (const auto& p : particles) {
            REQUIRE(p.pressure == 0.0f);
            REQUIRE(std::isfinite(p.density));
        }
    }
}

TEST_CASE("Uniform velocity and zero viscosity keep pressure free fluid equilibrium", "[fluid-viscosity]") {
    const bool projected = GENERATE(false, true);
    for (float viscosity : {0.0f, 1.0f}) {
        auto particles = ViscousPair(0.001f, viscosity);
        for (auto& p : particles) p.velocity = Vector2(0.25f, -0.5f);
        const double energy = Energy(particles);
        auto solver = MakeSolver(projected);
        solver->step(particles, 0.001f);
        REQUIRE(Energy(particles) == Catch::Approx(energy));
        for (const auto& p : particles) {
            REQUIRE(p.velocity == Vector2(0.25f, -0.5f));
        }
    }
}

TEST_CASE("Continuum viscosity timestep uses dynamic viscosity divided by density", "[fluid-viscosity]") {
    WcsphConfig config;
    config.externalAcceleration = {};
    config.speedOfSound = 0.001f;
    config.maximumTimeStep = 1000.0f;
    config.cflFactor = 1.0f;
    WcsphSolver solver(1.0f, config);
    auto particles = ViscousPair();
    particles.resize(1);
    particles[0].velocity = {};
    particles[0].density = 0.5f;
    REQUIRE(solver.getStableTimeStep(particles) == Catch::Approx(0.0625f));
    particles[0].density = 2.0f;
    REQUIRE(solver.getStableTimeStep(particles) == Catch::Approx(0.25f));
    particles[0].density = 1000.0f;
    REQUIRE(solver.getStableTimeStep(particles) == Catch::Approx(125.0f));
}

TEST_CASE("Prepared neighbor row bound limits diffusion even at zero velocity", "[fluid-viscosity]") {
    auto particles = ViscousPair();
    for (auto& p : particles) { p.position = {}; p.velocity = {}; }
    WcsphConfig config;
    config.externalAcceleration = {};
    config.speedOfSound = 0.1f;
    WcsphSolver solver(1.0f, config);
    solver.prepare(particles);
    const double rho = particles[0].density;
    const double lambda = static_cast<double>(particles[0].viscosity)
        * particles[0].mass * particles[1].mass / (rho * rho)
        * SphKernels2D::ViscosityLaplacian({}, 1.0f);
    const double rowLimit = 0.5 * particles[0].mass / lambda;
    REQUIRE(solver.getLastStatistics().stableTimeStep <= rowLimit);
    REQUIRE(solver.getLastStatistics().stableTimeStep == Catch::Approx(rowLimit));
    REQUIRE(solver.getStableTimeStep(particles) > solver.getLastStatistics().stableTimeStep);
    DfsphConfig dfsphConfig;
    dfsphConfig.externalAcceleration = {};
    DfsphSolver dfsph(1.0f, dfsphConfig);
    dfsph.step(particles, static_cast<float>(rowLimit * 2.5));
    REQUIRE(dfsph.getDiagnostics().substeps >= 3);
}

TEST_CASE("Viscosity diffusion has consistent density and time scaling", "[fluid-viscosity]") {
    const bool projected = GENERATE(false, true);
    auto low = ViscousPair(0.001f);
    auto high = ViscousPair(0.002f);
    auto first = MakeSolver(projected), second = MakeSolver(projected);
    first->step(low, 1e-5f);
    second->step(high, 2e-5f);
    REQUIRE(low[0].velocity.y == Catch::Approx(high[0].velocity.y).margin(1e-6f));
    REQUIRE(low[1].velocity.y == Catch::Approx(high[1].velocity.y).margin(1e-6f));
    REQUIRE(first->getDiagnostics().substeps == second->getDiagnostics().substeps);
}

TEST_CASE("Explicit viscous diffusion converges with timestep refinement", "[fluid-viscosity]") {
    const bool projected = GENERATE(false, true);
    const auto run = [projected](float maximumTimeStep) {
        auto particles = ViscousPair();
        auto solver = MakeSolver(projected, maximumTimeStep);
        solver->step(particles, 0.0001f);
        return particles[0].velocity.y;
    };
    const float reference = run(0.0001f / 64.0f);
    const float coarse = std::abs(run(0.0001f) - reference);
    const float medium = std::abs(run(0.00005f) - reference);
    const float fine = std::abs(run(0.000025f) - reference);
    REQUIRE(medium < coarse * 0.65f);
    REQUIRE(fine < medium * 0.65f);
}

TEST_CASE("Viscosity arithmetic keeps representable coefficients finite at extreme scale", "[fluid-viscosity]") {
    const bool projected = GENERATE(false, true);
    auto particles = ViscousPair(1e20f, 1e30f);
    particles[0].velocity = Vector2(0, 1e-20f);
    particles[1].velocity = Vector2(0, -1e-20f);
    const double before = Energy(particles);
    auto solver = MakeSolver(projected);
    REQUIRE_NOTHROW(solver->step(particles, 1e-11f));
    REQUIRE(Energy(particles) < before);
    for (const auto& p : particles) {
        REQUIRE(std::isfinite(p.force.x));
        REQUIRE(std::isfinite(p.force.y));
        REQUIRE(std::isfinite(p.velocity.y));
    }
}

TEST_CASE("Unrepresentable viscous force and timestep fail explicitly", "[fluid-viscosity]") {
    const bool projected = GENERATE(false, true);
    auto particles = ViscousPair(0.001f, std::numeric_limits<float>::max());
    auto solver = MakeSolver(projected);
    REQUIRE_THROWS_AS(solver->step(particles, 0.001f), std::overflow_error);
    for (auto& p : particles) { p.velocity = {}; p.mass = 1e-30f; p.restDensity = 1.0f; }
    // Force is zero, but the diffusion time bound is below the float time domain.
    REQUIRE_THROWS_AS(solver->step(particles, 0.001f), std::runtime_error);
}

TEST_CASE("Strong finite viscosity retains the configured substep work budget", "[fluid-viscosity]") {
    const bool projected = GENERATE(false, true);
    auto particles = ViscousPair(0.001f, 1000.0f);
    auto solver = MakeSolver(projected, 0.1f, 2);
    REQUIRE_THROWS_AS(solver->step(particles, 0.001f), std::runtime_error);
    for (const auto& p : particles) {
        REQUIRE(std::isfinite(p.position.x));
        REQUIRE(std::isfinite(p.velocity.y));
    }
}
