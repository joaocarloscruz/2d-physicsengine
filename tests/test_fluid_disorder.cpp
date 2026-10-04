#include "../benchmarks/fluid_disorder_operator.h"
#include "catch_amalgamated.hpp"
#include <algorithm>
#include <limits>

using namespace PhysicsEngine;
using namespace FluidDisorder;

TEST_CASE("Checkerboard displacement is a centrosymmetric density linear null mode",
          "[fluid][disorder][operator]") {
    for (auto family : {Family::Poly6Spiky, Family::CubicSpline})
        for (double dx : {.1, .05, .025})
            for (double ratio : {2., 4., 8.}) {
                const auto row = LatticeRow(dx, ratio * dx, .02, family);
                REQUIRE(row.linearSymbol.norm() < 2e-12);
                const auto pressureRow = LatticeRow(dx, ratio * dx, .02, family, false);
                REQUIRE(pressureRow.linearSymbol.norm() < 2e-12);
                const auto positive = LatticeRow(dx, ratio * dx, .0001, family);
                const auto negative = LatticeRow(dx, ratio * dx, -.0001, family);
                REQUIRE(positive.shifted ==
                        Catch::Approx(negative.shifted).epsilon(0).margin(3e-15));
                REQUIRE(row.shifted == Catch::Approx(LatticeRow(1, ratio, .02, family).shifted)
                                           .epsilon(0)
                                           .margin(4e-15));
            }
    // At h/dx=2 only the four axis (odd parity) neighbors move. Their
    // directional Hessian sum gives delta C_rho=-4.5*a^2/pi+O(a^4).
    const auto row = LatticeRow(1, 2, .001, Family::Poly6Spiky);
    REQUIRE((row.shifted - row.unshifted) / (.001 * .001) ==
            Catch::Approx(-4.5 / Pi).epsilon(0).margin(6e-6));
    const auto original = LatticeRow(1, 2, .02, Family::Poly6Spiky);
    REQUIRE(original.shifted / original.unshifted ==
            Catch::Approx(.9994358958280786).epsilon(0).margin(2e-15));
    REQUIRE(Pressure(original.shifted / original.unshifted, true) == 0);
    REQUIRE(SpecificEnergy(original.shifted / original.unshifted, true) == 0);
}

TEST_CASE("Original disorder inputs are stationary for either kernel at every tested timestep",
          "[fluid][disorder][regime]") {
    for (auto family : {Family::Poly6Spiky, Family::CubicSpline})
        for (int steps : {48, 96, 192}) {
            auto particles = Block(21, .1f, .2f);
            const auto original = particles;
            WcsphConfig c;
            c.externalAcceleration = {};
            c.speedOfSound = 15;
            c.kernelFamily = family;
            c.maximumTimeStep = .2f / steps;
            WcsphSolver solver(.2f, c);
            solver.prepare(particles);
            const auto independent = Build(FromParticles(particles, family));
            REQUIRE(independent.internalEnergy == 0);
            REQUIRE(independent.mechanicalWork == 0);
            for (std::size_t i = 0; i < particles.size(); ++i) {
                REQUIRE(particles[i].density < particles[i].restDensity);
                REQUIRE(particles[i].pressure == 0);
                REQUIRE(particles[i].force == Vector2{});
                REQUIRE(particles[i].density ==
                        Catch::Approx(independent.densities[i]).epsilon(0).margin(.002));
            }
            for (int step = 0; step < steps; ++step)
                solver.step(particles, c.maximumTimeStep);
            for (std::size_t i = 0; i < particles.size(); ++i) {
                REQUIRE(particles[i].position == original[i].position);
                REQUIRE(particles[i].velocity == Vector2{});
                REQUIRE(particles[i].mass == original[i].mass);
                REQUIRE(particles[i].restDensity == original[i].restDensity);
            }
        }
}

TEST_CASE("Removing the clamp drives the infinite bulk checkerboard away from its lattice",
          "[fluid][disorder][tensile][operator]") {
    for (auto family : {Family::Poly6Spiky, Family::CubicSpline}) {
        const auto row = LatticeRow(.1, .2, .02, family);
        const auto pressureRow = LatticeRow(.1, .2, .02, family, false);
        const double ratio = row.shifted * OriginalMassScale, p = Pressure(ratio, false);
        const D2 acceleration =
            pressureRow.gradientSum * (-2 * OriginalMassScale * p / (1000 * ratio * ratio));
        REQUIRE(p < 0);
        REQUIRE(acceleration.x + acceleration.y > 0); // same direction as the imposed shift
        auto particles = Block(21, .1f, .2f);
        WcsphConfig c;
        c.externalAcceleration = {};
        c.speedOfSound = 15;
        c.kernelFamily = family;
        c.clampNegativePressure = false;
        WcsphSolver solver(.2f, c);
        solver.prepare(particles);
        const auto &center = particles[220];
        const D2 native{center.force.x / center.mass, center.force.y / center.mass};
        REQUIRE(native.x + native.y > 0);
        REQUIRE((native - acceleration).norm() < .003);
        double maxSurfaceAcceleration = 0;
        for (std::size_t i = 0; i < particles.size(); ++i)
            if (i % 21 == 0 || i % 21 == 20 || i / 21 == 0 || i / 21 == 20)
                maxSurfaceAcceleration =
                    std::max(maxSurfaceAcceleration,
                             std::hypot(double(particles[i].force.x), particles[i].force.y) /
                                 particles[i].mass);
        REQUIRE(maxSurfaceAcceleration > 100 * acceleration.norm());
    }
}

TEST_CASE("Density derivative and pressure adjoint are independently distinguished",
          "[fluid][disorder][energy][conservation]") {
    for (auto family : {Family::Poly6Spiky, Family::CubicSpline}) {
        auto p = Block(9, .1f, .2f, 1, .02f);
        for (auto &particle : p) {
            particle.position = particle.position * .96f;
            const double x = particle.position.x, y = particle.position.y;
            particle.velocity = {float(.3 * x + .1 * std::sin(4 * y)),
                                 float(-.2 * y + .07 * std::cos(3 * x))};
            particle.viscosity = 0;
        }
        const auto s = FromParticles(p, family);
        const auto op = Build(s);
        WcsphConfig config;
        config.externalAcceleration = {};
        config.speedOfSound = 15;
        config.kernelFamily = family;
        WcsphSolver solver(.2f, config);
        solver.prepare(p);
        D2 forceSum;
        double torque = 0, maxForce = 0, maxDifference = 0, totalForce = 0;
        for (std::size_t i = 0; i < p.size(); ++i) {
            const auto force = op.forces[i];
            forceSum = forceSum + force;
            torque += s.positions[i].x * force.y - s.positions[i].y * force.x;
            maxForce = std::max(maxForce, force.norm());
            totalForce += force.norm();
            maxDifference =
                std::max(maxDifference, (force - D2{p[i].force.x, p[i].force.y}).norm());
        }
        REQUIRE(maxDifference < 2e-5 * maxForce);
        REQUIRE(forceSum.norm() < 2e-13 * totalForce);
        REQUIRE(std::abs(torque) < 2e-13 * totalForce);
        // The pressure-map work cancels by construction even for legacy: it
        // does not prove that the actual summation density has this derivative.
        REQUIRE(std::abs(op.mechanicalWork + op.pressureMapEnergyRate) <
                2e-12 * std::abs(op.mechanicalWork));
        const double residual = op.mechanicalWork + op.trueEnergyRate;
        if (family == Family::CubicSpline)
            REQUIRE(std::abs(residual) < 2e-12 * std::abs(op.mechanicalWork));
        else
            REQUIRE(std::abs(residual) > .05 * std::abs(op.mechanicalWork));
        const double e1 = std::abs(EnergyDifference(s, .001) - op.trueEnergyRate);
        const double e2 = std::abs(EnergyDifference(s, .0005) - op.trueEnergyRate);
        const double e3 = std::abs(EnergyDifference(s, .00025) - op.trueEnergyRate);
        REQUIRE(e1 / e2 > 3.8);
        REQUIRE(e1 / e2 < 4.2);
        REQUIRE(e2 / e3 > 3.8);
        REQUIRE(e2 / e3 < 4.2);
    }
}

TEST_CASE("Independent clamped barotropic energy has the correct derivative and flat branch",
          "[fluid][disorder][energy]") {
    for (double ratio : {.8, .99, 1.01, 1.1, 1.4}) {
        const double derivative =
            (SpecificEnergy(ratio + 1e-6, true) - SpecificEnergy(ratio - 1e-6, true)) / 2e-6;
        REQUIRE(
            derivative ==
            Catch::Approx(Pressure(ratio, true) / 1000 / (ratio * ratio)).epsilon(0).margin(5e-8));
    }
    REQUIRE_THROWS_AS(LatticeRow(0, .2, .02, Family::Poly6Spiky), std::invalid_argument);
    REQUIRE_THROWS_AS(
        LatticeRow(.1, std::numeric_limits<double>::infinity(), .02, Family::Poly6Spiky),
        std::invalid_argument);
    REQUIRE_THROWS_AS(Block(45, .1f, .2f), std::length_error);
    auto s = FromParticles(Block(3, .1f, .2f), Family::CubicSpline);
    REQUIRE_THROWS_AS(EnergyDifference(s, 0), std::invalid_argument);
}
