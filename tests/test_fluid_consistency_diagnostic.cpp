#include "catch_amalgamated.hpp"
#include "../benchmarks/fluid_consistency_metrics.h"
#include "physics/core/fluids/wcsph_solver.h"
#include <cmath>

using namespace PhysicsEngine;

TEST_CASE("Actual lattice kernel moments match analytic h/dx two sums", "[fluid][consistency][operator]") {
    constexpr double pi = 3.141592653589793238462643383279502884;
    const auto moments = FluidConsistency::MeasureKernelMoments(1, 2);
    const double diagonal = 1 - 1 / std::sqrt(2.0);
    // Only self, four axis neighbors, and four diagonal neighbors contribute.
    const double expectedDensity = 3.1875 / pi;
    const double expectedSpikyWeight = 2.5 / pi * (1.5 + 4 * diagonal * diagonal * diagonal);
    const double expectedSpikyGradient = 30 / (8 * pi)
        * (0.5 + 4 / std::sqrt(2.0) * diagonal * diagonal);
    REQUIRE(moments.supportSamples == 9);
    REQUIRE(moments.density == Catch::Approx(expectedDensity).margin(1e-7));
    REQUIRE(moments.pressureWeight == Catch::Approx(expectedSpikyWeight).margin(1e-7));
    REQUIRE(moments.pressureGradientXX == Catch::Approx(expectedSpikyGradient).margin(1e-7));
    REQUIRE(moments.pressureGradientYY == Catch::Approx(expectedSpikyGradient).margin(1e-7));
    REQUIRE(moments.pressureGradientXY == Catch::Approx(0).margin(1e-10));
    REQUIRE(moments.pressureGradientSumX == Catch::Approx(0).margin(1e-10));
    REQUIRE(moments.pressureGradientSumY == Catch::Approx(0).margin(1e-10));
    REQUIRE(moments.densityGradientXX == Catch::Approx(expectedDensity).margin(1e-7));
    const double massScale = SphKernels2D::SquareLatticeMassScale(1, 2);
    REQUIRE(massScale * moments.pressureGradientXX ==
        Catch::Approx(expectedSpikyGradient / expectedDensity).margin(1e-7));
}

TEST_CASE("Fixed h/dx refinement cannot remove density or gradient moment defects", "[fluid][consistency][refinement]") {
    const auto reference = FluidConsistency::MeasureKernelMoments(0.1f, 0.2f);
    for (float spacing : {0.05f, 0.025f, 0.0125f}) {
        const auto measured = FluidConsistency::MeasureKernelMoments(spacing, 2 * spacing);
        REQUIRE(measured.density == Catch::Approx(reference.density).margin(1e-7));
        REQUIRE(measured.pressureWeight == Catch::Approx(reference.pressureWeight).margin(1e-7));
        REQUIRE(measured.pressureGradientXX == Catch::Approx(reference.pressureGradientXX).margin(1e-7));
        REQUIRE(measured.densityGradientXX == Catch::Approx(reference.densityGradientXX).margin(1e-7));
    }
}

TEST_CASE("Neighbor refinement improves selected density and linear gradient moments", "[fluid][consistency][refinement]") {
    const auto coarse = FluidConsistency::MeasureKernelMoments(0.1f, 0.2f);
    const auto medium = FluidConsistency::MeasureKernelMoments(0.05f, 0.2f);
    const auto fine = FluidConsistency::MeasureKernelMoments(0.025f, 0.2f);
    REQUIRE(std::abs(medium.density - 1) < std::abs(coarse.density - 1));
    REQUIRE(std::abs(fine.density - 1) < std::abs(medium.density - 1));
    REQUIRE(std::abs(medium.pressureGradientXX - 1) < std::abs(coarse.pressureGradientXX - 1));
    REQUIRE(std::abs(fine.pressureGradientXX - 1) < std::abs(medium.pressureGradientXX - 1));
    REQUIRE(std::abs(fine.density - 1) < 0.0001);
    REQUIRE(std::abs(fine.pressureGradientXX - 1) < 0.005);
}

TEST_CASE("Initialized continuity preserves a zero-force rest block with nominal caller mass", "[fluid][consistency][continuity]") {
    FluidParticleProperties properties;
    properties.mass = 10;
    properties.smoothingLength = 0.2f;
    std::vector<FluidParticle> particles;
    for (int y = 0; y < 9; ++y)
        for (int x = 0; x < 9; ++x)
            particles.emplace_back(Vector2{x * 0.1f, y * 0.1f}, Vector2{}, properties);
    const auto initial = particles;
    WcsphConfig config;
    config.externalAcceleration = {};
    config.densityMode = WcsphDensityMode::Continuity;
    WcsphSolver solver(0.2f, config);
    for (int step = 0; step < 24; ++step) solver.step(particles, 1 / 240.0f);
    for (std::size_t i = 0; i < particles.size(); ++i) {
        REQUIRE(particles[i].position == initial[i].position);
        REQUIRE(particles[i].velocity == Vector2{});
        REQUIRE(particles[i].density == particles[i].restDensity);
        REQUIRE(particles[i].mass == initial[i].mass);
        REQUIRE(particles[i].restDensity == initial[i].restDensity);
    }
}

TEST_CASE("Cubic diagnostic candidate retains physical unit normalization and its derivative", "[fluid][consistency][candidate]") {
    constexpr double pi = 3.141592653589793238462643383279502884;
    for (double h : {0.2, 1.0, 2.0}) {
        constexpr int samples = 20000;
        const double width = h / samples;
        double integral = 0;
        for (int i = 0; i < samples; ++i) {
            const double radius = (i + 0.5) * width;
            integral += 2 * pi * radius * width
                * FluidConsistency::CubicSplineCandidate(radius, h).weight;
        }
        REQUIRE(integral == Catch::Approx(1).margin(1e-8));
        for (double ratio : {0.2, 0.49, 0.5, 0.7, 0.99}) {
            const double radius = h * ratio, epsilon = h * 1e-6;
            const double finiteDifference = (
                FluidConsistency::CubicSplineCandidate(radius + epsilon, h).weight -
                FluidConsistency::CubicSplineCandidate(radius - epsilon, h).weight) / (2 * epsilon);
            REQUIRE(FluidConsistency::CubicSplineCandidate(radius, h).radialDerivative ==
                Catch::Approx(finiteDifference).epsilon(1e-6));
        }
        REQUIRE(FluidConsistency::CubicSplineCandidate(h, h).weight == 0);
        REQUIRE(FluidConsistency::CubicSplineCandidate(h, h).radialDerivative == 0);
        REQUIRE(FluidConsistency::CubicSplineCandidate(0, h).radialDerivative == 0);
    }
}

TEST_CASE("Cubic candidate lattice moments match analytic sums without calibration", "[fluid][consistency][candidate]") {
    constexpr double pi = 3.141592653589793238462643383279502884;
    const double diagonal = 2 - std::sqrt(2.0);
    const auto moments = FluidConsistency::MeasureKernelMoments(1, 2);
    REQUIRE(moments.candidateCubicDensity ==
        Catch::Approx(10 / (7 * pi) * (2 + diagonal * diagonal * diagonal)).margin(1e-12));
    REQUIRE(moments.candidateCubicGradientXX ==
        Catch::Approx(10 / (7 * pi) * (1.5 + 3 / std::sqrt(2.0) * diagonal * diagonal)).margin(1e-12));
    REQUIRE(std::abs(moments.candidateCubicDensity - 1) < std::abs(moments.density - 1));
    REQUIRE(std::abs(moments.candidateCubicGradientXX - 1) < std::abs(moments.pressureGradientXX - 1));
}
