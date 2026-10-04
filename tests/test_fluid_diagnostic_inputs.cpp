#include "catch_amalgamated.hpp"
#include "physics/core/fluids/fluid_solver.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace PhysicsEngine;
namespace {
using Pair = FluidParticleSpatialGrid::ParticlePair;
const std::vector<Pair> Pairs{{0, 1}};
std::vector<FluidParticle> Particles() {
    FluidParticleProperties p;
    p.mass = 2;
    p.restDensity = 4;
    p.smoothingLength = 1;
    std::vector<FluidParticle> result{
        FluidParticle({0, 0}, {.25f, 0}, p), FluidParticle({.5f, 0}, {-.25f, 0}, p)};
    result[0].density = 6;
    result[1].density = 1;
    result[1].mass = 3;
    return result;
}
} // namespace

TEST_CASE("Public fluid diagnostics reject stale and out of range pair indices",
          "[fluid][diagnostic-input]") {
    const auto p = Particles();
    for (const Pair pair : {Pair{0, 2}, Pair{2, 0}, Pair{0, std::size_t(-1)},
                            Pair{std::size_t(-1), 0}}) {
        REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, {pair}), std::out_of_range);
        REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, {pair}, SphKernelFamily::CubicSpline),
                          std::out_of_range);
    }
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics({}, {{0, 0}}), std::out_of_range);
    const auto empty = MeasureFluidDiagnostics({}, {});
    REQUIRE(empty.maximumDensityError == 0);
    REQUIRE(empty.maximumAbsoluteDensityRate == 0);
    REQUIRE(empty.converged);
}

TEST_CASE("Public fluid diagnostics reject invalid consumed particle fields even without pairs",
          "[fluid][diagnostic-input]") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (const auto member : {&FluidParticle::mass, &FluidParticle::density,
                              &FluidParticle::restDensity, &FluidParticle::smoothingLength}) {
        for (const float bad : {0.f, -1.f, nan, inf, -inf}) {
            auto p = Particles();
            p[0].*member = bad;
            REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, {}), std::invalid_argument);
        }
    }
    for (const auto member : {&FluidParticle::position, &FluidParticle::velocity}) {
        for (const float bad : {nan, inf, -inf}) {
            auto p = Particles();
            (p[0].*member).x = bad;
            REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, {}), std::invalid_argument);
            p = Particles();
            (p[0].*member).y = bad;
            REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, {}), std::invalid_argument);
        }
    }
    // The operator does not consume the pressure, force, volume or cached rate.
    auto p = Particles();
    p[0].pressure = nan;
    p[0].force = {nan, inf};
    p[0].volume = nan;
    p[0].densityRate = nan;
    p[0].inverseMass = nan;
    p[0].viscosity = nan;
    REQUIRE_NOTHROW(MeasureFluidDiagnostics(p, Pairs));
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, Pairs, static_cast<SphKernelFamily>(99)),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, Pairs, SphKernelFamily::CubicSpline, {1}),
                      std::invalid_argument);
    for (const float bad : {nan, inf, -inf})
        REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, Pairs, SphKernelFamily::CubicSpline, {0, bad}),
                          std::invalid_argument);
}

TEST_CASE("Public fluid diagnostics reject overflowing differences rates and summaries",
          "[fluid][diagnostic-input]") {
    const float big = std::numeric_limits<float>::max();
    auto p = Particles();
    p[0].position.x = big;
    p[1].position.x = -big;
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, Pairs), std::overflow_error);
    p = Particles();
    p[0].velocity.x = big;
    p[1].velocity.x = -big;
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, Pairs), std::overflow_error);
    p[1].velocity.x = 0; // finite relative velocity, unrepresentable gradient dot
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, Pairs), std::overflow_error);
    p = Particles();
    p[1].mass = big; // finite dot, unrepresentable mass-weighted contribution
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, Pairs), std::overflow_error);
    p[0].velocity.x = .1f;
    p[1].velocity.x = -.1f; // individual contribution fits; accumulation does not
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, Pairs, SphKernelFamily::Poly6Spiky,
                                            {.75f * big, 0}), std::overflow_error);
    p = Particles();
    p[0].density = big;
    p[0].restDensity = .25f;
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, {}), std::overflow_error);
    p[0].density = 1;
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, {}, SphKernelFamily::Poly6Spiky, {big, 0}),
                      std::overflow_error);
    // Overflowing dot products can cancel to NaN, which std::max used to hide.
    p = Particles();
    p[1].position = {.25f, .25f};
    p[0].velocity = {big, -big};
    p[1].velocity = {};
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, Pairs), std::overflow_error);
}

TEST_CASE("Finite comparison diagnostics retain the pressure operator and caller inputs",
          "[fluid][diagnostic-input]") {
    const auto p = Particles();
    std::array<unsigned char, 2 * sizeof(FluidParticle)> before{};
    std::memcpy(before.data(), p.data(), before.size());
    const std::vector<float> wall{.25f, -.5f};
    constexpr double pi = 3.1415926535897932384626433832795;
    for (const auto family : {SphKernelFamily::Poly6Spiky, SphKernelFamily::CubicSpline}) {
        // Independently differentiated radial pressure weight at r/h=.5.
        const double gradient = family == SphKernelFamily::Poly6Spiky ? 7.5 / pi : 60 / (7 * pi);
        const double rate0 = (.25 + 3 * .5 * gradient) / 4;
        const auto d = MeasureFluidDiagnostics(p, Pairs, family, wall);
        REQUIRE(d.maximumDensityError == .75f);
        REQUIRE(d.maximumCompression == .5f);
        REQUIRE(d.maximumAbsoluteDensityRate == Catch::Approx(rate0));
        REQUIRE(d.maximumCompressionRate == Catch::Approx(rate0));
        const auto reversed = MeasureFluidDiagnostics(p, {{1, 0}}, family, wall);
        REQUIRE(reversed.maximumCompressionRate == d.maximumCompressionRate);
        const auto repeated = MeasureFluidDiagnostics(p, {{0, 1}, {0, 1}}, family);
        REQUIRE(repeated.maximumCompressionRate == Catch::Approx(2 * 3 * .5 * gradient / 4));
        const auto self = MeasureFluidDiagnostics(p, {{0, 0}, {1, 1}}, family, wall);
        REQUIRE(self.maximumAbsoluteDensityRate == .125f);
        REQUIRE(self.maximumCompressionRate == .0625f);
    }
    const auto legacy = MeasureFluidDiagnostics(p, Pairs);
    const auto explicitLegacy = MeasureFluidDiagnostics(p, Pairs, SphKernelFamily::Poly6Spiky);
    REQUIRE(legacy.maximumAbsoluteDensityRate == explicitLegacy.maximumAbsoluteDensityRate);
    REQUIRE(legacy.maximumCompressionRate == explicitLegacy.maximumCompressionRate);
    REQUIRE_THROWS_AS(MeasureFluidDiagnostics(p, {{0, 1}, {0, 2}}), std::out_of_range);
    REQUIRE(std::memcmp(before.data(), p.data(), before.size()) == 0);
    REQUIRE(Pairs == std::vector<Pair>{{0, 1}});
    REQUIRE(wall == std::vector<float>{.25f, -.5f});
}
