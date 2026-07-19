#include "catch_amalgamated.hpp"

#include "physics/core/fluids/sph_kernels.h"
#include "physics/core/fluids/wcsph_solver.h"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace PhysicsEngine;

namespace {

Vector2 TotalMomentum(const std::vector<FluidParticle>& particles) {
    Vector2 momentum;
    for (const FluidParticle& particle : particles) {
        momentum = momentum + particle.velocity * particle.mass;
    }
    return momentum;
}

void ClampToBenchmarkTank(std::vector<FluidParticle>& particles) {
    for (FluidParticle& particle : particles) {
        if (particle.position.x < -2.0f) {
            particle.position.x = -2.0f;
            particle.velocity.x = 0.0f;
        } else if (particle.position.x > 2.0f) {
            particle.position.x = 2.0f;
            particle.velocity.x = 0.0f;
        }
        if (particle.position.y < 0.0f) {
            particle.position.y = 0.0f;
            particle.velocity.y = 0.0f;
        } else if (particle.position.y > 4.0f) {
            particle.position.y = 4.0f;
            particle.velocity.y = 0.0f;
        }
    }
}

std::vector<FluidParticle> RunBenchmark(bool damBreak) {
    constexpr float spacing = 0.25f;
    FluidParticleProperties properties;
    properties.mass = properties.restDensity * spacing * spacing;
    properties.smoothingLength = 0.5f;
    properties.viscosity = 0.08f;

    std::vector<FluidParticle> particles;
    const int columns = damBreak ? 8 : 6;
    const int rows = damBreak ? 8 : 12;
    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < columns; ++column) {
            particles.emplace_back(
                Vector2(
                    -1.75f + static_cast<float>(column) * spacing,
                    0.15f + static_cast<float>(row) * spacing
                ),
                damBreak && row == rows - 1
                    ? Vector2(0.15f, 0.0f)
                    : Vector2(),
                properties
            );
        }
    }

    WcsphConfig config;
    config.speedOfSound = 15.0f;
    config.externalAcceleration = Vector2(0.0f, -9.81f);
    WcsphSolver solver(properties.smoothingLength, config);
    const int steps = damBreak ? 180 : 240;
    for (int step = 0; step < steps; ++step) {
        solver.step(particles, 0.002f);
        ClampToBenchmarkTank(particles);
    }
    return particles;
}

void RequireFinite(const std::vector<FluidParticle>& particles) {
    for (const FluidParticle& particle : particles) {
        REQUIRE(std::isfinite(particle.position.x));
        REQUIRE(std::isfinite(particle.position.y));
        REQUIRE(std::isfinite(particle.velocity.x));
        REQUIRE(std::isfinite(particle.velocity.y));
        REQUIRE(std::isfinite(particle.density));
        REQUIRE(std::isfinite(particle.pressure));
        REQUIRE(particle.density > 0.0f);
    }
}

} // namespace

TEST_CASE("WCSPH estimates density and applies its equation of state", "[fluid][wcsph][density]") {
    FluidParticleProperties properties;
    properties.mass = 8.0f;
    properties.restDensity = 10.0f;
    properties.smoothingLength = 0.8f;
    std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(), Vector2(), properties)
    };
    WcsphConfig config;
    config.speedOfSound = 4.0f;
    config.equationOfStateExponent = 7.0f;
    config.externalAcceleration = Vector2();
    WcsphSolver solver(properties.smoothingLength, config);

    solver.prepare(particles);

    const float expectedDensity = properties.mass
        * SphKernels2D::DensityWeight(Vector2(), properties.smoothingLength);
    const float ratio = expectedDensity / properties.restDensity;
    const float expectedPressure = properties.restDensity
        * config.speedOfSound * config.speedOfSound
        / config.equationOfStateExponent
        * (std::pow(ratio, config.equationOfStateExponent) - 1.0f);
    REQUIRE(particles.front().density == Catch::Approx(expectedDensity));
    REQUIRE(particles.front().pressure == Catch::Approx(std::max(expectedPressure, 0.0f)));
    REQUIRE(particles.front().pressure > 0.0f);
    REQUIRE(particles.front().volume == Catch::Approx(
        properties.mass / expectedDensity
    ));
}

TEST_CASE("WCSPH rejects invalid solver configuration and timesteps", "[fluid][wcsph][validation]") {
    WcsphConfig config;

    SECTION("speed of sound must be positive") {
        config.speedOfSound = 0.0f;
        REQUIRE_THROWS_AS(WcsphSolver(0.5f, config), std::invalid_argument);
    }
    SECTION("equation-of-state exponent must exceed one") {
        config.equationOfStateExponent = 1.0f;
        REQUIRE_THROWS_AS(WcsphSolver(0.5f, config), std::invalid_argument);
    }
    SECTION("CFL factor must be normalized") {
        config.cflFactor = 1.1f;
        REQUIRE_THROWS_AS(WcsphSolver(0.5f, config), std::invalid_argument);
    }
    SECTION("density diffusion must be normalized") {
        config.densityDiffusion = 1.1f;
        REQUIRE_THROWS_AS(WcsphSolver(0.5f, config), std::invalid_argument);
    }
    SECTION("density mode must be recognized") {
        config.densityMode = static_cast<WcsphDensityMode>(-1);
        REQUIRE_THROWS_AS(WcsphSolver(0.5f, config), std::invalid_argument);
    }
    SECTION("continuity mode requires positive supplied density") {
        config.densityMode = WcsphDensityMode::Continuity;
        WcsphSolver solver(0.5f, config);
        std::vector<FluidParticle> particles = {
            FluidParticle(Vector2(), Vector2(), FluidParticleProperties{})
        };
        particles.front().density = 0.0f;
        REQUIRE_THROWS_AS(solver.prepare(particles), std::invalid_argument);
    }
    SECTION("step rejects negative time") {
        WcsphSolver solver(0.5f, config);
        std::vector<FluidParticle> particles;
        REQUIRE_THROWS_AS(solver.step(particles, -0.01f), std::invalid_argument);
    }
}

TEST_CASE("WCSPH pair forces conserve linear momentum", "[fluid][wcsph][conservation]") {
    FluidParticleProperties properties;
    properties.mass = 1.0f;
    properties.restDensity = 1.0f;
    properties.smoothingLength = 1.0f;
    properties.viscosity = 0.2f;
    std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(-0.2f, 0.0f), Vector2(0.0f, 1.0f), properties),
        FluidParticle(Vector2(0.2f, 0.0f), Vector2(0.0f, -1.0f), properties),
    };
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    config.speedOfSound = 2.0f;
    WcsphSolver solver(properties.smoothingLength, config);
    const Vector2 momentumBefore = TotalMomentum(particles);
    const float tangentialSpeedBefore = std::abs(
        particles[1].velocity.y - particles[0].velocity.y
    );

    solver.step(particles, 0.001f);

    const Vector2 momentumAfter = TotalMomentum(particles);
    const float tangentialSpeedAfter = std::abs(
        particles[1].velocity.y - particles[0].velocity.y
    );
    REQUIRE(momentumAfter.x == Catch::Approx(momentumBefore.x).margin(1e-6f));
    REQUIRE(momentumAfter.y == Catch::Approx(momentumBefore.y).margin(1e-6f));
    REQUIRE(tangentialSpeedAfter < tangentialSpeedBefore);
}

TEST_CASE("WCSPH CFL limit responds to wave and particle speeds", "[fluid][wcsph][cfl]") {
    FluidParticleProperties properties;
    properties.smoothingLength = 0.5f;
    std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(), Vector2(), properties)
    };
    WcsphConfig config;
    config.speedOfSound = 10.0f;
    config.externalAcceleration = Vector2();
    WcsphSolver solver(properties.smoothingLength, config);
    const float restingLimit = solver.getStableTimeStep(particles);

    particles.front().velocity = Vector2(20.0f, 0.0f);
    const float movingLimit = solver.getStableTimeStep(particles);

    REQUIRE(restingLimit > 0.0f);
    REQUIRE(movingLimit < restingLimit);
    solver.step(particles, restingLimit * 2.5f);
    REQUIRE(solver.getLastStatistics().substepCount >= 3);
}

TEST_CASE("WCSPH continuity mode preserves supplied density at rest", "[fluid][wcsph][density][continuity]") {
    FluidParticleProperties properties;
    properties.mass = 10.0f;
    properties.restDensity = 1000.0f;
    properties.smoothingLength = 0.2f;
    std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(0.0f, 0.0f), Vector2(), properties),
        FluidParticle(Vector2(0.1f, 0.0f), Vector2(), properties),
    };
    particles[0].density = 1008.0f;
    particles[1].density = 1003.0f;
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    config.densityMode = WcsphDensityMode::Continuity;
    WcsphSolver solver(properties.smoothingLength, config);

    solver.prepare(particles);

    REQUIRE(particles[0].density == Catch::Approx(1008.0f));
    REQUIRE(particles[1].density == Catch::Approx(1003.0f));
    REQUIRE(particles[0].pressure > particles[1].pressure);
}

TEST_CASE("WCSPH continuity mode increases density under compression", "[fluid][wcsph][density][continuity]") {
    FluidParticleProperties properties;
    properties.mass = 10.0f;
    properties.restDensity = 1000.0f;
    properties.smoothingLength = 0.2f;
    properties.viscosity = 0.0f;
    std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(-0.05f, 0.0f), Vector2(1.0f, 0.0f), properties),
        FluidParticle(Vector2(0.05f, 0.0f), Vector2(-1.0f, 0.0f), properties),
    };
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    config.densityMode = WcsphDensityMode::Continuity;
    config.maximumTimeStep = 0.0001f;
    WcsphSolver solver(properties.smoothingLength, config);

    solver.step(particles, 0.0001f);

    REQUIRE(particles[0].density > properties.restDensity);
    REQUIRE(particles[1].density > properties.restDensity);
    REQUIRE(particles[0].density == Catch::Approx(particles[1].density));
}

TEST_CASE("WCSPH continuity diffusion damps dynamic density differences", "[fluid][wcsph][density][continuity][diffusion]") {
    FluidParticleProperties properties;
    properties.mass = 10.0f;
    properties.restDensity = 1000.0f;
    properties.smoothingLength = 0.2f;
    properties.viscosity = 0.0f;
    std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(-0.05f, 0.0f), Vector2(), properties),
        FluidParticle(Vector2(0.05f, 0.0f), Vector2(), properties),
    };
    particles[0].density = 1008.0f;
    particles[1].density = 1000.0f;
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    config.densityMode = WcsphDensityMode::Continuity;
    config.densityDiffusion = 0.1f;
    config.maximumTimeStep = 0.00001f;
    WcsphSolver solver(properties.smoothingLength, config);

    solver.step(particles, 0.00001f);

    REQUIRE(particles[0].density < 1008.0f);
    REQUIRE(particles[1].density > 1000.0f);
    REQUIRE(particles[0].density > particles[1].density);
}

TEST_CASE("WCSPH neighborhood work remains linear through ten thousand particles", "[fluid][wcsph][scaling]") {
    constexpr float spacing = 0.25f;
    FluidParticleProperties properties;
    properties.mass = properties.restDensity * spacing * spacing;
    properties.smoothingLength = 0.5f;
    for (std::size_t particleCount : {1000u, 3000u, 10000u}) {
        const std::size_t columns = static_cast<std::size_t>(std::ceil(
            std::sqrt(static_cast<double>(particleCount))
        ));
        std::vector<FluidParticle> particles;
        particles.reserve(particleCount);
        for (std::size_t index = 0; index < particleCount; ++index) {
            const std::size_t row = index / columns;
            const std::size_t column = index % columns;
            particles.emplace_back(
                Vector2(
                    static_cast<float>(column) * spacing,
                    static_cast<float>(row) * spacing
                ),
                Vector2(),
                properties
            );
        }
        WcsphConfig config;
        config.externalAcceleration = Vector2();
        WcsphSolver solver(properties.smoothingLength, config);

        solver.prepare(particles);

        const FluidNeighborStatistics& neighbors =
            solver.getLastStatistics().neighbors;
        INFO("particle count: " << particleCount);
        REQUIRE(neighbors.particleCount == particleCount);
        REQUIRE(neighbors.candidatePairCount < particleCount * 25);
        REQUIRE(neighbors.neighborPairCount < particleCount * 10);
    }
}

TEST_CASE("WCSPH hydrostatic column benchmark is finite and repeatable", "[fluid][wcsph][benchmark]") {
    const std::vector<FluidParticle> first = RunBenchmark(false);
    const std::vector<FluidParticle> second = RunBenchmark(false);
    RequireFinite(first);
    REQUIRE(first.size() == second.size());
    for (std::size_t index = 0; index < first.size(); ++index) {
        REQUIRE(first[index].position == second[index].position);
        REQUIRE(first[index].velocity == second[index].velocity);
        REQUIRE(first[index].density == second[index].density);
        REQUIRE(first[index].pressure == second[index].pressure);
    }
}

TEST_CASE("WCSPH dam-break benchmark is finite and repeatable", "[fluid][wcsph][benchmark]") {
    const std::vector<FluidParticle> first = RunBenchmark(true);
    const std::vector<FluidParticle> second = RunBenchmark(true);
    RequireFinite(first);
    REQUIRE(first.size() == second.size());
    for (std::size_t index = 0; index < first.size(); ++index) {
        REQUIRE(first[index].position == second[index].position);
        REQUIRE(first[index].velocity == second[index].velocity);
        REQUIRE(first[index].density == second[index].density);
        REQUIRE(first[index].pressure == second[index].pressure);
    }
}
