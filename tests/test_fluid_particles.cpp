#include "catch_amalgamated.hpp"

#include "physics/core/fluids/fluid_particle.h"
#include "physics/core/fluids/fluid_particle_spatial_grid.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using namespace PhysicsEngine;

namespace {

std::vector<FluidParticleSpatialGrid::ParticlePair> BruteForcePairs(
    const std::vector<FluidParticle>& particles,
    float radius
) {
    std::vector<FluidParticleSpatialGrid::ParticlePair> pairs;
    const float radiusSquared = radius * radius;
    for (std::size_t first = 0; first < particles.size(); ++first) {
        for (std::size_t second = first + 1; second < particles.size(); ++second) {
            if ((particles[second].position - particles[first].position)
                    .magnitudeSquared() <= radiusSquared) {
                pairs.emplace_back(first, second);
            }
        }
    }
    return pairs;
}

} // namespace

TEST_CASE("Fluid particles keep fluid state separate from generic particles", "[fluid][particle]") {
    FluidParticleProperties properties;
    properties.mass = 2.0f;
    properties.restDensity = 1000.0f;
    properties.smoothingLength = 0.6f;
    properties.viscosity = 0.08f;

    FluidParticle particle(
        Vector2(1.0f, 2.0f),
        Vector2(3.0f, 4.0f),
        properties
    );

    REQUIRE(particle.position == Vector2(1.0f, 2.0f));
    REQUIRE(particle.velocity == Vector2(3.0f, 4.0f));
    REQUIRE(particle.mass == Catch::Approx(2.0f));
    REQUIRE(particle.inverseMass == Catch::Approx(0.5f));
    REQUIRE(particle.density == Catch::Approx(1000.0f));
    REQUIRE(particle.pressure == Catch::Approx(0.0f));
    REQUIRE(particle.smoothingLength == Catch::Approx(0.6f));
    REQUIRE(particle.restDensity == Catch::Approx(1000.0f));
    REQUIRE(particle.viscosity == Catch::Approx(0.08f));
    REQUIRE(particle.volume == Catch::Approx(0.002f));
}

TEST_CASE("Fluid particle properties reject invalid numerical state", "[fluid][particle][validation]") {
    FluidParticleProperties properties;

    SECTION("mass must be positive and finite") {
        properties.mass = 0.0f;
        REQUIRE_THROWS_AS(FluidParticle(Vector2(), Vector2(), properties), std::invalid_argument);
        properties.mass = std::numeric_limits<float>::infinity();
        REQUIRE_THROWS_AS(FluidParticle(Vector2(), Vector2(), properties), std::invalid_argument);
    }

    SECTION("rest density and smoothing length must be positive") {
        properties.restDensity = 0.0f;
        REQUIRE_THROWS_AS(FluidParticle(Vector2(), Vector2(), properties), std::invalid_argument);
        properties.restDensity = 1000.0f;
        properties.smoothingLength = -0.1f;
        REQUIRE_THROWS_AS(FluidParticle(Vector2(), Vector2(), properties), std::invalid_argument);
    }

    SECTION("viscosity must be finite and non-negative") {
        properties.viscosity = -0.01f;
        REQUIRE_THROWS_AS(FluidParticle(Vector2(), Vector2(), properties), std::invalid_argument);
        properties.viscosity = std::numeric_limits<float>::quiet_NaN();
        REQUIRE_THROWS_AS(FluidParticle(Vector2(), Vector2(), properties), std::invalid_argument);
    }
}

TEST_CASE("Fluid neighbor pairs are deterministic and ordered", "[fluid][neighbors]") {
    const FluidParticleProperties properties;
    const std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(1.0f, 0.0f), Vector2(), properties),
        FluidParticle(Vector2(-0.5f, 0.0f), Vector2(), properties),
        FluidParticle(Vector2(0.0f, 0.0f), Vector2(), properties),
        FluidParticle(Vector2(0.5f, 0.0f), Vector2(), properties),
    };
    FluidParticleSpatialGrid grid(0.5f);
    grid.rebuild(particles);

    const auto first = grid.findNeighborPairs(particles, 0.5f);
    const auto second = grid.findNeighborPairs(particles, 0.5f);

    REQUIRE(first == second);
    REQUIRE(std::is_sorted(first.begin(), first.end()));
    REQUIRE(std::adjacent_find(first.begin(), first.end()) == first.end());
    REQUIRE(first == BruteForcePairs(particles, 0.5f));
}

TEST_CASE("Fluid spatial grid matches brute force across cell sizes", "[fluid][neighbors][differential]") {
    const FluidParticleProperties properties;
    std::vector<FluidParticle> particles;
    for (int index = 0; index < 160; ++index) {
        const float x = std::sin(static_cast<float>(index) * 1.73f) * 6.0f
            + static_cast<float>(index % 7) * 0.03125f;
        const float y = std::cos(static_cast<float>(index) * 0.91f) * 4.0f
            - static_cast<float>(index % 5) * 0.0625f;
        particles.emplace_back(Vector2(x, y), Vector2(), properties);
    }

    for (float cellSize : {0.2f, 0.5f, 1.25f}) {
        FluidParticleSpatialGrid grid(cellSize);
        grid.rebuild(particles);
        for (float radius : {0.25f, 0.75f, 1.8f}) {
            REQUIRE(
                grid.findNeighborPairs(particles, radius)
                == BruteForcePairs(particles, radius)
            );
        }
    }
}

TEST_CASE("Fluid neighbor statistics describe the latest query", "[fluid][neighbors][statistics]") {
    const FluidParticleProperties properties;
    const std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(-0.5f, 0.0f), Vector2(), properties),
        FluidParticle(Vector2(0.0f, 0.0f), Vector2(), properties),
        FluidParticle(Vector2(0.5f, 0.0f), Vector2(), properties),
        FluidParticle(Vector2(5.0f, 5.0f), Vector2(), properties),
    };
    FluidParticleSpatialGrid grid(0.5f);
    grid.rebuild(particles);
    const auto pairs = grid.findNeighborPairs(particles, 0.5f);
    const FluidNeighborStatistics& statistics = grid.getLastStatistics();

    REQUIRE(pairs.size() == 2);
    REQUIRE(statistics.particleCount == 4);
    REQUIRE(statistics.occupiedCellCount == 4);
    REQUIRE(statistics.candidatePairCount >= statistics.neighborPairCount);
    REQUIRE(statistics.neighborPairCount == 2);
    REQUIRE(statistics.maximumNeighborCount == 2);
}
