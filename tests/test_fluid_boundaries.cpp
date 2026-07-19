#include "catch_amalgamated.hpp"

#include "physics/core/fluids/fluid_boundary.h"
#include "physics/core/fluids/wcsph_solver.h"

#include <cmath>
#include <memory>
#include <vector>

using namespace PhysicsEngine;

namespace {

Vector2 FluidMomentum(const std::vector<FluidParticle>& particles) {
    Vector2 momentum;
    for (const FluidParticle& particle : particles) {
        momentum = momentum + particle.velocity * particle.mass;
    }
    return momentum;
}

std::vector<Vector2> SquareVertices(bool clockwise) {
    if (clockwise) {
        return {
            Vector2(-1.0f, -1.0f),
            Vector2(-1.0f, 1.0f),
            Vector2(1.0f, 1.0f),
            Vector2(1.0f, -1.0f),
        };
    }
    return {
        Vector2(-1.0f, -1.0f),
        Vector2(1.0f, -1.0f),
        Vector2(1.0f, 1.0f),
        Vector2(-1.0f, 1.0f),
    };
}

} // namespace

TEST_CASE("Circle fluid boundary removes outward motion without bouncing", "[fluid][boundary][circle]") {
    FluidBoundarySettings settings;
    settings.particleRadius = 0.1f;
    settings.restitution = 0.0f;
    settings.friction = 0.0f;
    FluidCircleContainer boundary(Vector2(), 1.0f, settings);
    FluidParticle particle(Vector2(1.2f, 0.0f), Vector2(3.0f, 2.0f));

    const FluidBoundaryCorrection correction = boundary.enforce(particle);

    REQUIRE(correction.corrected);
    REQUIRE(correction.penetration == Catch::Approx(0.3f));
    REQUIRE(particle.position.x == Catch::Approx(0.9f));
    REQUIRE(particle.position.y == Catch::Approx(0.0f));
    REQUIRE(particle.velocity.x == Catch::Approx(0.0f));
    REQUIRE(particle.velocity.y == Catch::Approx(2.0f));
    REQUIRE(boundary.contains(particle.position));
}

TEST_CASE("Convex polygon fluid boundary handles winding and corners", "[fluid][boundary][polygon]") {
    FluidBoundarySettings settings;
    settings.particleRadius = 0.1f;
    settings.restitution = 0.0f;
    settings.friction = 0.0f;

    for (bool clockwise : {false, true}) {
        FluidConvexPolygonContainer boundary(
            SquareVertices(clockwise),
            settings
        );
        FluidParticle particle(
            Vector2(1.2f, 1.3f),
            Vector2(2.0f, 3.0f)
        );

        const FluidBoundaryCorrection correction = boundary.enforce(particle);

        REQUIRE(correction.corrected);
        REQUIRE(boundary.contains(particle.position));
        REQUIRE(particle.position.x == Catch::Approx(0.9f));
        REQUIRE(particle.position.y == Catch::Approx(0.9f));
        REQUIRE(particle.velocity.x == Catch::Approx(0.0f));
        REQUIRE(particle.velocity.y == Catch::Approx(0.0f));
    }
}

TEST_CASE("Symmetric boundary corrections introduce no net fluid momentum", "[fluid][boundary][momentum]") {
    FluidBoundarySettings settings;
    settings.particleRadius = 0.1f;
    settings.restitution = 0.0f;
    settings.friction = 0.0f;
    FluidCircleContainer boundary(Vector2(), 1.0f, settings);
    std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(-1.1f, 0.0f), Vector2(-2.0f, 0.0f)),
        FluidParticle(Vector2(1.1f, 0.0f), Vector2(2.0f, 0.0f)),
    };

    const FluidBoundaryStatistics statistics = EnforceFluidBoundary(
        boundary,
        particles
    );

    const Vector2 momentum = FluidMomentum(particles);
    REQUIRE(statistics.correctedParticleCount == 2);
    REQUIRE(momentum.x == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(momentum.y == Catch::Approx(0.0f).margin(1e-6f));
}

TEST_CASE("WCSPH polygon container does not leak during a long run", "[fluid][boundary][wcsph][stability]") {
    FluidBoundarySettings settings;
    settings.particleRadius = 0.08f;
    settings.restitution = 0.0f;
    settings.friction = 0.08f;
    FluidConvexPolygonContainer boundary(SquareVertices(false), settings);

    FluidParticleProperties properties;
    properties.mass = properties.restDensity * 0.2f * 0.2f;
    properties.smoothingLength = 0.4f;
    properties.viscosity = 0.08f;
    std::vector<FluidParticle> particles;
    for (int row = 0; row < 7; ++row) {
        for (int column = 0; column < 7; ++column) {
            particles.emplace_back(
                Vector2(
                    -0.6f + column * 0.2f,
                    -0.6f + row * 0.2f
                ),
                Vector2(),
                properties
            );
        }
    }
    WcsphConfig config;
    config.speedOfSound = 15.0f;
    WcsphSolver solver(properties.smoothingLength, config);

    for (int step = 0; step < 2000; ++step) {
        solver.step(particles, 0.002f, boundary);
    }

    for (const FluidParticle& particle : particles) {
        REQUIRE(boundary.contains(particle.position));
        REQUIRE(std::isfinite(particle.velocity.x));
        REQUIRE(std::isfinite(particle.velocity.y));
    }
    REQUIRE(solver.getLastStatistics().boundaryCorrectionCount > 0);
}

TEST_CASE("A resting particle does not rebound from the floor", "[fluid][boundary][wcsph][resting]") {
    FluidBoundarySettings settings;
    settings.particleRadius = 0.1f;
    settings.restitution = 0.0f;
    settings.friction = 0.0f;
    FluidConvexPolygonContainer boundary(SquareVertices(false), settings);
    FluidParticleProperties properties;
    properties.smoothingLength = 0.4f;
    std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(0.0f, 0.0f), Vector2(), properties)
    };
    WcsphSolver solver(properties.smoothingLength);

    for (int step = 0; step < 1000; ++step) {
        solver.step(particles, 0.002f, boundary);
    }

    REQUIRE(particles.front().position.y == Catch::Approx(-0.9f));
    REQUIRE(std::abs(particles.front().velocity.y) < 1e-6f);
}

TEST_CASE("Fluid boundaries reject invalid geometry and response settings", "[fluid][boundary][validation]") {
    FluidBoundarySettings settings;

    SECTION("particle radius must be positive") {
        settings.particleRadius = 0.0f;
        REQUIRE_THROWS_AS(
            FluidCircleContainer(Vector2(), 1.0f, settings),
            std::invalid_argument
        );
    }
    SECTION("circle must contain a particle center region") {
        settings.particleRadius = 1.0f;
        REQUIRE_THROWS_AS(
            FluidCircleContainer(Vector2(), 1.0f, settings),
            std::invalid_argument
        );
    }
    SECTION("polygon must be strictly convex") {
        REQUIRE_THROWS_AS(
            FluidConvexPolygonContainer(
                {
                    Vector2(-1.0f, -1.0f),
                    Vector2(1.0f, -1.0f),
                    Vector2(0.0f, 0.0f),
                    Vector2(1.0f, 1.0f),
                    Vector2(-1.0f, 1.0f),
                },
                settings
            ),
            std::invalid_argument
        );
    }
}
