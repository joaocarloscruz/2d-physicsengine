#include "catch_amalgamated.hpp"

#include "physics/core/particles/particle.h"
#include "physics/core/particles/particle_spatial_grid.h"
#include "physics/core/particles/particle_system.h"
#include "physics/core/world.h"

#include <cmath>
#include <memory>
#include <stdexcept>

using namespace PhysicsEngine;

TEST_CASE("Particle integrates force without rigid-body state", "[Particle]") {
    Particle particle(Vector2(0.0f, 0.0f), Vector2(0.0f, 0.0f), 1.0f);
    particle.ApplyForce(Vector2(2.0f, 0.0f));

    particle.Integrate(1.0f);

    REQUIRE(particle.position.x == Catch::Approx(1.0f));
    REQUIRE(particle.velocity.x == Catch::Approx(2.0f));
    REQUIRE(particle.force.magnitudeSquared() == Catch::Approx(0.0f));
}

TEST_CASE("Particle requires positive finite mass", "[Particle]") {
    REQUIRE_THROWS_AS(
        Particle(Vector2(), Vector2(), 0.0f),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(
        Particle(Vector2(), Vector2(), INFINITY),
        std::invalid_argument
    );
}

TEST_CASE("ParticleSystem manages and steps a contiguous particle collection", "[ParticleSystem]") {
    ParticleSystem system;
    system.reserve(10000);
    system.setUniformAcceleration(Vector2(0.0f, -10.0f));

    for (std::size_t i = 0; i < 10000; ++i) {
        system.addParticle(Vector2(static_cast<float>(i), 0.0f), Vector2(), 2.0f);
    }

    system.step(0.5f);

    REQUIRE(system.size() == 10000);
    REQUIRE(system.getParticles().front().position.y == Catch::Approx(-1.25f));
    REQUIRE(system.getParticles().front().velocity.y == Catch::Approx(-5.0f));
    REQUIRE(system.getParticles().back().position.x == Catch::Approx(9999.0f));
}

TEST_CASE("ParticleSystem validates indexes and removes particles", "[ParticleSystem]") {
    ParticleSystem system;
    const std::size_t first = system.addParticle(Vector2(1.0f, 0.0f));
    system.addParticle(Vector2(2.0f, 0.0f));

    system.applyForce(first, Vector2(1.0f, 0.0f));
    system.removeParticle(first);

    REQUIRE(system.size() == 1);
    REQUIRE(system.getParticles().front().position.x == Catch::Approx(2.0f));
    REQUIRE_THROWS_AS(system.applyForce(10, Vector2()), std::out_of_range);
}

TEST_CASE("World owns the particle-system simulation schedule", "[ParticleSystem][World]") {
    World world;
    auto system = std::make_shared<ParticleSystem>();
    system->addParticle(Vector2(), Vector2(4.0f, 0.0f));
    world.addParticleSystem(system);

    world.step(0.25f);

    REQUIRE(system->getParticles().front().position.x == Catch::Approx(1.0f));
    REQUIRE(world.getParticleSystems().size() == 1);

    world.removeParticleSystem(system);
    REQUIRE(world.getParticleSystems().empty());
}

TEST_CASE("ParticleSpatialGrid finds unique nearby pairs across negative cells", "[ParticleSystem][BroadPhase]") {
    ParticleSystem system;
    system.addParticle(Vector2(-0.2f, 0.0f));
    system.addParticle(Vector2(0.2f, 0.0f));
    system.addParticle(Vector2(10.0f, 10.0f));

    ParticleSpatialGrid grid(0.25f);
    grid.rebuild(system.getParticles());
    const auto pairs = grid.findPotentialPairs(system.getParticles(), 0.5f);

    REQUIRE(pairs.size() == 1);
    REQUIRE(pairs.front().first == 0);
    REQUIRE(pairs.front().second == 1);
}

TEST_CASE("ParticleSpatialGrid validates spatial parameters", "[ParticleSystem][BroadPhase]") {
    REQUIRE_THROWS_AS(ParticleSpatialGrid(0.0f), std::invalid_argument);

    ParticleSpatialGrid grid(1.0f);
    const std::vector<Particle> particles;
    grid.rebuild(particles);
    REQUIRE_THROWS_AS(grid.findPotentialPairs(particles, 0.0f), std::invalid_argument);
}
