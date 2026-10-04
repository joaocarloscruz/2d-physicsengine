#include "catch_amalgamated.hpp"

#include "physics/core/particles/particle.h"
#include "physics/core/particles/particle_spatial_grid.h"
#include "physics/core/particles/particle_system.h"
#include "physics/core/world.h"

#include <cmath>
#include <memory>
#include <stdexcept>

using namespace PhysicsEngine;

TEST_CASE("Particle integration combines extreme acceleration and tiny time in double", "[particle-numerics]") {
    Particle p({}, {}, 1e-20f);
    p.ApplyForce({1e30f, -1e30f});
    const float dt = 1e-30f;
    const double ax = static_cast<double>(p.force.x) / p.mass;
    p.Integrate(dt);
    REQUIRE(p.position.x == Catch::Approx(0.5 * ax * dt * dt).epsilon(1e-6));
    REQUIRE(p.position.y == Catch::Approx(-0.5 * ax * dt * dt).epsilon(1e-6));
    REQUIRE(p.velocity.x == Catch::Approx(ax * dt).epsilon(1e-6));
    REQUIRE(p.velocity.y == Catch::Approx(-ax * dt).epsilon(1e-6));
    REQUIRE(p.force.x == 0);
}

TEST_CASE("Particle overflow retains its complete state and queued forces", "[particle-numerics]") {
    const float large = std::numeric_limits<float>::max();
    Particle p({1, 2}, {large, 3}, 1);
    p.ApplyForce({large, 4});
    REQUIRE_THROWS_AS(p.ApplyForce({large, 5}), std::overflow_error);
    REQUIRE(p.force.x == large);
    REQUIRE(p.force.y == 4);
    REQUIRE_THROWS_AS(p.Integrate(2), std::overflow_error);
    REQUIRE(p.position.x == 1);
    REQUIRE(p.position.y == 2);
    REQUIRE(p.velocity.x == large);
    REQUIRE(p.velocity.y == 3);
    REQUIRE(p.force.x == large);
    REQUIRE(p.force.y == 4);
    REQUIRE(p.inverseMass == 1);
}

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


TEST_CASE("Repeated particle system registration does not double integrate", "[review][particles]") {
    World world;
    auto particles = std::make_shared<ParticleSystem>();
    particles->addParticle({}, {1, 0}, 1);
    world.addParticleSystem(particles); world.addParticleSystem(particles);
    world.step(0.1f);
    REQUIRE(world.getParticleSystems().size() == 1);
    REQUIRE(particles->getParticles()[0].position.x == Catch::Approx(0.1f));
    REQUIRE(world.getLastStepStatistics().integratedParticleCount == 1);
}

TEST_CASE("Particle APIs reject invalid state before changing it", "[review][particles]") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    CHECK_THROWS_AS(Particle(Vector2(nan, 0)), std::invalid_argument);
    CHECK_THROWS_AS(Particle({}, Vector2(0, nan)), std::invalid_argument);
    Particle particle;
    CHECK_THROWS_AS(particle.ApplyForce({nan, 0}), std::invalid_argument);
    ParticleSystem system;
    CHECK_THROWS_AS(system.setUniformAcceleration({0, nan}), std::invalid_argument);
    CHECK_THROWS_AS(system.step(-1), std::invalid_argument);
    CHECK_THROWS_AS(system.step(nan), std::invalid_argument);
}
