#include "catch_amalgamated.hpp"

#include "physics/core/forces/gravity.h"
#include "physics/core/particles/particle_spatial_grid.h"
#include "physics/core/particles/particle_system.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/core/world.h"

#include <cmath>
#include <memory>
#include <random>
#include <set>
#include <utility>
#include <vector>

using namespace PhysicsEngine;

TEST_CASE("Long-running free motion remains finite", "[stress][integration]") {
    World world;
    Circle shape(0.5f);
    Material material{1.0f, 0.0f};
    auto body = std::make_shared<RigidBody>(&shape, material, Vector2(1.0f, -2.0f));
    body->SetVelocity(Vector2(0.125f, -0.25f));
    body->SetAngularVelocity(0.1f);
    world.addBody(body);

    for (int i = 0; i < 100000; ++i) {
        world.step(1.0f / 1000.0f);
    }

    REQUIRE(std::isfinite(body->GetPosition().x));
    REQUIRE(std::isfinite(body->GetPosition().y));
    REQUIRE(body->GetPosition().x == Catch::Approx(13.5f).margin(0.05f));
    REQUIRE(body->GetPosition().y == Catch::Approx(-27.0f).margin(0.05f));
}

TEST_CASE("A five-box stack settles without sinking or exploding", "[stress][collision]") {
    World world;
    auto floorShape = Polygon::MakeBox(20.0f, 1.0f);
    auto boxShape = Polygon::MakeBox(1.0f, 1.0f);
    Material material{1.0f, 0.0f, 0.7f, 0.5f};
    auto floor = std::make_shared<RigidBody>(
        &floorShape,
        material,
        Vector2(0.0f, -0.5f),
        true
    );
    world.addBody(floor);

    std::vector<RigidBodyPtr> boxes;
    for (int i = 0; i < 5; ++i) {
        boxes.push_back(std::make_shared<RigidBody>(
            &boxShape,
            material,
            Vector2(0.0f, 0.5f + static_cast<float>(i))
        ));
        world.addBody(boxes.back());
    }
    world.addUniversalForce(std::make_unique<Gravity>(Vector2(0.0f, -9.81f)));

    for (int i = 0; i < 1200; ++i) {
        world.step(1.0f / 120.0f);
    }

    for (std::size_t i = 0; i < boxes.size(); ++i) {
        CAPTURE(i);
        REQUIRE(boxes[i]->GetPosition().y == Catch::Approx(0.5f + static_cast<float>(i)).margin(0.35f));
        REQUIRE(std::abs(boxes[i]->GetVelocity().y) < 0.35f);
        REQUIRE(std::isfinite(boxes[i]->GetOrientation()));
    }
}

TEST_CASE("Particle spatial grid matches brute force neighborhoods", "[stress][ParticleSystem]") {
    std::mt19937 random(0x51A71A1u);
    std::uniform_real_distribution<float> coordinate(-20.0f, 20.0f);
    ParticleSystem system;
    for (int i = 0; i < 500; ++i) {
        system.addParticle(Vector2(coordinate(random), coordinate(random)));
    }

    constexpr float radius = 1.5f;
    const float radiusSquared = radius * radius;
    std::set<std::pair<std::size_t, std::size_t>> expected;
    const auto& particles = system.getParticles();
    for (std::size_t i = 0; i < particles.size(); ++i) {
        for (std::size_t j = i + 1; j < particles.size(); ++j) {
            if ((particles[j].position - particles[i].position).magnitudeSquared() <= radiusSquared) {
                expected.emplace(i, j);
            }
        }
    }

    ParticleSpatialGrid grid(0.75f);
    grid.rebuild(particles);
    const auto actualVector = grid.findPotentialPairs(particles, radius);
    const std::set<std::pair<std::size_t, std::size_t>> actual(
        actualVector.begin(),
        actualVector.end()
    );

    REQUIRE(actual == expected);
}

TEST_CASE("Ten thousand particles remain finite under uniform acceleration", "[stress][ParticleSystem]") {
    ParticleSystem system;
    system.reserve(10000);
    system.setUniformAcceleration(Vector2(0.0f, -9.81f));
    for (int i = 0; i < 10000; ++i) {
        system.addParticle(
            Vector2(static_cast<float>(i % 100), static_cast<float>(i / 100)),
            Vector2(0.5f, 0.0f),
            1.0f + static_cast<float>(i % 5)
        );
    }

    for (int step = 0; step < 600; ++step) {
        system.step(1.0f / 120.0f);
    }

    for (const Particle& particle : system.getParticles()) {
        REQUIRE(std::isfinite(particle.position.x));
        REQUIRE(std::isfinite(particle.position.y));
        REQUIRE(std::isfinite(particle.velocity.x));
        REQUIRE(std::isfinite(particle.velocity.y));
    }
}
