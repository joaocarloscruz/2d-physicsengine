#include "catch_amalgamated.hpp"

#include "physics/core/fluids/fluid_rigid_coupler.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace PhysicsEngine;

namespace {

constexpr float Pi = 3.14159265358979323846f;

Vector2 CoupledMomentum(
    const std::vector<FluidParticle>& particles,
    const RigidBody& body
) {
    Vector2 momentum = body.GetVelocity() * body.GetMass();
    for (const FluidParticle& particle : particles) {
        momentum = momentum + particle.velocity * particle.mass;
    }
    return momentum;
}

float HydrostaticBodyVelocity(float bodyDensity) {
    constexpr float fluidDensity = 1000.0f;
    constexpr float gravity = 9.81f;
    constexpr float radius = 0.5f;
    constexpr int samples = 128;
    constexpr float deltaTime = 0.001f;
    const float arcLength = 2.0f * Pi * radius / samples;

    Circle shape(radius);
    Material material{bodyDensity, 0.0f, 0.0f, 0.0f};
    RigidBody body(&shape, material, Vector2());
    FluidParticleProperties properties;
    properties.mass = fluidDensity * arcLength * arcLength;
    properties.restDensity = fluidDensity;
    properties.smoothingLength = arcLength;
    properties.viscosity = 0.0f;
    std::vector<FluidParticle> particles;
    particles.reserve(samples);
    for (int index = 0; index < samples; ++index) {
        const float angle = 2.0f * Pi * index / samples;
        const Vector2 position(
            radius * std::cos(angle),
            radius * std::sin(angle)
        );
        FluidParticle particle(position, Vector2(), properties);
        particle.density = fluidDensity;
        particle.volume = arcLength * arcLength;
        particle.pressure = fluidDensity * gravity * (1.0f - position.y);
        particles.push_back(particle);
    }

    FluidRigidCouplingSettings settings;
    settings.particleRadius = 0.0001f;
    settings.viscosityScale = 0.0f;
    FluidRigidCoupler coupler(settings);
    coupler.couple(particles, {&body}, deltaTime);
    body.ApplyImpulse(
        Vector2(0.0f, -body.GetMass() * gravity * deltaTime),
        Vector2()
    );
    return body.GetVelocity().y;
}

} // namespace

TEST_CASE("Fluid-rigid coupling applies equal-and-opposite impulses", "[fluid][coupling][momentum]") {
    auto shape = Polygon::MakeBox(1.0f, 1.0f);
    Material material{2.0f, 0.0f, 0.0f, 0.0f};
    RigidBody body(&shape, material, Vector2());
    FluidParticleProperties properties;
    properties.mass = 1.5f;
    properties.restDensity = 1000.0f;
    properties.smoothingLength = 0.4f;
    properties.viscosity = 0.3f;
    FluidParticle particle(
        Vector2(0.0f, -0.5f),
        Vector2(2.0f, 0.0f),
        properties
    );
    particle.pressure = 500.0f;
    std::vector<FluidParticle> particles = {particle};
    const Vector2 momentumBefore = CoupledMomentum(particles, body);
    FluidRigidCoupler coupler;

    const FluidRigidCouplingStatistics statistics = coupler.couple(
        particles,
        {&body},
        0.01f
    );

    const Vector2 momentumAfter = CoupledMomentum(particles, body);
    REQUIRE(statistics.interactionCount == 1);
    REQUIRE(statistics.pressureImpulseMagnitude > 0.0f);
    REQUIRE(statistics.viscosityImpulseMagnitude > 0.0f);
    REQUIRE(momentumAfter.x == Catch::Approx(momentumBefore.x).margin(1e-5f));
    REQUIRE(momentumAfter.y == Catch::Approx(momentumBefore.y).margin(1e-5f));
}

TEST_CASE("Off-center fluid pressure generates rigid-body torque", "[fluid][coupling][torque]") {
    auto shape = Polygon::MakeBox(1.0f, 1.0f);
    Material material{1.0f, 0.0f, 0.0f, 0.0f};
    RigidBody body(&shape, material, Vector2());
    FluidParticleProperties properties;
    properties.mass = 1.0f;
    properties.smoothingLength = 0.3f;
    properties.viscosity = 0.0f;
    FluidParticle particle(Vector2(0.4f, -0.5f), Vector2(), properties);
    particle.pressure = 1000.0f;
    std::vector<FluidParticle> particles = {particle};
    FluidRigidCouplingSettings settings;
    settings.particleRadius = 0.001f;
    settings.viscosityScale = 0.0f;
    FluidRigidCoupler coupler(settings);

    coupler.couple(particles, {&body}, 0.001f);

    REQUIRE(body.GetVelocity().y > 0.0f);
    REQUIRE(body.GetAngularVelocity() > 0.0f);
}

TEST_CASE("Hydrostatic coupling floats light bodies and sinks heavy bodies", "[fluid][coupling][buoyancy]") {
    REQUIRE(HydrostaticBodyVelocity(500.0f) > 0.0f);
    REQUIRE(HydrostaticBodyVelocity(1500.0f) < 0.0f);
}

TEST_CASE("Fluid-rigid contact prevents particle penetration without rebound", "[fluid][coupling][contact]") {
    Circle shape(0.5f);
    Material material{1.0f, 0.0f, 0.0f, 0.0f};
    RigidBody body(&shape, material, Vector2());
    FluidParticleProperties properties;
    properties.smoothingLength = 0.3f;
    FluidParticle particle(
        Vector2(0.45f, 0.0f),
        Vector2(-1.0f, 0.0f),
        properties
    );
    particle.pressure = 0.0f;
    std::vector<FluidParticle> particles = {particle};
    FluidRigidCouplingSettings settings;
    settings.particleRadius = 0.1f;
    settings.pressureScale = 0.0f;
    settings.viscosityScale = 0.0f;
    FluidRigidCoupler coupler(settings);

    const auto statistics = coupler.couple(particles, {&body}, 0.001f);

    REQUIRE(statistics.contactCorrectionCount == 1);
    REQUIRE(particles.front().position.x == Catch::Approx(0.6f));
    const float relativeNormalSpeed = (
        particles.front().velocity
        - body.GetVelocityAtPoint(Vector2(0.5f, 0.0f))
    ).x;
    REQUIRE(relativeNormalSpeed >= -1e-6f);
}

TEST_CASE("Fluid-rigid coupling rejects invalid numerical inputs", "[fluid][coupling][validation]") {
    FluidRigidCouplingSettings settings;
    settings.contactRestitution = 1.1f;
    REQUIRE_THROWS_AS(FluidRigidCoupler(settings), std::invalid_argument);

    Circle shape(0.5f);
    Material material{1.0f, 0.0f, 0.0f, 0.0f};
    RigidBody body(&shape, material, Vector2());
    std::vector<FluidParticle> particles = {FluidParticle()};
    FluidRigidCoupler coupler;
    REQUIRE_THROWS_AS(
        coupler.couple(particles, {&body}, -0.001f),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(
        coupler.couple(particles, {nullptr}, 0.001f),
        std::invalid_argument
    );

    particles.front().pressure = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(
        coupler.couple(particles, {&body}, 0.001f),
        std::invalid_argument
    );
}
