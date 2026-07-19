#include "catch_amalgamated.hpp"

#include "physics/core/fluids/coupled_fluid_simulation.h"
#include "physics/core/shape.h"

#include <vector>

using namespace PhysicsEngine;

namespace {

float RunTankBody(float bodyDensity) {
    constexpr float spacing = 0.15f;
    constexpr float smoothingLength = 0.3f;
    constexpr float bodyRadius = 0.3f;
    FluidBoundarySettings boundarySettings;
    boundarySettings.particleRadius = spacing * 0.5f;
    FluidConvexPolygonContainer tank(
        {
            Vector2(-1.5f, 0.0f),
            Vector2(1.5f, 0.0f),
            Vector2(1.5f, 3.0f),
            Vector2(-1.5f, 3.0f),
        },
        boundarySettings
    );
    Circle shape(bodyRadius);
    Material material{bodyDensity, 0.0f, 0.0f, 0.0f};
    RigidBody body(&shape, material, Vector2(0.0f, 1.0f));
    FluidParticleProperties properties;
    properties.mass = properties.restDensity * spacing * spacing;
    properties.smoothingLength = smoothingLength;
    properties.viscosity = 0.08f;
    std::vector<FluidParticle> particles;
    for (float y = spacing; y <= 1.8f; y += spacing) {
        for (float x = -1.35f; x <= 1.35f; x += spacing) {
            if ((Vector2(x, y) - body.GetPosition()).magnitude()
                > bodyRadius + boundarySettings.particleRadius) {
                particles.emplace_back(Vector2(x, y), Vector2(), properties);
            }
        }
    }
    WcsphConfig fluidConfig;
    fluidConfig.speedOfSound = 15.0f;
    fluidConfig.maximumTimeStep = 0.003f;
    FluidRigidCouplingSettings couplingSettings;
    couplingSettings.particleRadius = boundarySettings.particleRadius;
    CoupledFluidSimulation simulation(
        smoothingLength,
        fluidConfig,
        couplingSettings
    );
    for (int step = 0; step < 200; ++step) {
        simulation.step(particles, {&body}, 0.002f, tank);
    }
    REQUIRE(std::isfinite(body.GetPosition().y));
    REQUIRE(std::isfinite(body.GetVelocity().y));
    return body.GetPosition().y;
}

} // namespace

TEST_CASE("Coupled simulation advances rigid bodies on every fluid substep", "[fluid][coupled][substep]") {
    WcsphConfig fluidConfig;
    fluidConfig.externalAcceleration = Vector2(0.0f, -10.0f);
    fluidConfig.maximumTimeStep = 0.01f;
    fluidConfig.speedOfSound = 1.0f;
    CoupledFluidSimulation simulation(0.5f, fluidConfig);
    Circle shape(0.5f);
    Material material{1.0f, 0.0f, 0.0f, 0.0f};
    RigidBody body(&shape, material, Vector2());
    std::vector<FluidParticle> particles;

    simulation.step(particles, {&body}, 0.025f);

    const CoupledFluidStatistics& statistics = simulation.getLastStatistics();
    REQUIRE(statistics.fluid.substepCount == 3);
    REQUIRE(statistics.rigidBodySubstepCount == 3);
    REQUIRE(statistics.integratedTime == Catch::Approx(0.025f));
    REQUIRE(body.GetVelocity().y == Catch::Approx(-0.25f));
}

TEST_CASE("Coupled simulation preserves zero-restitution fluid contact across substeps", "[fluid][coupled][contact]") {
    WcsphConfig fluidConfig;
    fluidConfig.externalAcceleration = Vector2();
    fluidConfig.speedOfSound = 2.0f;
    fluidConfig.maximumTimeStep = 0.005f;
    FluidRigidCouplingSettings couplingSettings;
    couplingSettings.particleRadius = 0.1f;
    couplingSettings.pressureScale = 0.0f;
    couplingSettings.viscosityScale = 0.0f;
    CoupledFluidSimulation simulation(
        0.3f,
        fluidConfig,
        couplingSettings
    );
    Circle shape(0.5f);
    Material material{1.0f, 0.0f, 0.0f, 0.0f};
    RigidBody body(&shape, material, Vector2());
    FluidParticleProperties properties;
    properties.smoothingLength = 0.3f;
    std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(0.45f, 0.0f), Vector2(-1.0f, 0.0f), properties)
    };

    simulation.step(particles, {&body}, 0.01f);

    REQUIRE(simulation.getLastStatistics().coupling.contactCorrectionCount > 0);
    REQUIRE(
        particles.front().position.x - body.GetPosition().x
        >= 0.6f - 1e-5f
    );
    REQUIRE((particles.front().velocity
        - body.GetVelocityAtPoint(Vector2(0.5f, 0.0f))).x >= -1e-5f);
}

TEST_CASE("Coupled tank keeps a light body above a denser body", "[fluid][coupled][buoyancy][benchmark]") {
    const float lightBodyHeight = RunTankBody(500.0f);
    const float denseBodyHeight = RunTankBody(1500.0f);

    INFO("light height: " << lightBodyHeight);
    INFO("dense height: " << denseBodyHeight);
    REQUIRE(lightBodyHeight > denseBodyHeight + 0.05f);
}
