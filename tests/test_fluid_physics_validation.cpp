#include "catch_amalgamated.hpp"

#include "physics/core/fluids/fluid_boundary.h"
#include "physics/core/fluids/fluid_rigid_coupler.h"
#include "physics/core/fluids/wcsph_solver.h"
#include "physics/core/shape.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

using namespace PhysicsEngine;

namespace {

constexpr float Pi = 3.14159265358979323846f;

std::vector<FluidParticle> MakeLattice(
    int columns,
    int rows,
    float spacing,
    float smoothingLength,
    const Vector2& origin = Vector2(),
    const Vector2& velocity = Vector2(),
    float massScale = 1.0f
) {
    FluidParticleProperties properties;
    properties.mass = properties.restDensity * spacing * spacing * massScale;
    properties.smoothingLength = smoothingLength;
    properties.viscosity = 0.05f;
    std::vector<FluidParticle> particles;
    particles.reserve(static_cast<std::size_t>(columns * rows));
    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < columns; ++column) {
            particles.emplace_back(
                origin + Vector2(column * spacing, row * spacing),
                velocity,
                properties
            );
        }
    }
    return particles;
}

struct RestStateMetrics {
    float maximumSpeed = 0.0f;
    float maximumDisplacement = 0.0f;
    Vector2 totalMomentum;
};

struct HydrostaticTrace {
    float maximumSpeed = 0.0f;
    float maximumDensityError = 0.0f;
    float maximumAcceleration = 0.0f;
    float maximumForceCriterionViolation = 0.0f;
};

HydrostaticTrace TraceHydrostaticColumn(float outerTimeStep, float duration) {
    constexpr int columns = 19;
    constexpr int rows = 15;
    constexpr float spacing = 0.1f;
    constexpr float smoothingLength = 0.2f;
    constexpr float massScale = 1.0f / 1.014612675f;
    FluidBoundarySettings boundarySettings;
    boundarySettings.particleRadius = 0.05f;
    boundarySettings.friction = 0.0f;
    FluidConvexPolygonContainer tank(
        {
            Vector2(-1.0f, 0.0f),
            Vector2(1.0f, 0.0f),
            Vector2(1.0f, 2.0f),
            Vector2(-1.0f, 2.0f),
        },
        boundarySettings
    );
    FluidBoundarySamplingSettings sampling;
    sampling.spacing = spacing;
    sampling.supportRadius = smoothingLength;
    auto boundaryParticles = SampleFluidContainerBoundary(tank, sampling);
    for (FluidBoundaryParticle& boundaryParticle : boundaryParticles) {
        boundaryParticle.volume *= massScale;
    }
    auto particles = MakeLattice(
        columns,
        rows,
        spacing,
        smoothingLength,
        Vector2(-0.9f, 0.1f),
        Vector2(),
        massScale
    );
    WcsphConfig config;
    config.speedOfSound = 40.0f;
    config.maximumTimeStep = outerTimeStep;
    WcsphSolver solver(smoothingLength, config);
    const WcsphSolver::SubstepCallback noOp = [](float) {};
    const int steps = static_cast<int>(std::round(duration / outerTimeStep));
    HydrostaticTrace trace;
    for (int step = 0; step < steps; ++step) {
        solver.step(
            particles, outerTimeStep, tank, boundaryParticles, noOp
        );
        float stepMaximumAcceleration = 0.0f;
        for (const FluidParticle& particle : particles) {
            trace.maximumSpeed = std::max(
                trace.maximumSpeed, particle.velocity.magnitude()
            );
            trace.maximumDensityError = std::max(
                trace.maximumDensityError,
                std::abs(particle.density / particle.restDensity - 1.0f)
            );
            stepMaximumAcceleration = std::max(
                stepMaximumAcceleration,
                particle.force.magnitude() * particle.inverseMass
            );
        }
        trace.maximumAcceleration = std::max(
            trace.maximumAcceleration, stepMaximumAcceleration
        );
        if (stepMaximumAcceleration > 0.0f) {
            const float forceLimit = config.cflFactor * std::sqrt(
                smoothingLength / stepMaximumAcceleration
            );
            trace.maximumForceCriterionViolation = std::max(
                trace.maximumForceCriterionViolation,
                solver.getLastStatistics().stableTimeStep / forceLimit
            );
        }
    }
    return trace;
}

RestStateMetrics SimulateRestState(
    float massScale,
    float deltaTime,
    float duration = 0.1f
) {
    constexpr int size = 21;
    auto particles = MakeLattice(
        size, size, 0.1f, 0.2f, Vector2(), Vector2(), massScale
    );
    const auto initial = particles;
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    config.speedOfSound = 15.0f;
    config.maximumTimeStep = deltaTime;
    WcsphSolver solver(0.2f, config);
    const int steps = static_cast<int>(std::round(duration / deltaTime));
    for (int step = 0; step < steps; ++step) {
        solver.step(particles, deltaTime);
    }

    RestStateMetrics result;
    for (std::size_t index = 0; index < particles.size(); ++index) {
        result.maximumSpeed = std::max(
            result.maximumSpeed,
            particles[index].velocity.magnitude()
        );
        result.maximumDisplacement = std::max(
            result.maximumDisplacement,
            (particles[index].position - initial[index].position).magnitude()
        );
        result.totalMomentum = result.totalMomentum
            + particles[index].velocity * particles[index].mass;
    }
    return result;
}

float MeanDensityRatio(
    const std::vector<FluidParticle>& particles,
    int columns,
    int firstColumn,
    int lastColumn,
    int firstRow,
    int lastRow
) {
    double total = 0.0;
    std::size_t count = 0;
    for (int row = firstRow; row <= lastRow; ++row) {
        for (int column = firstColumn; column <= lastColumn; ++column) {
            const FluidParticle& particle = particles[
                static_cast<std::size_t>(row * columns + column)
            ];
            total += particle.density / particle.restDensity;
            ++count;
        }
    }
    return static_cast<float>(total / static_cast<double>(count));
}

} // namespace

TEST_CASE("Uniform SPH lattice has translation-invariant interior density", "[fluid][validation][consistency][!mayfail]") {
    constexpr int size = 21;
    constexpr float spacing = 0.1f;
    constexpr float smoothingLength = 0.2f;
    auto original = MakeLattice(size, size, spacing, smoothingLength);
    auto translated = MakeLattice(
        size,
        size,
        spacing,
        smoothingLength,
        Vector2(0.037f, -0.061f)
    );
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    WcsphSolver solver(smoothingLength, config);

    solver.prepare(original);
    solver.prepare(translated);

    const float originalMean = MeanDensityRatio(
        original, size, 3, size - 4, 3, size - 4
    );
    const float translatedMean = MeanDensityRatio(
        translated, size, 3, size - 4, 3, size - 4
    );
    REQUIRE(std::abs(originalMean - 1.0f) < 0.01f);
    REQUIRE(translatedMean == Catch::Approx(originalMean).margin(1e-5f));
}

TEST_CASE("Discrete density calibration is resolution-ratio dependent", "[fluid][validation][consistency][resolution]") {
    constexpr int size = 25;
    constexpr float spacing = 0.1f;
    for (const float smoothingRatio : {1.5f, 2.0f, 2.5f}) {
        DYNAMIC_SECTION("h/dx = " << smoothingRatio) {
            const float smoothingLength = spacing * smoothingRatio;
            auto nominal = MakeLattice(size, size, spacing, smoothingLength);
            WcsphConfig config;
            config.externalAcceleration = Vector2();
            WcsphSolver solver(smoothingLength, config);
            solver.prepare(nominal);
            const float nominalRatio = MeanDensityRatio(
                nominal, size, 4, size - 5, 4, size - 5
            );
            auto calibrated = MakeLattice(
                size,
                size,
                spacing,
                smoothingLength,
                Vector2(),
                Vector2(),
                1.0f / nominalRatio
            );
            solver.prepare(calibrated);
            const float calibratedRatio = MeanDensityRatio(
                calibrated, size, 4, size - 5, 4, size - 5
            );
            INFO("nominal density ratio: " << nominalRatio);
            INFO("mass correction: " << 1.0f / nominalRatio);
            INFO("calibrated density ratio: " << calibratedRatio);
            REQUIRE(calibratedRatio == Catch::Approx(1.0f).margin(2e-5f));
        }
    }
}

TEST_CASE("Open free surface is distinguished from bulk density", "[fluid][validation][free-surface]") {
    constexpr int size = 21;
    auto particles = MakeLattice(size, size, 0.1f, 0.2f);
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    WcsphSolver solver(0.2f, config);
    solver.prepare(particles);

    const float bulk = MeanDensityRatio(particles, size, 3, 17, 3, 17);
    const float surface = MeanDensityRatio(particles, size, 3, 17, 20, 20);

    REQUIRE(bulk > 0.99f);
    REQUIRE(surface < bulk * 0.8f);
}

TEST_CASE("Compressed SPH pair produces equal repulsive pressure forces", "[fluid][validation][pressure]") {
    FluidParticleProperties properties;
    properties.mass = 1.0f;
    properties.restDensity = 1.0f;
    properties.smoothingLength = 1.0f;
    properties.viscosity = 0.0f;
    std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(-0.1f, 0.0f), Vector2(), properties),
        FluidParticle(Vector2(0.1f, 0.0f), Vector2(), properties),
    };
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    config.speedOfSound = 2.0f;
    WcsphSolver solver(1.0f, config);

    solver.prepare(particles);

    REQUIRE(particles[0].pressure > 0.0f);
    REQUIRE(particles[0].force.x < 0.0f);
    REQUIRE(particles[1].force.x > 0.0f);
    REQUIRE(particles[0].force.x
        == Catch::Approx(-particles[1].force.x).margin(1e-6f));
    REQUIRE(particles[0].force.y
        == Catch::Approx(-particles[1].force.y).margin(1e-6f));
}

TEST_CASE("Nominal zero-gravity fluid block remains at rest", "[fluid][validation][equilibrium][!mayfail]") {
    const RestStateMetrics result = SimulateRestState(1.0f, 1.0f / 240.0f);
    INFO("maximum speed: " << result.maximumSpeed);
    INFO("maximum displacement: " << result.maximumDisplacement);
    INFO("total momentum: " << result.totalMomentum.magnitude());
    REQUIRE(result.totalMomentum.magnitude() < 1e-3f);
    REQUIRE(result.maximumSpeed < 0.05f);
    REQUIRE(result.maximumDisplacement < 0.005f);
}

TEST_CASE("Density-calibrated particle mass improves rest equilibrium", "[fluid][validation][equilibrium][calibration]") {
    constexpr float nominalBulkDensityRatio = 1.014612675f;
    const RestStateMetrics nominal = SimulateRestState(1.0f, 1.0f / 480.0f);
    const RestStateMetrics calibrated = SimulateRestState(
        1.0f / nominalBulkDensityRatio,
        1.0f / 480.0f
    );
    INFO("nominal maximum speed: " << nominal.maximumSpeed);
    INFO("calibrated maximum speed: " << calibrated.maximumSpeed);
    INFO("nominal displacement: " << nominal.maximumDisplacement);
    INFO("calibrated displacement: " << calibrated.maximumDisplacement);
    REQUIRE(calibrated.maximumSpeed < nominal.maximumSpeed * 0.5f);
    REQUIRE(calibrated.maximumDisplacement < nominal.maximumDisplacement * 0.5f);
}

TEST_CASE("Calibrated lattice heals small positional disorder", "[fluid][validation][equilibrium][disorder][!mayfail]") {
    constexpr int size = 21;
    constexpr float spacing = 0.1f;
    constexpr float massScale = 1.0f / 1.014612675f;
    auto particles = MakeLattice(
        size, size, spacing, 0.2f, Vector2(), Vector2(), massScale
    );
    const auto nominal = particles;
    for (int row = 0; row < size; ++row) {
        for (int column = 0; column < size; ++column) {
            FluidParticle& particle = particles[row * size + column];
            particle.position.x += ((row + column) % 2 == 0 ? 1.0f : -1.0f)
                * spacing * 0.02f;
            particle.position.y += ((row * 3 + column) % 2 == 0 ? 1.0f : -1.0f)
                * spacing * 0.02f;
        }
    }
    const auto disturbed = particles;
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    config.speedOfSound = 15.0f;
    config.maximumTimeStep = 1.0f / 480.0f;
    WcsphSolver solver(0.2f, config);
    for (int step = 0; step < 96; ++step) {
        solver.step(particles, 1.0f / 480.0f);
    }
    float maximumSpeed = 0.0f;
    float meanSpeed = 0.0f;
    float initialRmsDisorder = 0.0f;
    float finalRmsDisorder = 0.0f;
    for (std::size_t index = 0; index < particles.size(); ++index) {
        const FluidParticle& particle = particles[index];
        maximumSpeed = std::max(maximumSpeed, particle.velocity.magnitude());
        meanSpeed += particle.velocity.magnitude();
        initialRmsDisorder += (
            nominal[index].position - disturbed[index].position
        ).magnitudeSquared();
        finalRmsDisorder += (
            nominal[index].position - particle.position
        ).magnitudeSquared();
    }
    meanSpeed /= static_cast<float>(particles.size());
    initialRmsDisorder = std::sqrt(
        initialRmsDisorder / static_cast<float>(particles.size())
    );
    finalRmsDisorder = std::sqrt(
        finalRmsDisorder / static_cast<float>(particles.size())
    );
    INFO("maximum speed after 0.2 s: " << maximumSpeed);
    INFO("mean speed after 0.2 s: " << meanSpeed);
    INFO("initial RMS disorder: " << initialRmsDisorder);
    INFO("final RMS disorder: " << finalRmsDisorder);
    REQUIRE(finalRmsDisorder < initialRmsDisorder * 0.9f);
}

TEST_CASE("Rest-state error is dominated by spatial bias, not timestep", "[fluid][validation][equilibrium][convergence]") {
    const RestStateMetrics coarse = SimulateRestState(1.0f, 1.0f / 240.0f);
    const RestStateMetrics medium = SimulateRestState(1.0f, 1.0f / 480.0f);
    const RestStateMetrics fine = SimulateRestState(1.0f, 1.0f / 960.0f);
    const float coarseToMedium = std::abs(
        coarse.maximumDisplacement - medium.maximumDisplacement
    );
    const float mediumToFine = std::abs(
        medium.maximumDisplacement - fine.maximumDisplacement
    );
    INFO("coarse displacement: " << coarse.maximumDisplacement);
    INFO("medium displacement: " << medium.maximumDisplacement);
    INFO("fine displacement: " << fine.maximumDisplacement);
    INFO("coarse-medium difference: " << coarseToMedium);
    INFO("medium-fine difference: " << mediumToFine);
    REQUIRE(fine.maximumDisplacement > 0.015f);
    REQUIRE(std::abs(fine.maximumDisplacement - coarse.maximumDisplacement)
        / coarse.maximumDisplacement < 0.02f);
}

TEST_CASE("WCSPH stable timestep includes the force criterion", "[fluid][validation][timestep]") {
    FluidParticleProperties properties;
    properties.mass = 10.0f;
    properties.restDensity = 1.0f;
    properties.smoothingLength = 1.0f;
    properties.viscosity = 0.0f;
    std::vector<FluidParticle> particles = {
        FluidParticle(Vector2(-0.005f, 0.0f), Vector2(), properties),
        FluidParticle(Vector2(0.005f, 0.0f), Vector2(), properties),
    };
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    config.speedOfSound = 1.0f;
    config.maximumTimeStep = 0.1f;
    config.cflFactor = 0.25f;
    WcsphSolver solver(1.0f, config);
    solver.prepare(particles);
    const float maximumAcceleration = std::max(
        particles[0].force.magnitude() * particles[0].inverseMass,
        particles[1].force.magnitude() * particles[1].inverseMass
    );
    const float forceLimit = config.cflFactor * std::sqrt(
        properties.smoothingLength / maximumAcceleration
    );

    INFO("selected timestep: " << solver.getLastStatistics().stableTimeStep);
    INFO("force timestep: " << forceLimit);
    REQUIRE(solver.getLastStatistics().stableTimeStep <= forceLimit);
}

TEST_CASE("Hydrostatic surface integration recovers Archimedes force", "[fluid][validation][coupling][archimedes]") {
    constexpr float fluidDensity = 1000.0f;
    constexpr float gravity = 9.81f;
    constexpr float radius = 0.5f;
    constexpr int sampleCount = 256;
    constexpr float deltaTime = 0.0005f;
    const float arcLength = 2.0f * Pi * radius / sampleCount;
    Circle shape(radius);
    Material material{fluidDensity, 0.0f, 0.0f, 0.0f};
    RigidBody body(&shape, material, Vector2());
    FluidParticleProperties properties;
    properties.mass = fluidDensity * arcLength * arcLength;
    properties.restDensity = fluidDensity;
    properties.smoothingLength = arcLength;
    properties.viscosity = 0.0f;
    std::vector<FluidParticle> particles;
    particles.reserve(sampleCount);
    for (int index = 0; index < sampleCount; ++index) {
        const float angle = 2.0f * Pi * index / sampleCount;
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
    settings.particleRadius = 0.00001f;
    settings.viscosityScale = 0.0f;
    FluidRigidCoupler coupler(settings);

    coupler.couple(particles, {&body}, deltaTime);

    const Vector2 hydrodynamicForce = body.GetVelocity()
        * (body.GetMass() / deltaTime);
    const float expectedBuoyancy = fluidDensity * gravity * Pi * radius * radius;
    INFO("measured buoyancy: " << hydrodynamicForce.y);
    INFO("expected buoyancy: " << expectedBuoyancy);
    REQUIRE(hydrodynamicForce.x == Catch::Approx(0.0f).margin(0.1f));
    REQUIRE(hydrodynamicForce.y
        == Catch::Approx(expectedBuoyancy).epsilon(0.01f));

    body.ApplyImpulse(
        Vector2(0.0f, -body.GetMass() * gravity * deltaTime),
        Vector2()
    );
    REQUIRE(body.GetVelocity().y == Catch::Approx(0.0f).margin(1e-5f));
}

TEST_CASE("Sampled flat wall and corner preserve bulk density", "[fluid][validation][boundary][density]") {
    constexpr int size = 19;
    constexpr float spacing = 0.1f;
    constexpr float smoothingLength = 0.2f;
    FluidBoundarySettings boundarySettings;
    boundarySettings.particleRadius = 0.05f;
    FluidConvexPolygonContainer tank(
        {
            Vector2(-1.0f, 0.0f),
            Vector2(1.0f, 0.0f),
            Vector2(1.0f, 2.0f),
            Vector2(-1.0f, 2.0f),
        },
        boundarySettings
    );
    FluidBoundarySamplingSettings sampling;
    sampling.spacing = spacing;
    sampling.supportRadius = smoothingLength;
    const auto boundaryParticles = SampleFluidContainerBoundary(tank, sampling);
    auto particles = MakeLattice(
        size,
        size,
        spacing,
        smoothingLength,
        Vector2(-0.9f, 0.1f)
    );
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    WcsphSolver solver(smoothingLength, config);
    solver.prepare(particles, boundaryParticles);

    const float bulk = particles[9 * size + 9].density;
    const float flatWall = particles[9].density;
    const float corner = particles[0].density;
    INFO("bulk density: " << bulk);
    INFO("flat-wall density: " << flatWall);
    INFO("corner density: " << corner);
    REQUIRE(flatWall == Catch::Approx(bulk).epsilon(0.05f));
    REQUIRE(corner == Catch::Approx(bulk).epsilon(0.05f));
}

TEST_CASE("Polygon corner projection prevents escape and outward velocity", "[fluid][validation][boundary][corner]") {
    FluidBoundarySettings settings;
    settings.particleRadius = 0.05f;
    settings.restitution = 0.0f;
    settings.friction = 0.0f;
    FluidConvexPolygonContainer tank(
        {
            Vector2(-1.0f, 0.0f),
            Vector2(1.0f, 0.0f),
            Vector2(1.0f, 2.0f),
            Vector2(-1.0f, 2.0f),
        },
        settings
    );
    FluidParticle particle(
        Vector2(-1.25f, -0.30f),
        Vector2(-7.0f, -9.0f)
    );

    const FluidBoundaryCorrection correction = tank.enforce(particle);

    INFO("projected position: " << particle.position.x << ", " << particle.position.y);
    INFO("projected velocity: " << particle.velocity.x << ", " << particle.velocity.y);
    REQUIRE(correction.corrected);
    REQUIRE(tank.contains(particle.position));
    REQUIRE(particle.position.x == Catch::Approx(-0.95f).margin(1e-5f));
    REQUIRE(particle.position.y == Catch::Approx(0.05f).margin(1e-5f));
    REQUIRE(particle.velocity.x >= 0.0f);
    REQUIRE(particle.velocity.y >= 0.0f);
}

TEST_CASE("Wall restitution has the expected normal energy ratio", "[fluid][validation][boundary][impact]") {
    FluidBoundarySettings settings;
    settings.particleRadius = 0.05f;
    settings.restitution = 0.25f;
    settings.friction = 0.0f;
    FluidConvexPolygonContainer tank(
        {
            Vector2(-1.0f, 0.0f),
            Vector2(1.0f, 0.0f),
            Vector2(1.0f, 2.0f),
            Vector2(-1.0f, 2.0f),
        },
        settings
    );
    FluidParticle particle(Vector2(0.0f, 0.0f), Vector2(2.0f, -4.0f));
    const float normalEnergyBefore = 0.5f * particle.mass * 16.0f;

    tank.enforce(particle);

    const float normalEnergyAfter = 0.5f * particle.mass
        * particle.velocity.y * particle.velocity.y;
    REQUIRE(particle.velocity.x == Catch::Approx(2.0f));
    REQUIRE(particle.velocity.y == Catch::Approx(1.0f));
    REQUIRE(normalEnergyAfter / normalEnergyBefore
        == Catch::Approx(settings.restitution * settings.restitution));
}

TEST_CASE("Sampled circular wall reconstructs density without radial bias", "[fluid][validation][boundary][density][curved]") {
    constexpr float spacing = 0.1f;
    constexpr float smoothingLength = 0.2f;
    FluidBoundarySettings settings;
    settings.particleRadius = 0.05f;
    settings.friction = 0.0f;
    FluidCircleContainer tank(Vector2(), 2.0f, settings);
    FluidBoundarySamplingSettings sampling;
    sampling.spacing = spacing;
    sampling.supportRadius = smoothingLength;
    const auto boundaryParticles = SampleFluidContainerBoundary(tank, sampling);
    FluidParticleProperties properties;
    properties.mass = properties.restDensity * spacing * spacing;
    properties.smoothingLength = smoothingLength;
    std::vector<FluidParticle> particles;
    for (int y = -19; y <= 19; ++y) {
        for (int x = -19; x <= 19; ++x) {
            const Vector2 position(x * spacing, y * spacing);
            if (tank.contains(position)) {
                particles.emplace_back(position, Vector2(), properties);
            }
        }
    }
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    WcsphSolver solver(smoothingLength, config);
    solver.prepare(particles, boundaryParticles);

    double centerTotal = 0.0;
    double wallTotal = 0.0;
    int centerCount = 0;
    int wallCount = 0;
    for (const FluidParticle& particle : particles) {
        const float radius = particle.position.magnitude();
        if (radius < 1.0f) {
            centerTotal += particle.density / particle.restDensity;
            ++centerCount;
        } else if (radius > 1.75f) {
            wallTotal += particle.density / particle.restDensity;
            ++wallCount;
        }
    }
    const float centerMean = static_cast<float>(centerTotal / centerCount);
    const float wallMean = static_cast<float>(wallTotal / wallCount);
    INFO("center density ratio: " << centerMean);
    INFO("curved-wall density ratio: " << wallMean);
    REQUIRE(wallMean == Catch::Approx(centerMean).epsilon(0.05f));
}

TEST_CASE("Free-slip wall does not arrest tangential particle motion", "[fluid][validation][boundary][adhesion]") {
    FluidBoundarySettings settings;
    settings.particleRadius = 0.05f;
    settings.restitution = 0.0f;
    settings.friction = 0.0f;
    FluidConvexPolygonContainer tank(
        {
            Vector2(-1.0f, 0.0f),
            Vector2(1.0f, 0.0f),
            Vector2(1.0f, 2.0f),
            Vector2(-1.0f, 2.0f),
        },
        settings
    );
    FluidParticle particle(
        Vector2(0.0f, 0.04f),
        Vector2(1.0f, -0.5f)
    );

    const FluidBoundaryCorrection correction = tank.enforce(particle);

    REQUIRE(correction.corrected);
    REQUIRE(particle.position.y == Catch::Approx(0.05f));
    REQUIRE(particle.velocity.x == Catch::Approx(1.0f));
    REQUIRE(particle.velocity.y == Catch::Approx(0.0f));
}

TEST_CASE("Default fluid wall response does not create repeated tangential adhesion", "[fluid][validation][boundary][adhesion]") {
    FluidBoundarySettings settings;
    settings.particleRadius = 0.05f;
    settings.restitution = 0.0f;
    FluidConvexPolygonContainer tank(
        {
            Vector2(-1.0f, 0.0f),
            Vector2(1.0f, 0.0f),
            Vector2(1.0f, 2.0f),
            Vector2(-1.0f, 2.0f),
        },
        settings
    );
    FluidParticle particle(
        Vector2(0.0f, 0.04f),
        Vector2(1.0f, -0.1f)
    );
    for (int correction = 0; correction < 120; ++correction) {
        particle.position.y = 0.04f;
        particle.velocity.y = -0.1f;
        tank.enforce(particle);
    }

    INFO("remaining tangential speed: " << particle.velocity.x);
    REQUIRE(particle.velocity.x > 0.9f);
}

TEST_CASE("Wall friction is invariant to boundary correction frequency", "[fluid][validation][boundary][adhesion][rate]") {
    FluidBoundarySettings settings;
    settings.particleRadius = 0.05f;
    settings.restitution = 0.0f;
    settings.friction = 0.05f;
    FluidConvexPolygonContainer tank(
        {
            Vector2(-1.0f, 0.0f),
            Vector2(1.0f, 0.0f),
            Vector2(1.0f, 2.0f),
            Vector2(-1.0f, 2.0f),
        },
        settings
    );
    FluidParticle oneCorrection(Vector2(0.0f, 0.04f), Vector2(1.0f, -0.1f));
    FluidParticle fourCorrections = oneCorrection;
    tank.enforce(oneCorrection);
    for (int index = 0; index < 4; ++index) {
        fourCorrections.position.y = 0.04f;
        fourCorrections.velocity.y = -0.025f;
        tank.enforce(fourCorrections);
    }
    INFO("one-correction tangential speed: " << oneCorrection.velocity.x);
    INFO("four-correction tangential speed: " << fourCorrections.velocity.x);
    REQUIRE(fourCorrections.velocity.x
        == Catch::Approx(oneCorrection.velocity.x).margin(1e-5f));
}

TEST_CASE("Resting column is initialized with hydrostatic pressure support", "[fluid][validation][hydrostatic][initialization][!mayfail]") {
    constexpr int columns = 19;
    constexpr int rows = 15;
    constexpr float spacing = 0.1f;
    constexpr float smoothingLength = 0.2f;
    constexpr float massScale = 1.0f / 1.014612675f;
    constexpr float surfaceHeight = 1.55f;
    FluidBoundarySettings boundarySettings;
    boundarySettings.particleRadius = 0.05f;
    boundarySettings.friction = 0.0f;
    FluidConvexPolygonContainer tank(
        {
            Vector2(-1.0f, 0.0f),
            Vector2(1.0f, 0.0f),
            Vector2(1.0f, 2.0f),
            Vector2(-1.0f, 2.0f),
        },
        boundarySettings
    );
    FluidBoundarySamplingSettings sampling;
    sampling.spacing = spacing;
    sampling.supportRadius = smoothingLength;
    auto boundaryParticles = SampleFluidContainerBoundary(tank, sampling);
    for (FluidBoundaryParticle& boundaryParticle : boundaryParticles) {
        boundaryParticle.volume *= massScale;
    }
    auto particles = MakeLattice(
        columns,
        rows,
        spacing,
        smoothingLength,
        Vector2(-0.9f, 0.1f),
        Vector2(),
        massScale
    );
    WcsphConfig config;
    config.speedOfSound = 40.0f;
    WcsphSolver solver(smoothingLength, config);
    solver.prepare(particles, boundaryParticles);

    double squaredPressureError = 0.0;
    double squaredAccelerationResidual = 0.0;
    int samples = 0;
    for (int row = 2; row <= 11; ++row) {
        const FluidParticle& particle = particles[row * columns + columns / 2];
        const float expectedPressure = particle.restDensity * 9.81f
            * (surfaceHeight - particle.position.y);
        const float pressureError = (particle.pressure - expectedPressure)
            / (particle.restDensity * 9.81f * surfaceHeight);
        squaredPressureError += pressureError * pressureError;
        const Vector2 acceleration = particle.force * particle.inverseMass;
        squaredAccelerationResidual += acceleration.magnitudeSquared();
        ++samples;
    }
    const float normalizedPressureRmse = static_cast<float>(std::sqrt(
        squaredPressureError / samples
    ));
    const float accelerationRms = static_cast<float>(std::sqrt(
        squaredAccelerationResidual / samples
    ));
    INFO("normalized hydrostatic pressure RMSE: " << normalizedPressureRmse);
    INFO("RMS acceleration at t=0: " << accelerationRms);
    REQUIRE(normalizedPressureRmse < 0.1f);
    REQUIRE(accelerationRms < 1.0f);
}

TEST_CASE("Sampled-wall hydrostatic column remains quiet and weakly compressible", "[fluid][validation][boundary][hydrostatic][!mayfail]") {
    constexpr int columns = 19;
    constexpr int rows = 15;
    constexpr float spacing = 0.1f;
    constexpr float smoothingLength = 0.2f;
    FluidBoundarySettings boundarySettings;
    boundarySettings.particleRadius = 0.05f;
    boundarySettings.friction = 0.0f;
    FluidConvexPolygonContainer tank(
        {
            Vector2(-1.0f, 0.0f),
            Vector2(1.0f, 0.0f),
            Vector2(1.0f, 2.0f),
            Vector2(-1.0f, 2.0f),
        },
        boundarySettings
    );
    FluidBoundarySamplingSettings sampling;
    sampling.spacing = spacing;
    sampling.supportRadius = smoothingLength;
    const auto boundaryParticles = SampleFluidContainerBoundary(tank, sampling);
    auto particles = MakeLattice(
        columns,
        rows,
        spacing,
        smoothingLength,
        Vector2(-0.9f, 0.1f)
    );
    WcsphConfig config;
    config.speedOfSound = 40.0f;
    config.maximumTimeStep = 0.001f;
    WcsphSolver solver(smoothingLength, config);
    const WcsphSolver::SubstepCallback noOp = [](float) {};
    for (int step = 0; step < 500; ++step) {
        solver.step(
            particles,
            0.001f,
            tank,
            boundaryParticles,
            noOp
        );
    }

    float maximumBulkDensityError = 0.0f;
    float maximumSpeed = 0.0f;
    bool allContained = true;
    for (const FluidParticle& particle : particles) {
        if (particle.position.y < 1.2f
            && std::abs(particle.position.x) < 0.75f) {
            maximumBulkDensityError = std::max(
                maximumBulkDensityError,
                std::abs(particle.density / particle.restDensity - 1.0f)
            );
        }
        maximumSpeed = std::max(maximumSpeed, particle.velocity.magnitude());
        allContained = allContained && tank.contains(particle.position);
    }
    INFO("maximum bulk density error: " << maximumBulkDensityError);
    INFO("maximum particle speed: " << maximumSpeed);
    REQUIRE(allContained);
    REQUIRE(maximumBulkDensityError < 0.05f);
    REQUIRE(maximumSpeed < 0.2f);
}

TEST_CASE("Calibrated sampled-wall hydrostatic column remains bounded", "[fluid][validation][boundary][hydrostatic][calibration][!mayfail]") {
    constexpr int columns = 19;
    constexpr int rows = 15;
    constexpr float spacing = 0.1f;
    constexpr float smoothingLength = 0.2f;
    constexpr float massScale = 1.0f / 1.014612675f;
    FluidBoundarySettings boundarySettings;
    boundarySettings.particleRadius = 0.05f;
    boundarySettings.friction = 0.0f;
    FluidConvexPolygonContainer tank(
        {
            Vector2(-1.0f, 0.0f),
            Vector2(1.0f, 0.0f),
            Vector2(1.0f, 2.0f),
            Vector2(-1.0f, 2.0f),
        },
        boundarySettings
    );
    FluidBoundarySamplingSettings sampling;
    sampling.spacing = spacing;
    sampling.supportRadius = smoothingLength;
    auto boundaryParticles = SampleFluidContainerBoundary(tank, sampling);
    for (FluidBoundaryParticle& boundaryParticle : boundaryParticles) {
        boundaryParticle.volume *= massScale;
    }
    auto particles = MakeLattice(
        columns,
        rows,
        spacing,
        smoothingLength,
        Vector2(-0.9f, 0.1f),
        Vector2(),
        massScale
    );
    WcsphConfig config;
    config.speedOfSound = 40.0f;
    config.maximumTimeStep = 0.001f;
    WcsphSolver solver(smoothingLength, config);
    const WcsphSolver::SubstepCallback noOp = [](float) {};
    for (int step = 0; step < 500; ++step) {
        solver.step(particles, 0.001f, tank, boundaryParticles, noOp);
    }

    float maximumDensityError = 0.0f;
    float maximumSpeed = 0.0f;
    bool allContained = true;
    for (const FluidParticle& particle : particles) {
        maximumDensityError = std::max(
            maximumDensityError,
            std::abs(particle.density / particle.restDensity - 1.0f)
        );
        maximumSpeed = std::max(maximumSpeed, particle.velocity.magnitude());
        allContained = allContained && tank.contains(particle.position);
    }
    INFO("maximum density error: " << maximumDensityError);
    INFO("maximum particle speed: " << maximumSpeed);
    REQUIRE(allContained);
    REQUIRE(maximumDensityError < 0.05f);
    REQUIRE(maximumSpeed < 0.5f);
}

TEST_CASE("Hydrostatic instability is measured under timestep refinement", "[fluid][validation][hydrostatic][convergence]") {
    const HydrostaticTrace coarse = TraceHydrostaticColumn(0.001f, 0.2f);
    const HydrostaticTrace fine = TraceHydrostaticColumn(0.00025f, 0.2f);
    INFO("coarse max speed: " << coarse.maximumSpeed);
    INFO("fine max speed: " << fine.maximumSpeed);
    INFO("coarse max density error: " << coarse.maximumDensityError);
    INFO("fine max density error: " << fine.maximumDensityError);
    INFO("coarse max acceleration: " << coarse.maximumAcceleration);
    INFO("fine max acceleration: " << fine.maximumAcceleration);
    INFO("coarse force-criterion violation: "
        << coarse.maximumForceCriterionViolation);
    INFO("fine force-criterion violation: "
        << fine.maximumForceCriterionViolation);
    REQUIRE(fine.maximumSpeed < coarse.maximumSpeed);
    REQUIRE(fine.maximumDensityError < coarse.maximumDensityError);
}
