#include "catch_amalgamated.hpp"

#include "physics/core/fluids/fluid_boundary.h"
#include "physics/core/fluids/sph_kernels.h"
#include "physics/core/fluids/wcsph_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace PhysicsEngine;

namespace {

// Rectangular tank [-1, 1] x [0, 2] filled with a square lattice of rest
// density particles (spacing dx, h = 2 dx) whose rows start at y = dx. The free
// surface of a full column of `rows` rows sits half a spacing above its top row.
constexpr int kColumns = 19;
constexpr float kSpacing = 0.1f;
constexpr float kSmoothingLength = 0.2f;
constexpr float kRestDensity = 1000.0f;

struct Tank {
    std::vector<FluidParticle> particles;
    std::vector<FluidBoundaryParticle> walls;
    WcsphConfig config;
    int rows = 0;

    std::size_t index(int row, int column) const {
        return static_cast<std::size_t>(row * kColumns + column);
    }
    int centre() const { return kColumns / 2; }
    float surfaceHeight() const { return (rows + 0.5f) * kSpacing; }
    // Distance from the free surface to the particle in the given row.
    float depth(int row) const {
        return surfaceHeight() - particles[index(row, centre())].position.y;
    }
};

float LatticeMassScale() {
    return SphKernels2D::SquareLatticeMassScale(
        kSpacing, kSmoothingLength, SphKernelFamily::WendlandC2);
}

std::vector<FluidBoundaryParticle> SampleTank() {
    FluidBoundarySettings settings;
    settings.particleRadius = 0.05f;
    settings.friction = 0.0f;
    FluidConvexPolygonContainer tank(
        {Vector2(-1.0f, 0.0f), Vector2(1.0f, 0.0f),
         Vector2(1.0f, 2.0f), Vector2(-1.0f, 2.0f)},
        settings
    );
    FluidBoundarySamplingSettings sampling;
    sampling.spacing = kSpacing;
    sampling.supportRadius = kSmoothingLength;
    auto walls = SampleFluidContainerBoundary(tank, sampling);
    const float massScale = LatticeMassScale();
    for (FluidBoundaryParticle& wall : walls) {
        wall.volume *= massScale;
    }
    return walls;
}

// Appends a block of columns x rows rest-density particles, lower-left at origin.
void AppendBlock(
    std::vector<FluidParticle>& particles,
    int columns,
    int rows,
    const Vector2& origin
) {
    FluidParticleProperties properties;
    properties.mass = properties.restDensity * kSpacing * kSpacing * LatticeMassScale();
    properties.smoothingLength = kSmoothingLength;
    properties.viscosity = 0.05f;
    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < columns; ++column) {
            particles.emplace_back(
                origin + Vector2(column * kSpacing, row * kSpacing),
                Vector2(),
                properties
            );
        }
    }
}

Tank MakeFilledTank(int rows, float gravityScale = 1.0f) {
    Tank tank;
    tank.rows = rows;
    tank.walls = SampleTank();
    AppendBlock(tank.particles, kColumns, rows, Vector2(-0.9f, 0.1f));
    tank.config.speedOfSound = 40.0f;
    tank.config.externalAcceleration = Vector2(0.0f, -9.81f * gravityScale);
    return tank;
}

// Inverse of the Tait equation of state p = rho0 c^2 / gamma ((rho/rho0)^gamma - 1).
double TaitPressure(float density, const WcsphConfig& config) {
    const double gamma = config.equationOfStateExponent;
    const double c2 = static_cast<double>(config.speedOfSound) * config.speedOfSound;
    return kRestDensity * c2 / gamma * (std::pow(density / kRestDensity, gamma) - 1.0);
}

double TaitDensity(double pressure, const WcsphConfig& config) {
    const double gamma = config.equationOfStateExponent;
    const double c2 = static_cast<double>(config.speedOfSound) * config.speedOfSound;
    return kRestDensity * std::pow(1.0 + gamma * pressure / (kRestDensity * c2), 1.0 / gamma);
}

void Prepare(Tank& tank) {
    WcsphSolver solver(kSmoothingLength, tank.config);
    solver.prepare(tank.particles, tank.walls);
}

bool AnyWallWithin(const Tank& tank, const FluidParticle& particle, float distance) {
    for (const FluidBoundaryParticle& wall : tank.walls) {
        if ((particle.position - wall.position).magnitude() < distance) {
            return true;
        }
    }
    return false;
}

} // namespace

TEST_CASE("Hydrostatic seed gives a rest column depth-proportional pressure", "[fluid][hydrostatic-init]") {
    Tank tank = MakeFilledTank(15);
    Prepare(tank);

    const double weightPerDepth = kRestDensity * 9.81;
    const double bottomPressure = weightPerDepth * tank.depth(0);
    double previousPressure = std::numeric_limits<double>::max();
    // Tolerance: the seed targets the discrete pressure force rather than the
    // continuum profile, so allow 10% of the bottom pressure (the same budget
    // as the validation suite's normalized hydrostatic RMSE).
    for (int row = 0; row < tank.rows; ++row) {
        const FluidParticle& particle = tank.particles[tank.index(row, tank.centre())];
        const double expected = weightPerDepth * tank.depth(row);
        INFO("row " << row << " depth " << tank.depth(row) << " pressure "
            << particle.pressure << " expected " << expected);
        CHECK(std::abs(particle.pressure - expected) < 0.1 * bottomPressure);
        // Pressure must increase strictly with depth (decrease with height).
        CHECK(particle.pressure < previousPressure);
        previousPressure = particle.pressure;
        // The stored density is the Tait density of the stored pressure.
        CHECK(particle.density > kRestDensity);
        CHECK(particle.density == Catch::Approx(TaitDensity(particle.pressure, tank.config)).epsilon(1e-4));
        CHECK(particle.pressure == Catch::Approx(TaitPressure(particle.density, tank.config)).epsilon(1e-3));
    }
    // The point of the seed: the discrete pressure force already carries the
    // weight, so interior column particles start near force balance. Gravity
    // alone would give 9.81; require under 10% of it.
    for (int row = 2; row <= 11; ++row) {
        const FluidParticle& particle = tank.particles[tank.index(row, tank.centre())];
        const Vector2 acceleration = particle.force * particle.inverseMass;
        INFO("row " << row << " residual acceleration " << acceleration.magnitude());
        CHECK(acceleration.magnitude() < 1.0f);
    }
    const FluidParticle& top = tank.particles[tank.index(tank.rows - 1, tank.centre())];
    const FluidParticle& bottom = tank.particles[tank.index(0, tank.centre())];
    // The top particle sits half a spacing below the surface.
    CHECK(top.pressure < 0.1 * bottomPressure);
    CHECK(top.pressure >= 0.0f);
    CHECK(bottom.pressure > 0.9 * bottomPressure);
}

TEST_CASE("Hydrostatic seed pressure scales with gravity magnitude", "[fluid][hydrostatic-init]") {
    Tank reference = MakeFilledTank(15, 1.0f);
    Tank doubled = MakeFilledTank(15, 2.0f);
    Tank halved = MakeFilledTank(15, 0.5f);
    Prepare(reference);
    Prepare(doubled);
    Prepare(halved);

    const double bottomPressure = kRestDensity * 9.81 * reference.depth(0);
    for (int row = 0; row < reference.rows; ++row) {
        const std::size_t i = reference.index(row, reference.centre());
        const double base = reference.particles[i].pressure;
        INFO("row " << row);
        // The profile is linear in g up to the weak Tait nonlinearity (density
        // rises <1%), so a doubled / halved gravity must double / halve it. The
        // absolute error budget is 5% of the scaled bottom pressure.
        CHECK(std::abs(doubled.particles[i].pressure - 2.0 * base) < 0.05 * 2.0 * bottomPressure);
        CHECK(std::abs(halved.particles[i].pressure - 0.5 * base) < 0.05 * 0.5 * bottomPressure);
        // And each is itself near rho0 |g| depth.
        CHECK(std::abs(doubled.particles[i].pressure
            - 2.0 * kRestDensity * 9.81 * reference.depth(row)) < 0.1 * 2.0 * bottomPressure);
    }
}

TEST_CASE("Hydrostatic seed needs gravity", "[fluid][hydrostatic-init]") {
    Tank tank = MakeFilledTank(15, 0.0f);
    Prepare(tank);
    for (const FluidParticle& particle : tank.particles) {
        CHECK(particle.density == particle.restDensity);
        CHECK(particle.pressure == 0.0f);
    }
}

TEST_CASE("Hydrostatic seed preserves explicitly set densities", "[fluid][hydrostatic-init]") {
    Tank tank = MakeFilledTank(15);
    // Three particles in different places carry their own densities; the first
    // is below rest density, which must also survive (not be treated as unset).
    const std::size_t belowRest = tank.index(0, tank.centre());
    const std::size_t midColumn = tank.index(7, tank.centre());
    const std::size_t offCentre = tank.index(3, 4);
    tank.particles[belowRest].density = 0.99f * kRestDensity;
    tank.particles[midColumn].density = 1.003f * kRestDensity;
    tank.particles[offCentre].density = 1.0001f * kRestDensity;
    Prepare(tank);

    CHECK(tank.particles[belowRest].density == 0.99f * kRestDensity);
    CHECK(tank.particles[midColumn].density == 1.003f * kRestDensity);
    CHECK(tank.particles[offCentre].density == 1.0001f * kRestDensity);
    // The neighbors that were still at rest density are seeded.
    CHECK(tank.particles[tank.index(1, tank.centre())].density > kRestDensity);
    CHECK(tank.particles[tank.index(8, tank.centre())].density > kRestDensity);
}

TEST_CASE("Disabled hydrostatic initialization leaves rest density", "[fluid][hydrostatic-init]") {
    Tank tank = MakeFilledTank(15);
    tank.config.hydrostaticInitialization = WcsphHydrostaticInitialization::Disabled;
    Prepare(tank);
    for (const FluidParticle& particle : tank.particles) {
        CHECK(particle.density == particle.restDensity);
        // Tait pressure at exactly rest density is exactly zero.
        CHECK(particle.pressure == 0.0f);
    }
}

TEST_CASE("Hydrostatic seed ignores unsupported fluid", "[fluid][hydrostatic-init]") {
    SECTION("blob floating far above the floor") {
        Tank tank;
        tank.walls = SampleTank();
        tank.config.speedOfSound = 40.0f;
        AppendBlock(tank.particles, 5, 5, Vector2(-0.2f, 1.0f));
        for (const FluidParticle& particle : tank.particles) {
            // No wall sample is anywhere near: the blob hangs in mid-tank.
            REQUIRE_FALSE(AnyWallWithin(tank, particle, 3.0f * kSmoothingLength));
        }
        Prepare(tank);
        for (const FluidParticle& particle : tank.particles) {
            CHECK(particle.density == particle.restDensity);
            CHECK(particle.pressure == 0.0f);
        }
    }
    SECTION("blob touching only a side wall") {
        Tank tank;
        tank.walls = SampleTank();
        tank.config.speedOfSound = 40.0f;
        // The left column sits 0.1 from the wall at x = -1: inside the kernel
        // support, but the side-wall samples are not below the particles, so no
        // floor straddles them and the support rule leaves them unsupported.
        AppendBlock(tank.particles, 3, 4, Vector2(-0.9f, 1.0f));
        bool touchesSideWall = false;
        for (const FluidParticle& particle : tank.particles) {
            touchesSideWall |= AnyWallWithin(tank, particle, kSmoothingLength);
        }
        REQUIRE(touchesSideWall);
        Prepare(tank);
        for (const FluidParticle& particle : tank.particles) {
            CHECK(particle.density == particle.restDensity);
        }
    }
    SECTION("detached blob above a supported column") {
        Tank tank = MakeFilledTank(8);
        const std::size_t columnCount = tank.particles.size();
        // The gap between the column top (y = 0.8) and the blob (y = 1.4) is far
        // above the 1.5 spacing link distance.
        AppendBlock(tank.particles, 5, 3, Vector2(-0.2f, 1.4f));
        Prepare(tank);
        for (std::size_t i = 0; i < columnCount; ++i) {
            CHECK(tank.particles[i].density > tank.particles[i].restDensity
                - 1e-3f);  // column particles are never reduced
        }
        CHECK(tank.particles[tank.index(0, tank.centre())].density > kRestDensity);
        for (std::size_t i = columnCount; i < tank.particles.size(); ++i) {
            CHECK(tank.particles[i].density == tank.particles[i].restDensity);
        }
        // The column is seeded as if the blob did not exist: its surface is its own top.
        const FluidParticle& bottom = tank.particles[tank.index(0, tank.centre())];
        const double expected = kRestDensity * 9.81 * (8.5 * kSpacing - bottom.position.y);
        CHECK(std::abs(bottom.pressure - expected) < 0.1 * expected);
    }
}

TEST_CASE("Stepping never seeds the hydrostatic profile", "[fluid][hydrostatic-init]") {
    Tank tank = MakeFilledTank(15);
    const WcsphSolver::SubstepCallback noOp = [](float) {};

    SECTION("zero step without walls") {
        WcsphSolver solver(kSmoothingLength, tank.config);
        solver.step(tank.particles, 0.0f);
        for (const FluidParticle& particle : tank.particles) {
            CHECK(particle.density == particle.restDensity);
            CHECK(particle.pressure == 0.0f);
        }
    }
    SECTION("zero step with wall samples") {
        WcsphSolver solver(kSmoothingLength, tank.config);
        solver.step(tank.particles, 0.0f, tank.walls, noOp);
        for (const FluidParticle& particle : tank.particles) {
            CHECK(particle.density == particle.restDensity);
            CHECK(particle.pressure == 0.0f);
        }
    }
    SECTION("short positive step with wall samples") {
        // A seeded profile would put the bottom density ~0.9% above rest. One
        // short step moves density only by the continuity rate rho div(v) dt:
        // the freely accelerating fluid is divergence-free away from the walls,
        // so the change must stay far below that profile.
        Tank seeded = MakeFilledTank(15);
        Prepare(seeded);
        const std::size_t bottomIndex = tank.index(0, tank.centre());
        const double seededRise = seeded.particles[bottomIndex].density - kRestDensity;
        REQUIRE(seededRise > 5.0);

        WcsphSolver solver(kSmoothingLength, tank.config);
        solver.step(tank.particles, 1.0f / 1000.0f, tank.walls, noOp);
        double maximumRise = 0.0;
        for (const FluidParticle& particle : tank.particles) {
            maximumRise = std::max(maximumRise,
                static_cast<double>(std::abs(particle.density - kRestDensity)));
        }
        INFO("seeded bottom rise " << seededRise << ", largest unseeded change " << maximumRise);
        CHECK(maximumRise < 0.2 * seededRise);
    }
}

TEST_CASE("WcsphConfig rejects unrecognized hydrostatic and wall modes", "[fluid][hydrostatic-init]") {
    WcsphConfig config;
    CHECK_NOTHROW(config.Validate());

    WcsphConfig badInitialization;
    badInitialization.hydrostaticInitialization =
        static_cast<WcsphHydrostaticInitialization>(99);
    CHECK_THROWS_AS(badInitialization.Validate(), std::invalid_argument);
    CHECK_THROWS_AS(WcsphSolver(kSmoothingLength, badInitialization), std::invalid_argument);

    WcsphConfig badWall;
    badWall.wallConsistency = static_cast<WcsphWallConsistency>(99);
    CHECK_THROWS_AS(badWall.Validate(), std::invalid_argument);
    CHECK_THROWS_AS(WcsphSolver(kSmoothingLength, badWall), std::invalid_argument);

    // Both valid values of each enum are accepted.
    for (const auto mode : {WcsphHydrostaticInitialization::Disabled,
                            WcsphHydrostaticInitialization::RestDensityParticles}) {
        WcsphConfig valid;
        valid.hydrostaticInitialization = mode;
        CHECK_NOTHROW(valid.Validate());
    }
    for (const auto mode : {WcsphWallConsistency::RawSamples,
                            WcsphWallConsistency::ConstantPressureCompletion}) {
        WcsphConfig valid;
        valid.wallConsistency = mode;
        CHECK_NOTHROW(valid.Validate());
    }
}

namespace {

// Uniformly compressed lattice beside the sampled floor, with every other
// force source off, so the only force is the pressure force of a uniform
// pressure p. Such a pressure must exert no net force.
struct UniformPressureCase {
    Tank tank;
    double pressure = 0.0;

    UniformPressureCase(
        WcsphWallConsistency consistency,
        float compression,
        bool isolateLastParticle = false
    ) {
        tank = MakeFilledTank(6);
        if (isolateLastParticle) {
            auto& particles = tank.particles;
            particles.erase(
                std::remove_if(particles.begin(), particles.end(),
                    [](const FluidParticle& p) { return p.position.x > 0.25f; }),
                particles.end());
            AppendBlock(particles, 1, 1, Vector2(0.9f, 0.1f));
        }
        tank.config.externalAcceleration = Vector2();
        tank.config.acousticDamping = 0.0f;
        tank.config.particleShifting = 0.0f;
        tank.config.wallConsistency = consistency;
        tank.config.hydrostaticInitialization = WcsphHydrostaticInitialization::Disabled;
        for (FluidParticle& particle : tank.particles) {
            particle.viscosity = 0.0f;
            particle.density = kRestDensity * compression;
        }
        pressure = TaitPressure(kRestDensity * compression, tank.config);
    }

    void prepare() { Prepare(tank); }
    Vector2 force(int row, int column) const {
        return tank.particles[tank.index(row, column)].force;
    }
};

// Force a pressure p exerts across one particle width.
double ReferenceForce(const UniformPressureCase& scenario) {
    return scenario.pressure * kSpacing;
}

}

TEST_CASE("Wall completion cancels uniform-pressure force beside a staggered wall", "[fluid][hydrostatic-init]") {
    // The sampler places floor samples at x = -0.95 + 0.1 i, i.e. half a
    // spacing off the fluid lattice at x = -0.9 + 0.1 k; the 2% compression
    // additionally shrinks the fluid volumes against rest-volume wall samples.
    constexpr float compression = 1.02f;
    UniformPressureCase completed(WcsphWallConsistency::ConstantPressureCompletion, compression);
    UniformPressureCase raw(WcsphWallConsistency::RawSamples, compression);
    completed.prepare();
    raw.prepare();
    REQUIRE(completed.pressure > 0.0);
    const double reference = ReferenceForce(raw);

    // Floor-adjacent row, away from the side walls (columns 4..14).
    double largestRaw = 0.0;
    double largestCompleted = 0.0;
    for (int column = 4; column <= 14; ++column) {
        largestRaw = std::max(largestRaw, static_cast<double>(raw.force(0, column).magnitude()));
        largestCompleted = std::max(largestCompleted,
            static_cast<double>(completed.force(0, column).magnitude()));
        // The interior is a symmetric lattice: no wall, no force, either mode.
        CHECK(raw.force(2, column).magnitude() < 1e-4 * reference);
        CHECK(completed.force(2, column).magnitude() < 1e-4 * reference);
    }
    INFO("reference force " << reference << ", raw " << largestRaw
        << ", completed " << largestCompleted);
    // Raw samples leave a measurable residual (here around a percent of p dx);
    // completion removes it down to round-off of the float force accumulation.
    CHECK(largestRaw > 2e-3 * reference);
    CHECK(largestCompleted < 1e-4 * reference);
    CHECK(largestCompleted < 0.05 * largestRaw);

    // The same holds for particles beside a side wall (rows 2..4, column 0).
    for (int row = 2; row <= 4; ++row) {
        CHECK(completed.force(row, 0).magnitude() < 1e-3 * reference);
    }
}

TEST_CASE("Wall completion is not applied to badly incomplete stencils", "[fluid][hydrostatic-init]") {
    constexpr float compression = 1.02f;

    SECTION("free-surface corner keeps the raw response") {
        UniformPressureCase completed(WcsphWallConsistency::ConstantPressureCompletion, compression);
        UniformPressureCase raw(WcsphWallConsistency::RawSamples, compression);
        completed.prepare();
        raw.prepare();
        const double reference = ReferenceForce(raw);
        // The top-left particle has wall samples on one side and nothing above
        // it: its fluid stencil misses half the kernel, far over the 10%
        // mismatch cutoff, so completion must leave the raw force alone.
        const int top = completed.tank.rows - 1;
        const Vector2 rawForce = raw.force(top, 0);
        const Vector2 completedForce = completed.force(top, 0);
        REQUIRE(rawForce.magnitude() > 0.1 * reference);
        CHECK(completedForce.x == rawForce.x);
        CHECK(completedForce.y == rawForce.y);
        // The top row between the side walls has no wall at all.
        CHECK(completed.force(top, 9).x == raw.force(top, 9).x);
        CHECK(completed.force(top, 9).y == raw.force(top, 9).y);
    }
    SECTION("isolated particle near the floor keeps the raw response") {
        UniformPressureCase completed(WcsphWallConsistency::ConstantPressureCompletion, compression, true);
        UniformPressureCase raw(WcsphWallConsistency::RawSamples, compression, true);
        completed.prepare();
        raw.prepare();
        // The lone particle sits 0.1 above the floor and 0.65 or more from the
        // remaining lattice, outside its kernel support, so its fluid stencil is
        // empty and the mismatch is 100% of the wall term.
        const FluidParticle& loneCompleted = completed.tank.particles.back();
        const FluidParticle& loneRaw = raw.tank.particles.back();
        REQUIRE(loneCompleted.position.x == Catch::Approx(0.9f));
        const double reference = ReferenceForce(raw);
        // It must feel the floor, otherwise the comparison proves nothing.
        REQUIRE(loneRaw.force.magnitude() > 0.1 * reference);
        CHECK(loneCompleted.force.x == loneRaw.force.x);
        CHECK(loneCompleted.force.y == loneRaw.force.y);
    }
}
