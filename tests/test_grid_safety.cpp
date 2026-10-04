#include "catch_amalgamated.hpp"
#include "physics/core/collisions/broad_phase/uniform_grid.h"
#include "physics/core/fluids/fluid_boundary.h"
#include "physics/core/fluids/fluid_particle_spatial_grid.h"
#include "physics/core/fluids/sph_kernels.h"
#include "physics/core/fluids/wcsph_solver.h"
#include "physics/core/particles/particle_spatial_grid.h"
#include "../src/physics/core/checked_grid.h"

#include <cmath>
#include <limits>

using namespace PhysicsEngine;

TEST_CASE("Grid dimensions reject all invalid finite and nonfinite sizes", "[grid-safety]") {
    for (float size : {0.0f, -1.0f, std::numeric_limits<float>::infinity(),
                       std::numeric_limits<float>::quiet_NaN()}) {
        REQUIRE_THROWS_AS(ParticleSpatialGrid(size), std::invalid_argument);
        REQUIRE_THROWS_AS(FluidParticleSpatialGrid(size), std::invalid_argument);
        REQUIRE_THROWS_AS(UniformGrid(size), std::invalid_argument);
        UniformGrid grid(2.0f);
        REQUIRE_THROWS_AS(grid.setCellSize(size), std::invalid_argument);
        REQUIRE(grid.getCellSize() == 2.0f);
    }
}

TEST_CASE("Rejected particle grid rebuilds preserve the published index and statistics", "[grid-safety]") {
    const std::vector<Particle> original = {Particle(Vector2(0.0f, 0.0f)), Particle(Vector2(0.5f, 0.0f))};
    const std::vector<FluidParticle> fluidOriginal = {FluidParticle(Vector2(0.0f, 0.0f)), FluidParticle(Vector2(0.5f, 0.0f))};
    ParticleSpatialGrid grid(1.0f);
    FluidParticleSpatialGrid fluidGrid(1.0f);
    grid.rebuild(original);
    fluidGrid.rebuild(fluidOriginal);
    const auto expected = grid.findPotentialPairs(original, 1.0f);
    const auto fluidExpected = fluidGrid.findNeighborPairs(fluidOriginal, 1.0f);
    const auto statistics = fluidGrid.getLastStatistics();
    for (const Vector2& badPosition : {Vector2(1e30f, 0.0f), Vector2(0.0f, -1e30f),
         Vector2(std::numeric_limits<float>::quiet_NaN(), 0.0f),
         Vector2(0.0f, std::numeric_limits<float>::infinity())}) {
        auto changed = original;
        auto fluidChanged = fluidOriginal;
        // A valid first item goes into a different cell before the invalid last item.
        changed[0].position = fluidChanged[0].position = Vector2(50.0f, 0.0f);
        changed[1].position = fluidChanged[1].position = badPosition;
        REQUIRE_THROWS(grid.rebuild(changed));
        REQUIRE_THROWS(fluidGrid.rebuild(fluidChanged));
        REQUIRE(fluidGrid.getLastStatistics().occupiedCellCount == statistics.occupiedCellCount);
        REQUIRE(fluidGrid.getLastStatistics().neighborPairCount == statistics.neighborPairCount);
        REQUIRE(grid.findPotentialPairs(original, 1.0f) == expected);
        REQUIRE(fluidGrid.findNeighborPairs(fluidOriginal, 1.0f) == fluidExpected);
        REQUIRE_THROWS(grid.findPotentialPairs(changed, 1.0f));
        REQUIRE_THROWS(fluidGrid.findNeighborPairs(fluidChanged, 1.0f));
    }
}

TEST_CASE("Grid coordinates and scan endpoints handle exact integer boundaries", "[grid-safety]") {
    const int low = std::numeric_limits<int>::min();
    const int high = std::numeric_limits<int>::max();
    REQUIRE(CheckedGrid::Integer(static_cast<double>(low)) == low);
    REQUIRE(CheckedGrid::Integer(static_cast<double>(high)) == high);
    REQUIRE_THROWS_AS(CheckedGrid::Integer(static_cast<double>(low) - 1.0), std::overflow_error);
    REQUIRE_THROWS_AS(CheckedGrid::Integer(static_cast<double>(high) + 1.0), std::overflow_error);
    const auto lowerWindow = CheckedGrid::Around({low, low}, 1);
    const auto upperWindow = CheckedGrid::Around({high, high}, 1);
    REQUIRE(lowerWindow.minX == low);
    REQUIRE(lowerWindow.maxX == static_cast<std::int64_t>(low) + 1);
    REQUIRE(upperWindow.maxY == high);
    REQUIRE(upperWindow.minY == static_cast<std::int64_t>(high) - 1);
    std::uint64_t visits = CheckedGrid::MaximumGridVisits;
    REQUIRE_THROWS_AS(CheckedGrid::Charge(CheckedGrid::Window{low, high, low, high}, visits), std::length_error);

    const float upperOutside = 2147483648.0f;
    const float upperInside = std::nextafter(upperOutside, 0.0f);
    REQUIRE(CheckedGrid::Coordinate(static_cast<float>(low), 1.0f) == low);
    REQUIRE(CheckedGrid::Coordinate(upperInside, 1.0f) == 2147483520);
    REQUIRE_THROWS_AS(CheckedGrid::Coordinate(upperOutside, 1.0f), std::overflow_error);
    REQUIRE_THROWS_AS(CheckedGrid::Coordinate(std::nextafter(static_cast<float>(low), -INFINITY), 1.0f), std::overflow_error);
    for (float coordinate : {static_cast<float>(low), upperInside}) {
        std::vector<Particle> particles = {Particle(Vector2(coordinate, coordinate)), Particle(Vector2(coordinate, coordinate))};
        std::vector<FluidParticle> fluidParticles = {FluidParticle(Vector2(coordinate, coordinate)), FluidParticle(Vector2(coordinate, coordinate))};
        ParticleSpatialGrid grid(1.0f);
        FluidParticleSpatialGrid fluidGrid(1.0f);
        grid.rebuild(particles);
        fluidGrid.rebuild(fluidParticles);
        REQUIRE(grid.findPotentialPairs(particles, 1.0f).size() == 1);
        REQUIRE(fluidGrid.findNeighborPairs(fluidParticles, 1.0f).size() == 1);
    }
}

TEST_CASE("Grid queries reject overflowing radius and bounded but excessive work", "[grid-safety]") {
    const std::vector<Particle> particles = {Particle(), Particle(Vector2(0.5f, 0.0f))};
    const std::vector<FluidParticle> fluidParticles = {FluidParticle(), FluidParticle(Vector2(0.5f, 0.0f))};
    ParticleSpatialGrid grid(1.0f);
    FluidParticleSpatialGrid fluidGrid(1.0f);
    grid.rebuild(particles);
    fluidGrid.rebuild(fluidParticles);
    fluidGrid.findNeighborPairs(fluidParticles, 1.0f);
    REQUIRE_THROWS_AS(grid.findPotentialPairs(particles, 1e30f), std::overflow_error);
    REQUIRE_THROWS_AS(fluidGrid.findNeighborPairs(fluidParticles, 1e30f), std::overflow_error);
    REQUIRE_THROWS_AS(grid.findPotentialPairs(particles, 3000.0f), std::length_error);
    REQUIRE_THROWS_AS(fluidGrid.findNeighborPairs(fluidParticles, 3000.0f), std::length_error);
    REQUIRE(fluidGrid.getLastStatistics().neighborPairCount == 1);
    // Individually cheap windows must still respect the whole-query budget.
    std::vector<Particle> many(2000);
    std::vector<FluidParticle> fluidMany(2000);
    grid.rebuild(many);
    fluidGrid.rebuild(fluidMany);
    REQUIRE_THROWS_AS(grid.findPotentialPairs(many, 50.0f), std::length_error);
    REQUIRE_THROWS_AS(fluidGrid.findNeighborPairs(fluidMany, 50.0f), std::length_error);
    const float tiny = std::numeric_limits<float>::denorm_min();
    ParticleSpatialGrid tinyGrid(tiny);
    FluidParticleSpatialGrid tinyFluidGrid(tiny);
    REQUIRE_THROWS_AS(tinyGrid.rebuild(particles), std::overflow_error);
    REQUIRE_THROWS_AS(tinyFluidGrid.rebuild(fluidParticles), std::overflow_error);
    tinyGrid.rebuild({Particle()});
    tinyFluidGrid.rebuild({FluidParticle()});
    REQUIRE_THROWS_AS(tinyGrid.findPotentialPairs({Particle()}, 1.0f), std::overflow_error);
    REQUIRE_THROWS_AS(tinyFluidGrid.findNeighborPairs({FluidParticle()}, 1.0f), std::overflow_error);
}

TEST_CASE("Uniform grid rejects invalid coordinates and excessive AABB rasterization", "[grid-safety]") {
    Circle shape(1.0f);
    Material material{1.0f, 0.0f};
    auto first = std::make_shared<RigidBody>(&shape, material, Vector2());
    auto far = std::make_shared<RigidBody>(&shape, material, Vector2(1e30f, 0.0f));
    UniformGrid grid(1.0f);
    REQUIRE_THROWS_AS(grid.FindPotentialCollisions({first, far}), std::overflow_error);
    REQUIRE_THROWS_AS(grid.FindPotentialCollisions({far}), std::overflow_error);
    REQUIRE_THROWS_AS(grid.FindPotentialCollisions({nullptr}), std::invalid_argument);
    UniformGrid small(0.0001f);
    REQUIRE_THROWS_AS(small.FindPotentialCollisions({first}), std::length_error);
    UniformGrid tiny(std::numeric_limits<float>::denorm_min());
    REQUIRE_THROWS_AS(tiny.FindPotentialCollisions({first}), std::overflow_error);
    auto second = std::make_shared<RigidBody>(&shape, material, Vector2(1.0f, 0.0f));
    REQUIRE(grid.FindPotentialCollisions({first, second}).size() == 1);
}

TEST_CASE("Lattice calibration rejects excessive extents before iterating", "[grid-safety]") {
    REQUIRE_THROWS_AS(SphKernels2D::SquareLatticeMassScale(1e-30f, 1.0f), std::overflow_error);
    REQUIRE_THROWS_AS(SphKernels2D::SquareLatticeMassScale(0.001f, 1.0f), std::length_error);
    REQUIRE_THROWS_AS(SphKernels2D::SquareLatticeMassScale(1.0f, 2147483648.0f), std::overflow_error);
    REQUIRE(std::isfinite(SphKernels2D::SquareLatticeMassScale(0.25f, 0.5f)));
}

TEST_CASE("Particle neighbor cutoffs remain exact when float squared distances overflow", "[grid-safety]") {
    const std::vector<Particle> particles = {
        Particle(), Particle(Vector2(0.5e20f, 0.0f)), Particle(Vector2(2e20f, 0.0f))
    };
    const std::vector<FluidParticle> fluidParticles = {
        FluidParticle(), FluidParticle(Vector2(0.5e20f, 0.0f)), FluidParticle(Vector2(2e20f, 0.0f))
    };
    ParticleSpatialGrid grid(1e30f);
    FluidParticleSpatialGrid fluidGrid(1e30f);
    grid.rebuild(particles);
    fluidGrid.rebuild(fluidParticles);
    const std::vector<ParticleSpatialGrid::ParticlePair> expected = {{0, 1}};
    REQUIRE(grid.findPotentialPairs(particles, 1e20f) == expected);
    REQUIRE(fluidGrid.findNeighborPairs(fluidParticles, 1e20f) == expected);
}

TEST_CASE("Boundary sampling preflights layer and sample work and preserves append outputs", "[grid-safety]") {
    FluidCircleContainer circle(Vector2(), 1.0f);
    FluidConvexPolygonContainer polygon({{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}});
    for (const IFluidContainer* container : {static_cast<const IFluidContainer*>(&circle),
                                          static_cast<const IFluidContainer*>(&polygon)}) {
        std::vector<FluidBoundaryParticle> existing = {{Vector2(7.0f, 8.0f), Vector2(), 1.0f}};
        FluidBoundarySamplingSettings settings;
        settings.spacing = 1e-15f;
        REQUIRE_THROWS_AS(container->appendBoundaryParticles(settings, existing), std::overflow_error);
        settings.spacing = 0.001f;
        settings.supportRadius = 1.0f;
        REQUIRE_THROWS_AS(container->appendBoundaryParticles(settings, existing), std::length_error);
        settings.spacing = 1.0f;
        settings.supportRadius = 2000000.0f;
        REQUIRE_THROWS_AS(container->appendBoundaryParticles(settings, existing), std::length_error);
        REQUIRE(existing.size() == 1);
        REQUIRE(existing[0].position == Vector2(7.0f, 8.0f));
        settings.spacing = std::numeric_limits<float>::denorm_min();
        REQUIRE_THROWS_AS(container->appendBoundaryParticles(settings, existing), std::invalid_argument);
        settings.spacing = 1e30f;
        REQUIRE_THROWS_AS(container->appendBoundaryParticles(settings, existing), std::invalid_argument);
        settings = FluidBoundarySamplingSettings{};
        const auto ordinary = SampleFluidContainerBoundary(*container, settings);
        REQUIRE(!ordinary.empty());
        for (const auto& sample : ordinary) {
            REQUIRE(std::isfinite(sample.position.x));
            REQUIRE(std::isfinite(sample.position.y));
            REQUIRE(sample.volume == Catch::Approx(settings.spacing * settings.spacing));
        }
    }
    // A one-layer circumference/edge can overflow even with a small support radius.
    FluidBoundarySamplingSettings settings{1e-15f, 1e-15f};
    REQUIRE_THROWS_AS(SampleFluidContainerBoundary(circle, settings), std::overflow_error);
    REQUIRE_THROWS_AS(SampleFluidContainerBoundary(polygon, settings), std::overflow_error);
    FluidCircleContainer hugeCircle(Vector2(), 1e30f);
    REQUIRE_THROWS_AS(SampleFluidContainerBoundary(hugeCircle, FluidBoundarySamplingSettings{}), std::overflow_error);
}

TEST_CASE("Rigid boundary sampling charges containment work and aggregate shapes", "[grid-safety]") {
    Circle shape(1.0f);
    Material material{1.0f, 0.0f};
    RigidBody body(&shape, material, Vector2());
    FluidBoundarySamplingSettings settings{0.001f, 0.5f};
    REQUIRE_THROWS_AS(SampleRigidBodyBoundaries({&body}, settings), std::length_error);
    settings = {0.01f, 0.01f};
    REQUIRE(!SampleRigidBodyBoundaries({&body}, settings).empty());
    std::vector<RigidBody*> repeated(2000, &body);
    REQUIRE_THROWS_AS(SampleRigidBodyBoundaries(repeated, settings), std::length_error);
}

TEST_CASE("WCSPH boundary lookup rejects extreme coordinates and oversized radii", "[grid-safety]") {
    WcsphSolver solver(1.0f);
    std::vector<FluidParticle> particles = {FluidParticle()};
    std::vector<FluidBoundaryParticle> boundaries = {{Vector2(1e30f, 0.0f), Vector2(), 0.01f}};
    const float density = particles[0].density;
    REQUIRE_THROWS_AS(solver.prepare(particles, boundaries), std::overflow_error);
    REQUIRE(particles[0].density == density);
    std::vector<FluidParticle> empty;
    REQUIRE_THROWS_AS(solver.prepare(empty, boundaries), std::overflow_error);
    boundaries[0].position = Vector2(0.2f, 0.0f);
    particles[0].position = Vector2(-2147483648.0f, 0.0f);
    boundaries[0].position = particles[0].position;
    REQUIRE_NOTHROW(solver.prepare(particles, boundaries));
    particles[0].position = Vector2();
    particles[0].smoothingLength = 3000.0f;
    REQUIRE_THROWS_AS(solver.prepare(particles, boundaries), std::length_error);
    particles[0].smoothingLength = 0.5f;
    boundaries[0].position = Vector2(0.2f, 0.0f);
    solver.prepare(particles, boundaries);
    REQUIRE(solver.getLastStatistics().boundaryCandidateCount == 1);
}
