#include "catch_amalgamated.hpp"

#include "physics/core/fluids/fluid_boundary.h"
#include "physics/core/fluids/wcsph_solver.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"

#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

using namespace PhysicsEngine;

TEST_CASE("Fluid containers reject consistently turning self intersecting outlines", "[fluid-geometry]") {
    std::vector<Vector2> star;
    for (int index : {0, 2, 4, 1, 3}) {
        const float angle = index * 6.28318530718f / 5.0f;
        star.emplace_back(std::cos(angle), std::sin(angle));
    }
    for (int winding = 0; winding < 2; ++winding) {
        REQUIRE_THROWS_AS(FluidConvexPolygonContainer(star), std::invalid_argument);
        std::reverse(star.begin(), star.end());
    }
}

TEST_CASE("Fluid polygon geometry retains nonzero normals for large finite edges", "[fluid-geometry]") {
    const float halfSize = 1e20f;
    std::vector<Vector2> vertices{{-halfSize, -halfSize}, {halfSize, -halfSize},
                                 {halfSize, halfSize}, {-halfSize, halfSize}};
    FluidBoundarySettings settings;
    settings.particleRadius = 1e18f;
    for (int winding = 0; winding < 2; ++winding) {
        FluidConvexPolygonContainer boundary(vertices, settings);
        REQUIRE(boundary.contains(Vector2()));
        REQUIRE_FALSE(boundary.contains(Vector2(1.2e20f, 0.0f)));
        FluidParticle particle(Vector2(1.2e20f, 0.0f), Vector2(3.0f, 2.0f));
        const auto correction = boundary.enforce(particle);
        REQUIRE(correction.corrected);
        REQUIRE(std::isfinite(correction.penetration));
        REQUIRE(particle.position.x == Catch::Approx(halfSize - settings.particleRadius));
        REQUIRE(particle.velocity == Vector2(0.0f, 2.0f));
        REQUIRE(boundary.contains(particle.position));
        const FluidBoundarySamplingSettings sampling{1e18f, 2e18f};
        const auto samples = SampleFluidContainerBoundary(boundary, sampling);
        REQUIRE(std::any_of(samples.begin(), samples.end(), [halfSize](const auto& sample) {
            return static_cast<double>(-halfSize) - sample.position.y > 0.5e18;
        }));
        std::reverse(vertices.begin(), vertices.end());
    }
}

TEST_CASE("Fluid polygon outlines validate consistently with rigid shapes at small scale", "[fluid-geometry]") {
    std::vector<Vector2> vertices{{-1e-4f, -1e-4f}, {1e-4f, -1e-4f},
                                 {1e-4f, 1e-4f}, {-1e-4f, 1e-4f}};
    FluidBoundarySettings settings;
    settings.particleRadius = 1e-5f;
    for (int winding = 0; winding < 2; ++winding) {
        REQUIRE_NOTHROW(Polygon(vertices));
        FluidConvexPolygonContainer boundary(vertices, settings);
        REQUIRE(boundary.contains(Vector2()));
        FluidParticle particle(Vector2(2e-4f, 0.0f), Vector2(1.0f, 2.0f));
        REQUIRE(boundary.enforce(particle).corrected);
        REQUIRE(boundary.contains(particle.position));
        REQUIRE(particle.position.x == Catch::Approx(9e-5f).margin(1e-9f));
        REQUIRE(!SampleFluidContainerBoundary(boundary, {2e-5f, 4e-5f}).empty());
        std::reverse(vertices.begin(), vertices.end());
    }
}

TEST_CASE("Fluid polygon validation rejects repeated collinear and concave outlines", "[fluid-geometry]") {
    const std::vector<std::vector<Vector2>> invalid = {
        {{0, 0}, {1, 0}, {1, 0}, {0, 1}},
        {{0, 0}, {1, 0}, {2, 0}, {2, 1}, {0, 1}},
        {{0, 0}, {2, 0}, {1, 0.5f}, {2, 2}, {0, 2}},
        {{0, 0}, {1, 1}, {0, 1}, {1, 0}}
    };
    for (auto vertices : invalid) {
        for (int winding = 0; winding < 2; ++winding) {
            REQUIRE_THROWS_AS(Polygon(vertices), std::invalid_argument);
            REQUIRE_THROWS_AS(FluidConvexPolygonContainer(vertices), std::invalid_argument);
            std::reverse(vertices.begin(), vertices.end());
        }
    }
}

TEST_CASE("Translated fluid outlines use local differences for winding and distance", "[fluid-geometry]") {
    const float center = 1e20f;
    const float low = center - 1e15f;
    const float high = center + 1e15f;
    std::vector<Vector2> vertices{{low, low}, {high, low}, {high, high}, {low, high}};
    FluidBoundarySettings settings;
    settings.particleRadius = 1e14f;
    for (int winding = 0; winding < 2; ++winding) {
        FluidConvexPolygonContainer boundary(vertices, settings);
        REQUIRE(boundary.contains(Vector2(center, center)));
        FluidParticle particle(Vector2(std::nextafter(high, INFINITY), center), Vector2(1, 2));
        REQUIRE_FALSE(boundary.contains(particle.position));
        REQUIRE(boundary.enforce(particle).corrected);
        REQUIRE(boundary.contains(particle.position));
        REQUIRE(particle.velocity == Vector2(0, 2));
        const auto samples = SampleFluidContainerBoundary(boundary, {2e14f, 4e14f});
        REQUIRE(std::any_of(samples.begin(), samples.end(), [low](const auto& sample) {
            return static_cast<double>(low) - sample.position.y > 1e14;
        }));
        std::reverse(vertices.begin(), vertices.end());
    }
}

TEST_CASE("Large diagonal fluid normals project corners and damp outward velocity", "[fluid-geometry]") {
    std::vector<Vector2> vertices{{0, -1e20f}, {1e20f, 0}, {0, 1e20f}, {-1e20f, 0}};
    FluidBoundarySettings settings;
    settings.particleRadius = 1e18f;
    for (int winding = 0; winding < 2; ++winding) {
        FluidConvexPolygonContainer boundary(vertices, settings);
        REQUIRE(boundary.contains(Vector2()));
        FluidParticle particle(Vector2(0, 1.5e20f), Vector2(0, 10));
        REQUIRE(boundary.enforce(particle).corrected);
        REQUIRE(boundary.contains(particle.position));
        REQUIRE(std::hypot(particle.velocity.x, particle.velocity.y) < 1e-5f);
        REQUIRE(std::isfinite(particle.position.x));
        REQUIRE(std::isfinite(particle.position.y));
        Polygon shape(vertices);
        RigidBody body(&shape, Material{}, Vector2(), true);
        const auto samples = SampleRigidBodyBoundaries({&body}, {1e18f, 2e18f});
        const double halfPlaneOffset = 1e20f / std::sqrt(2.0);
        const auto surfaceCount = std::count_if(samples.begin(), samples.end(),
            [halfPlaneOffset](const auto& sample) {
                const double distance = halfPlaneOffset
                    - (std::abs(static_cast<double>(sample.position.x))
                       + std::abs(static_cast<double>(sample.position.y))) / std::sqrt(2.0);
                return std::abs(distance) < 1e16;
            });
        const int edgeCount = static_cast<int>(std::ceil(std::hypot(1e20f, 1e20f) / 1e18f));
        REQUIRE(surfaceCount == 4 * edgeCount);
        std::reverse(vertices.begin(), vertices.end());
    }
}

TEST_CASE("Large finite circle geometry avoids overflow in containment and projection", "[fluid-geometry]") {
    FluidBoundarySettings settings;
    settings.particleRadius = 1e18f;
    FluidCircleContainer boundary(Vector2(1e20f, -1e20f), 1e20f, settings);
    REQUIRE(boundary.contains(Vector2(1e20f, -1e20f)));
    REQUIRE_FALSE(boundary.contains(Vector2(2.5e20f, -1e20f)));
    FluidParticle particle(Vector2(2.5e20f, -1e20f), Vector2(3, 2));
    REQUIRE(boundary.enforce(particle).corrected);
    REQUIRE(boundary.contains(particle.position));
    REQUIRE(particle.velocity == Vector2(0, 2));
}

TEST_CASE("Boundary geometry rejects unrepresentable derived results without partial publication", "[fluid-geometry]") {
    const float maximum = std::numeric_limits<float>::max();
    FluidCircleContainer circle(Vector2(maximum, 0), 1e24f);
    std::vector<FluidBoundaryParticle> samples = {{Vector2(7, 8), Vector2(), 1}};
    REQUIRE_THROWS_AS(circle.appendBoundaryParticles({1e19f, 1e19f}, samples), std::overflow_error);
    REQUIRE(samples.size() == 1);
    REQUIRE(samples[0].position == Vector2(7, 8));

    FluidConvexPolygonContainer polygon({{1e38f, 1e38f}, {2e38f, 1e38f}, {2e38f, 2e38f}, {1e38f, 2e38f}});
    FluidParticle particle(Vector2(-maximum, 1.5e38f), Vector2(-1, 2));
    const auto original = particle;
    REQUIRE_THROWS_AS(polygon.enforce(particle), std::overflow_error);
    REQUIRE(particle.position == original.position);
    REQUIRE(particle.velocity == original.velocity);
    REQUIRE_THROWS_AS(circle.enforce(particle), std::overflow_error);
    REQUIRE(particle.position == original.position);
    REQUIRE(particle.velocity == original.velocity);

    Polygon shape = Polygon::MakeBox(4, 4);
    RigidBody body(&shape, Material{}, Vector2(), true);
    body.SetAngularVelocity(maximum);
    REQUIRE_THROWS_AS(SampleRigidBodyBoundaries({&body}, {0.1f, 0.1f}), std::overflow_error);
}

TEST_CASE("Boundary containment rejects nonfinite queries explicitly", "[fluid-geometry]") {
    FluidCircleContainer circle(Vector2(), 1);
    FluidConvexPolygonContainer polygon({{-1, -1}, {1, -1}, {1, 1}, {-1, 1}});
    for (float coordinate : {std::numeric_limits<float>::quiet_NaN(), INFINITY}) {
        REQUIRE_THROWS_AS(circle.contains(Vector2(coordinate, 0)), std::invalid_argument);
        REQUIRE_THROWS_AS(polygon.contains(Vector2(0, coordinate)), std::invalid_argument);
    }
}

TEST_CASE("Sampled container boundaries restore SPH density support", "[fluid][boundary][samples]") {
    FluidBoundarySettings boundarySettings;
    boundarySettings.particleRadius = 0.05f;
    FluidConvexPolygonContainer container(
        {
            Vector2(-2.0f, -2.0f),
            Vector2(2.0f, -2.0f),
            Vector2(2.0f, 2.0f),
            Vector2(-2.0f, 2.0f),
        },
        boundarySettings
    );
    FluidBoundarySamplingSettings sampling;
    sampling.spacing = 0.1f;
    sampling.supportRadius = 0.4f;
    const auto samples = SampleFluidContainerBoundary(container, sampling);
    REQUIRE_FALSE(samples.empty());
    for (const FluidBoundaryParticle& sample : samples) {
        REQUIRE(sample.pressureScale == Catch::Approx(1.0f));
        REQUIRE(sample.acceleration == Vector2());
    }

    FluidParticleProperties properties;
    properties.mass = 1000.0f * 0.1f * 0.1f;
    properties.restDensity = 1000.0f;
    properties.smoothingLength = 0.4f;
    std::vector<FluidParticle> unsupported = {
        FluidParticle(Vector2(0.0f, -1.9f), Vector2(), properties)
    };
    std::vector<FluidParticle> supported = unsupported;
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    WcsphSolver solver(properties.smoothingLength, config);

    solver.prepare(unsupported);
    solver.prepare(supported, samples);

    REQUIRE(supported.front().density > unsupported.front().density);
    REQUIRE(std::abs(supported.front().density - properties.restDensity)
        < std::abs(unsupported.front().density - properties.restDensity));
}

TEST_CASE("Sampled rigid boundaries follow surface motion", "[fluid][boundary][samples][rigid]") {
    Circle shape(0.5f);
    Material material{1.0f, 0.0f, 0.0f, 0.0f};
    RigidBody body(&shape, material, Vector2(1.0f, 2.0f));
    body.SetVelocity(Vector2(3.0f, -1.0f));
    body.SetAngularVelocity(2.0f);
    FluidBoundarySamplingSettings sampling;
    sampling.spacing = 0.1f;
    sampling.supportRadius = 0.3f;

    const auto samples = SampleRigidBodyBoundaries({&body}, sampling);

    REQUIRE_FALSE(samples.empty());
    for (const FluidBoundaryParticle& sample : samples) {
        REQUIRE((sample.position - body.GetPosition()).magnitude()
            <= shape.GetRadius() + 1e-5f);
        const Vector2 expectedVelocity = body.GetVelocityAtPoint(sample.position);
        REQUIRE(sample.velocity.x == Catch::Approx(expectedVelocity.x).margin(1e-5f));
        REQUIRE(sample.velocity.y == Catch::Approx(expectedVelocity.y).margin(1e-5f));
        REQUIRE(sample.volume == Catch::Approx(0.01f));
        REQUIRE(sample.pressureScale == Catch::Approx(0.0f));
    }
}

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
