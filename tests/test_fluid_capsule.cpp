#include "catch_amalgamated.hpp"

#include "physics/core/fluids/coupled_fluid_simulation.h"
#include "physics/core/fluids/fluid_boundary.h"
#include "physics/core/fluids/fluid_rigid_coupler.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace PhysicsEngine;

namespace {

constexpr double Pi = 3.14159265358979323846;
// Rigid bodies start here, in the tank of tests/test_coupled_fluid_simulation.cpp
// with fluid up to y = 1.8. The fluid lattice has rows every 0.15 m at
// multiples of 0.15, so 1.05 leaves equal gaps above and below both the
// 0.3 m circle and a horizontal 0.2 m capsule.
constexpr float StartHeight = 1.05f;

// Independent double-precision capsule geometry, no engine code.
struct SpineFrame {
    double cx, cy, ux, uy, h, r;
};

SpineFrame FrameOf(const RigidBody& body) {
    const auto& capsule = dynamic_cast<const Capsule&>(*body.shape);
    const double angle = body.GetOrientation();
    return {body.GetPosition().x, body.GetPosition().y, std::cos(angle), std::sin(angle),
            capsule.GetHalfLength(), capsule.GetRadius()};
}

// Distance from (x, y) to the spine and the unit direction away from it.
double SpineDistance(const SpineFrame& f, double x, double y, double* nx = nullptr, double* ny = nullptr) {
    const double rx = x - f.cx, ry = y - f.cy;
    const double t = std::clamp(rx * f.ux + ry * f.uy, -f.h, f.h);
    const double dx = rx - f.ux * t, dy = ry - f.uy * t;
    const double d = std::hypot(dx, dy);
    if (nx) *nx = dx / d;
    if (ny) *ny = dy / d;
    return d;
}

bool SameBits(float a, float b) { return std::memcmp(&a, &b, sizeof a) == 0; }
bool SameBits(const Vector2& a, const Vector2& b) { return SameBits(a.x, b.x) && SameBits(a.y, b.y); }

FluidParticleProperties Properties(float smoothingLength, float viscosity) {
    FluidParticleProperties properties;
    properties.mass = 1.0f;
    properties.smoothingLength = smoothingLength;
    properties.viscosity = viscosity;
    return properties;
}

// Every particle's momentum plus the body's, and angular momentum about the
// world origin, accumulated in double.
struct Momentum { double x, y, angular; };
Momentum TotalMomentum(const std::vector<FluidParticle>& particles, const RigidBody& body) {
    const double m = body.GetMass();
    const Vector2 p = body.GetPosition(), v = body.GetVelocity();
    Momentum total{m * v.x, m * v.y,
        double(body.inertia) * body.GetAngularVelocity() + m * (double(p.x) * v.y - double(p.y) * v.x)};
    for (const FluidParticle& particle : particles) {
        total.x += double(particle.mass) * particle.velocity.x;
        total.y += double(particle.mass) * particle.velocity.y;
        total.angular += double(particle.mass) * (double(particle.position.x) * particle.velocity.y
            - double(particle.position.y) * particle.velocity.x);
    }
    return total;
}

// The tank of tests/test_coupled_fluid_simulation.cpp: fluid on a 0.15 m
// lattice up to y = 1.8, the body released from rest at StartHeight.
struct TankResult { float y, vy, orientation; };
TankResult RunTank(const Shape& shape, float density, float orientation,
                   SphKernelFamily family = SphKernelFamily::Poly6Spiky, int steps = 200,
                   int holdSteps = 0) {
    constexpr float spacing = 0.15f;
    constexpr float smoothingLength = 0.3f;
    FluidBoundarySettings boundarySettings;
    boundarySettings.particleRadius = spacing * 0.5f;
    FluidConvexPolygonContainer tank(
        {Vector2(-1.5f, 0.0f), Vector2(1.5f, 0.0f), Vector2(1.5f, 3.0f), Vector2(-1.5f, 3.0f)},
        boundarySettings);
    Material material{density, 0.0f, 0.0f, 0.0f};
    RigidBody body(&shape, material, Vector2(0.0f, StartHeight));
    body.SetOrientation(orientation);
    FluidParticleProperties properties;
    properties.mass = properties.restDensity * spacing * spacing;
    properties.smoothingLength = smoothingLength;
    properties.viscosity = 0.08f;
    // Fill the same lattice and drop particles within particleRadius of the
    // body surface. A circle is a capsule with a point spine; a polygon here
    // is a centered box.
    const double cosine = std::cos(double(orientation)), sine = std::sin(double(orientation));
    const auto clearance = [&](double x, double y) {
        if (shape.type == ShapeType::POLYGON) {
            const auto& vertices = static_cast<const Polygon&>(shape).getVertices();
            const double hx = std::abs(vertices[0].x), hy = std::abs(vertices[0].y);
            const double lx = cosine * x + sine * (y - StartHeight);
            const double ly = -sine * x + cosine * (y - StartHeight);
            return std::hypot(std::max(std::abs(lx) - hx, 0.0), std::max(std::abs(ly) - hy, 0.0));
        }
        const double halfLength = shape.type == ShapeType::CAPSULE
            ? static_cast<const Capsule&>(shape).GetHalfLength() : 0.0;
        const SpineFrame frame{0.0, StartHeight, cosine, sine, halfLength, shape.GetRadius()};
        return SpineDistance(frame, x, y) - shape.GetRadius();
    };
    std::vector<FluidParticle> particles;
    for (float y = spacing; y <= 1.8f; y += spacing) {
        for (float x = -1.35f; x <= 1.35f; x += spacing) {
            if (clearance(x, y) > boundarySettings.particleRadius) {
                particles.emplace_back(Vector2(x, y), Vector2(), properties);
            }
        }
    }
    WcsphConfig fluidConfig;
    fluidConfig.kernelFamily = family;
    fluidConfig.speedOfSound = 15.0f;
    fluidConfig.maximumTimeStep = 0.003f;
    FluidRigidCouplingSettings couplingSettings;
    couplingSettings.particleRadius = boundarySettings.particleRadius;
    CoupledFluidSimulation simulation(smoothingLength, fluidConfig, couplingSettings);
    // Optionally hold the body in place while the fluid column settles.
    for (int step = 0; step < holdSteps; ++step) {
        simulation.step(particles, {&body}, 0.002f, tank);
        body.SetPosition(Vector2(0.0f, StartHeight));
        body.SetOrientation(orientation);
        body.SetVelocity(Vector2());
        body.SetAngularVelocity(0.0f);
    }
    for (int step = 0; step < steps; ++step) {
        simulation.step(particles, {&body}, 0.002f, tank);
    }
    REQUIRE(std::isfinite(body.GetPosition().y));
    REQUIRE(std::isfinite(body.GetVelocity().y));
    return {body.GetPosition().y, body.GetVelocity().y, body.GetOrientation()};
}

} // namespace

TEST_CASE("Capsule boundary samples lie on the inset outlines with the requested spacing", "[fluid][capsule][boundary]") {
    const float orientation = GENERATE(0.0f, 0.7f, -2.3f);
    const float halfLength = 0.6f, radius = 0.3f, spacing = 0.05f;
    Capsule shape(halfLength, radius);
    RigidBody body(&shape, Material{1.0f, 0.0f, 0.0f, 0.0f}, Vector2(0.4f, -0.2f));
    body.SetOrientation(orientation);
    const SpineFrame frame = FrameOf(body);

    SECTION("surface layer") {
        // A support radius of one spacing keeps only the surface layer.
        const auto samples = SampleRigidBodyBoundaries({&body}, {spacing, spacing});
        const double perimeter = 4.0 * halfLength + 2.0 * Pi * radius;
        const auto expectedCount = static_cast<std::size_t>(std::ceil(perimeter / spacing));
        REQUIRE(samples.size() == expectedCount);
        // Even arc-length spacing: one step of perimeter / count, no larger
        // than the requested spacing and within one sample of it.
        const double step = perimeter / expectedCount;
        REQUIRE(step <= spacing);
        REQUIRE(step > spacing * (expectedCount - 1) / expectedCount);
        double maximumSurfaceError = 0.0;
        for (std::size_t index = 0; index < samples.size(); ++index) {
            const FluidBoundaryParticle& sample = samples[index];
            double nx = 0.0, ny = 0.0;
            const double distance = SpineDistance(frame, sample.position.x, sample.position.y, &nx, &ny);
            maximumSurfaceError = std::max(maximumSurfaceError, std::abs(distance - radius));
            // Outward unit normal of the capsule at the sample: away from the
            // spine and from the centroid; a small step along it leaves the
            // capsule and a step against it enters.
            REQUIRE(std::hypot(nx, ny) == Catch::Approx(1.0).margin(1e-12));
            REQUIRE(nx * (sample.position.x - frame.cx) + ny * (sample.position.y - frame.cy) > 0.0);
            REQUIRE(SpineDistance(frame, sample.position.x + 1e-3 * nx, sample.position.y + 1e-3 * ny) > radius);
            REQUIRE(SpineDistance(frame, sample.position.x - 1e-3 * nx, sample.position.y - 1e-3 * ny) < radius);
            // Neighbours are one arc step apart: the chord equals it on the
            // straight sides and is at most 0.2% shorter on the caps here.
            const FluidBoundaryParticle& next = samples[(index + 1) % samples.size()];
            const double chord = std::hypot(double(next.position.x) - sample.position.x,
                                            double(next.position.y) - sample.position.y);
            REQUIRE(chord <= step + 1e-6);
            REQUIRE(chord >= step * 0.998 - 1e-6);
            REQUIRE(sample.volume == spacing * spacing);
            REQUIRE(sample.pressureScale == 0.0f);
        }
        INFO("maximum surface error " << maximumSurfaceError);
        REQUIRE(maximumSurfaceError < 2e-7 * (1.0 + std::abs(frame.cx) + std::abs(frame.cy) + halfLength + radius));
    }

    SECTION("interior layers and spine") {
        // Support deeper than the radius: layers at depth k * spacing for
        // radius 0.3 have rho = 0.3, 0.25, ..., 0.05, then the spine layer.
        const auto samples = SampleRigidBodyBoundaries({&body}, {spacing, 0.5f});
        std::size_t expectedCount = 0;
        for (int layer = 0; layer < 6; ++layer) {
            const double rho = double(radius) - layer * double(spacing);
            expectedCount += static_cast<std::size_t>(std::ceil((4.0 * halfLength + 2.0 * Pi * rho) / spacing));
        }
        const auto spineCount = static_cast<std::size_t>(std::ceil(2.0 * halfLength / spacing));
        REQUIRE(samples.size() == expectedCount + spineCount);
        std::size_t onSpine = 0;
        for (const FluidBoundaryParticle& sample : samples) {
            const double distance = SpineDistance(frame, sample.position.x, sample.position.y);
            const double layer = (radius - distance) / spacing;
            REQUIRE(distance <= radius + 1e-6);
            REQUIRE(std::abs(layer - std::round(layer)) < 1e-4);
            if (distance < 1e-5) ++onSpine;
            // Rigid sample velocity is the body's velocity at that point.
            const Vector2 expected = body.GetVelocityAtPoint(sample.position);
            REQUIRE(sample.velocity.x == Catch::Approx(expected.x).margin(1e-6));
            REQUIRE(sample.velocity.y == Catch::Approx(expected.y).margin(1e-6));
        }
        REQUIRE(onSpine == spineCount);
    }
}

TEST_CASE("Moving capsule boundary samples carry the rigid surface velocity", "[fluid][capsule][boundary]") {
    Capsule shape(0.5f, 0.25f);
    RigidBody body(&shape, Material{1.0f, 0.0f, 0.0f, 0.0f}, Vector2(1.0f, 2.0f));
    body.SetOrientation(0.4f);
    body.SetVelocity(Vector2(0.3f, -0.7f));
    body.SetAngularVelocity(2.5f);
    const auto samples = SampleRigidBodyBoundaries({&body}, {0.05f, 0.2f});
    REQUIRE_FALSE(samples.empty());
    for (const FluidBoundaryParticle& sample : samples) {
        const Vector2 offset = sample.position - body.GetPosition();
        REQUIRE(sample.velocity.x == Catch::Approx(0.3f - 2.5f * offset.y).margin(1e-5));
        REQUIRE(sample.velocity.y == Catch::Approx(-0.7f + 2.5f * offset.x).margin(1e-5));
    }
}

TEST_CASE("Zero half-length capsule samples and couples exactly like a circle", "[fluid][capsule][circle]") {
    const float radius = 0.3f;
    Capsule capsuleShape(0.0f, radius);
    Circle circleShape(radius);
    const Material material{1.0f, 0.0f, 0.0f, 0.0f};
    const float orientation = GENERATE(0.0f, 1.1f);
    RigidBody capsule(&capsuleShape, material, Vector2(0.3f, -0.2f));
    RigidBody circle(&circleShape, material, Vector2(0.3f, -0.2f));
    for (RigidBody* body : {&capsule, &circle}) {
        body->SetOrientation(orientation);
        body->SetVelocity(Vector2(0.2f, -0.4f));
        body->SetAngularVelocity(1.3f);
    }

    for (const FluidBoundarySamplingSettings settings :
         {FluidBoundarySamplingSettings{0.05f, 0.2f}, FluidBoundarySamplingSettings{0.13f, 0.5f}}) {
        const auto a = SampleRigidBodyBoundaries({&capsule}, settings);
        const auto b = SampleRigidBodyBoundaries({&circle}, settings);
        REQUIRE(a.size() == b.size());
        for (std::size_t index = 0; index < a.size(); ++index) {
            REQUIRE(SameBits(a[index].position, b[index].position));
            REQUIRE(SameBits(a[index].velocity, b[index].velocity));
            REQUIRE(SameBits(a[index].volume, b[index].volume));
        }
    }

    // One coupling pass over a particle cloud straddling the surface. The
    // surface samples are bit-identical; only the body's mass and inertia
    // come from different float formulas (Capsule rounds pi r^2 from double).
    FluidParticleProperties properties = Properties(0.2f, 0.1f);
    std::vector<FluidParticle> cloud;
    for (int i = -8; i <= 8; ++i) {
        for (int j = -8; j <= 8; ++j) {
            FluidParticle particle(Vector2(0.3f + i * 0.05f, -0.2f + j * 0.05f),
                                   Vector2(0.1f * i, -0.05f * j), properties);
            particle.pressure = 100.0f + 3.0f * j;
            cloud.push_back(particle);
        }
    }
    std::vector<FluidParticle> capsuleCloud = cloud, circleCloud = cloud;
    FluidRigidCouplingSettings settings;
    settings.particleRadius = 0.03f;
    settings.contactRestitution = 0.2f;
    FluidRigidCoupler coupler(settings);
    const auto capsuleStatistics = coupler.couple(capsuleCloud, {&capsule}, 0.004f);
    const auto circleStatistics = coupler.couple(circleCloud, {&circle}, 0.004f);
    REQUIRE(capsuleStatistics.interactionCount == circleStatistics.interactionCount);
    REQUIRE(capsuleStatistics.contactCorrectionCount == circleStatistics.contactCorrectionCount);
    INFO("capsule mass " << capsule.GetMass() << " circle mass " << circle.GetMass());
    REQUIRE(capsule.GetMass() == Catch::Approx(circle.GetMass()).epsilon(1e-6));
    REQUIRE(capsule.inertia == Catch::Approx(circle.inertia).epsilon(1e-6));
    for (std::size_t index = 0; index < cloud.size(); ++index) {
        REQUIRE(SameBits(capsuleCloud[index].position, circleCloud[index].position));
        // Contact impulses divide by the effective mass, so they inherit the
        // relative mass difference (at most 1e-6) of the body.
        REQUIRE(capsuleCloud[index].velocity.x == Catch::Approx(circleCloud[index].velocity.x).epsilon(1e-5).margin(1e-6));
        REQUIRE(capsuleCloud[index].velocity.y == Catch::Approx(circleCloud[index].velocity.y).epsilon(1e-5).margin(1e-6));
    }
    REQUIRE(capsule.GetVelocity().x == Catch::Approx(circle.GetVelocity().x).epsilon(1e-5));
    REQUIRE(capsule.GetVelocity().y == Catch::Approx(circle.GetVelocity().y).epsilon(1e-5));
    REQUIRE(capsule.GetAngularVelocity() == Catch::Approx(circle.GetAngularVelocity()).epsilon(1e-5));
}

TEST_CASE("Zero half-length capsule follows the circle trajectory in a WCSPH tank", "[fluid][capsule][circle][coupled]") {
    const float radius = GENERATE(0.3f, 0.27f);
    Capsule capsuleShape(0.0f, radius);
    Circle circleShape(radius);
    const float density = GENERATE(500.0f, 1500.0f);
    const TankResult capsule = RunTank(capsuleShape, density, 0.0f);
    const TankResult circle = RunTank(circleShape, density, 0.0f);
    INFO("radius " << radius << " density " << density << " capsule y " << capsule.y << " circle y " << circle.y);
    // Samples and surface queries are bit-identical (previous test). The
    // only possible difference is mass and inertia, which Capsule rounds from
    // double and Circle computes in float: equal to 1e-6 relative, and for
    // r = 0.3 equal to the bit. With equal bits the coupled run must be
    // bit-identical; otherwise allow 1e-4 m, two orders of magnitude above
    // what a 1e-7 relative perturbation of an O(1) m/s^2 acceleration
    // integrates to over 0.4 s.
    const RigidBody capsuleBody(&capsuleShape, Material{density, 0.0f, 0.0f, 0.0f});
    const RigidBody circleBody(&circleShape, Material{density, 0.0f, 0.0f, 0.0f});
    REQUIRE(capsuleBody.GetMass() == Catch::Approx(circleBody.GetMass()).epsilon(1e-6));
    REQUIRE(capsuleBody.inertia == Catch::Approx(circleBody.inertia).epsilon(1e-6));
    if (SameBits(capsuleBody.GetMass(), circleBody.GetMass())
        && SameBits(capsuleBody.inertia, circleBody.inertia)) {
        REQUIRE(SameBits(capsule.y, circle.y));
        REQUIRE(SameBits(capsule.vy, circle.vy));
        REQUIRE(SameBits(capsule.orientation, circle.orientation));
    } else {
        REQUIRE(capsule.y == Catch::Approx(circle.y).margin(1e-4));
        REQUIRE(capsule.vy == Catch::Approx(circle.vy).margin(1e-3));
    }
}

TEST_CASE("Hydrostatic pressure on a capsule gives Archimedes buoyancy at the centroid", "[fluid][capsule][coupling][buoyancy]") {
    // The pattern of tests/test_fluid_rigid_coupling.cpp: fluid particles on
    // the surface, one arc length apart, carrying hydrostatic pressure
    // rho g (1 - y), coupled for one step. Their pressure impulses are a
    // quadrature of the closed-surface integral of p n, which is rho g A up.
    constexpr double fluidDensity = 1000.0, gravity = 9.81;
    constexpr float deltaTime = 0.001f;
    const float orientation = GENERATE(0.0f, 0.6f, 1.5707963f);
    const float bodyDensity = GENERATE(500.0f, 1000.0f, 1500.0f);
    const double radius = 0.2, halfLength = 0.3;
    Capsule shape{float(halfLength), float(radius)};
    RigidBody body(&shape, Material{bodyDensity, 0.0f, 0.0f, 0.0f}, Vector2(0.1f, 0.2f));
    body.SetOrientation(orientation);
    const SpineFrame frame = FrameOf(body);
    constexpr int samples = 256;
    const double perimeter = 4.0 * halfLength + 2.0 * Pi * radius;
    const double arcLength = perimeter / samples;
    FluidParticleProperties properties = Properties(float(arcLength), 0.0f);
    properties.mass = float(fluidDensity * arcLength * arcLength);
    std::vector<FluidParticle> particles;
    for (int index = 0; index < samples; ++index) {
        // Independent outline walk: right cap, top side, left cap, bottom side.
        double s = (index + 0.5) * arcLength, lx = 0.0, ly = 0.0;
        const double cap = Pi * radius;
        if (s < cap) {
            const double a = -0.5 * Pi + s / radius; lx = halfLength + radius * std::cos(a); ly = radius * std::sin(a);
        } else if ((s -= cap) < 2.0 * halfLength) {
            lx = halfLength - s; ly = radius;
        } else if ((s -= 2.0 * halfLength) < cap) {
            const double a = 0.5 * Pi + s / radius; lx = -halfLength + radius * std::cos(a); ly = radius * std::sin(a);
        } else {
            s -= cap; lx = -halfLength + s; ly = -radius;
        }
        const Vector2 position(float(frame.cx + frame.ux * lx - frame.uy * ly),
                               float(frame.cy + frame.uy * lx + frame.ux * ly));
        FluidParticle particle(position, Vector2(), properties);
        particle.volume = float(arcLength * arcLength);
        particle.pressure = float(fluidDensity * gravity * (1.0 - position.y));
        particles.push_back(particle);
    }
    FluidRigidCouplingSettings settings;
    settings.particleRadius = 0.0001f;
    settings.viscosityScale = 0.0f;
    FluidRigidCoupler coupler(settings);
    coupler.couple(particles, {&body}, deltaTime);

    const double impulseX = double(body.GetVelocity().x) * body.GetMass();
    const double impulseY = double(body.GetVelocity().y) * body.GetMass();
    const double archimedes = fluidDensity * gravity * shape.GetArea() * deltaTime;
    INFO("orientation " << orientation << " impulse " << impulseX << ", " << impulseY
         << " archimedes " << archimedes << " angular " << body.GetAngularVelocity());
    REQUIRE(impulseY == Catch::Approx(archimedes).epsilon(2e-3));
    REQUIRE(std::abs(impulseX) < 2e-3 * archimedes);
    // Buoyancy of a submerged body acts at its centroid, the capsule
    // center: the torque impulse is negligible against the force times the
    // capsule's half extent.
    REQUIRE(std::abs(double(body.GetAngularVelocity()) * body.inertia)
        < 2e-3 * archimedes * (halfLength + radius));
    // Adding gravity gives the float / hover / sink direction of the body.
    const double netVelocity = body.GetVelocity().y - gravity * deltaTime;
    const double expected = gravity * deltaTime * (fluidDensity / bodyDensity - 1.0);
    REQUIRE(netVelocity == Catch::Approx(expected).margin(3e-3 * gravity * deltaTime * fluidDensity / bodyDensity));
}

TEST_CASE("Capsules float, hover and sink in a WCSPH tank like the equal-area circle", "[fluid][capsule][coupled][buoyancy]") {
    // Equal area with the 0.3 m circle: 4 h r + pi r^2 = pi 0.3^2 at r = 0.2.
    const float capsuleRadius = 0.2f;
    const float halfLength = static_cast<float>((Pi * 0.09 - Pi * 0.04) / (4.0 * 0.2));
    Capsule capsuleShape(halfLength, capsuleRadius);
    Circle circleShape(0.3f);
    REQUIRE(capsuleShape.GetArea() == Catch::Approx(circleShape.GetArea()).epsilon(1e-6));
    const auto family = GENERATE(SphKernelFamily::Poly6Spiky, SphKernelFamily::CubicSpline);

    // Rise after 0.4 s from rest, the protocol of the existing coupled-tank
    // test. The unrelaxed fluid column first settles, so every body drops
    // about 0.05 m in the first 0.1 s before buoyancy builds, and the end
    // state is still transient: the neutral circle itself drifts.
    const float densities[3] = {500.0f, 1000.0f, 1500.0f};
    const float orientations[3] = {0.0f, 0.6f, 1.5707963f};
    float circleRise[3], capsuleRise[3][3];
    for (int d = 0; d < 3; ++d) {
        circleRise[d] = RunTank(circleShape, densities[d], 0.0f, family).y - StartHeight;
        for (int o = 0; o < 3; ++o) {
            capsuleRise[o][d] = RunTank(capsuleShape, densities[d], orientations[o], family).y - StartHeight;
        }
    }
    INFO("kernel family " << static_cast<int>(family));
    INFO("circle rise light/neutral/heavy " << circleRise[0] << " " << circleRise[1] << " " << circleRise[2]);
    INFO("capsule at 0 rise " << capsuleRise[0][0] << " " << capsuleRise[0][1] << " " << capsuleRise[0][2]);
    INFO("capsule at 0.6 rise " << capsuleRise[1][0] << " " << capsuleRise[1][1] << " " << capsuleRise[1][2]);
    INFO("capsule at pi/2 rise " << capsuleRise[2][0] << " " << capsuleRise[2][1] << " " << capsuleRise[2][2]);
    for (int o = 0; o < 3; ++o) {
        const float light = capsuleRise[o][0], neutral = capsuleRise[o][1], heavy = capsuleRise[o][2];
        // The existing coupled-tank criterion, then direction and ordering.
        REQUIRE(light > heavy + 0.05f);
        REQUIRE(light > 0.0f);
        REQUIRE(heavy < 0.0f);
        REQUIRE(light > neutral);
        REQUIRE(neutral > heavy);
        // Neutral hovers: it moves less than the circle's own light rise or
        // heavy sink.
        REQUIRE(std::abs(neutral) < std::min(circleRise[0], -circleRise[2]));
    }
    // Tilted and broadside capsules track the circle at neutral and heavy
    // density within 2 cm. End-on (pi/2) the 0.79 m tall capsule leaves 0.2 m
    // fluid voids at both tips on the 0.15 m lattice, which refill and push
    // it up, so only the ordering above is required there.
    for (int o = 0; o < 2; ++o) {
        REQUIRE(capsuleRise[o][1] == Catch::Approx(circleRise[1]).margin(0.02));
        REQUIRE(capsuleRise[o][2] == Catch::Approx(circleRise[2]).margin(0.02));
    }
    // Same buoyancy, different drag and added mass: the light capsule moving
    // broadside (0.79 m wide) rises slower than the 0.6 m circle, and end-on
    // (0.4 m wide) faster.
    REQUIRE(capsuleRise[0][0] < circleRise[0]);
    REQUIRE(capsuleRise[2][0] > circleRise[0]);
}

TEST_CASE("Fluid impulses on a rotated capsule conserve momentum and angular momentum", "[fluid][capsule][coupling][momentum]") {
    Capsule shape(0.5f, 0.25f);
    RigidBody body(&shape, Material{2.0f, 0.0f, 0.0f, 0.0f}, Vector2(0.2f, -0.1f));
    body.SetOrientation(0.9f);
    body.SetVelocity(Vector2(0.1f, 0.05f));
    body.SetAngularVelocity(0.3f);
    const SpineFrame frame = FrameOf(body);

    // Particles in a ring 0.05 m outside the surface, beyond the contact
    // radius, so only pressure acts. The pressure impulse lies along the
    // surface normal through the particle, so it also conserves angular
    // momentum about any point.
    FluidParticleProperties properties = Properties(0.2f, 0.0f);
    std::vector<FluidParticle> particles;
    for (int index = 0; index < 48; ++index) {
        const double angle = 2.0 * Pi * index / 48;
        const double x = frame.cx + (frame.h + 0.3) * std::cos(angle);
        const double y = frame.cy + (frame.h * 0.5 + 0.3) * std::sin(angle);
        // Project onto the offset curve at distance r + 0.05 from the spine.
        double nx = 0.0, ny = 0.0;
        const double d = SpineDistance(frame, x, y, &nx, &ny);
        const double scale = (frame.r + 0.05) - d;
        FluidParticle particle(Vector2(float(x + nx * scale), float(y + ny * scale)),
                               Vector2(0.2f * float(std::cos(angle)), 0.0f), properties);
        particle.pressure = 400.0f + 200.0f * float(std::sin(3.0 * angle));
        particles.push_back(particle);
    }

    SECTION("pressure only") {
        FluidRigidCouplingSettings settings;
        settings.particleRadius = 0.01f;
        settings.viscosityScale = 0.0f;
        FluidRigidCoupler coupler(settings);
        const Momentum before = TotalMomentum(particles, body);
        const std::vector<FluidParticle> original = particles;
        const auto statistics = coupler.couple(particles, {&body}, 0.01f);
        const Momentum after = TotalMomentum(particles, body);
        REQUIRE(statistics.interactionCount == particles.size());
        REQUIRE(statistics.contactCorrectionCount == 0);
        REQUIRE(statistics.pressureImpulseMagnitude > 0.1f);
        INFO("linear " << after.x - before.x << ", " << after.y - before.y
             << " angular " << after.angular - before.angular);
        REQUIRE(after.x == Catch::Approx(before.x).margin(1e-5));
        REQUIRE(after.y == Catch::Approx(before.y).margin(1e-5));
        REQUIRE(after.angular == Catch::Approx(before.angular).margin(1e-5));

        // Each particle is pushed along the oracle outward normal, and the
        // body's torque equals the sum of the reaction moments.
        double expectedTorqueImpulse = 0.0;
        for (std::size_t index = 0; index < particles.size(); ++index) {
            const double jx = double(particles[index].velocity.x) - original[index].velocity.x;
            const double jy = double(particles[index].velocity.y) - original[index].velocity.y;
            double nx = 0.0, ny = 0.0;
            SpineDistance(frame, original[index].position.x, original[index].position.y, &nx, &ny);
            const double magnitude = std::hypot(jx, jy);
            REQUIRE(magnitude > 0.0);
            REQUIRE((jx * nx + jy * ny) / magnitude == Catch::Approx(1.0).margin(1e-4));
            const double armX = double(original[index].position.x) - frame.cx;
            const double armY = double(original[index].position.y) - frame.cy;
            expectedTorqueImpulse -= armX * jy - armY * jx;
        }
        const double torqueImpulse = (double(body.GetAngularVelocity()) - 0.3) * body.inertia;
        INFO("torque impulse " << torqueImpulse << " expected " << expectedTorqueImpulse);
        REQUIRE(std::abs(expectedTorqueImpulse) > 1e-3);
        REQUIRE(torqueImpulse == Catch::Approx(expectedTorqueImpulse).epsilon(1e-4));
    }

    SECTION("pressure, viscosity and contact conserve linear momentum") {
        // Pull half the ring inside the contact radius and add viscosity.
        for (std::size_t index = 0; index < particles.size(); index += 2) {
            double nx = 0.0, ny = 0.0;
            SpineDistance(frame, particles[index].position.x, particles[index].position.y, &nx, &ny);
            particles[index].position = particles[index].position - Vector2(float(nx), float(ny)) * 0.06f;
            particles[index].velocity = Vector2(-float(nx), -float(ny));
            particles[index].viscosity = 0.3f;
        }
        FluidRigidCouplingSettings settings;
        settings.particleRadius = 0.03f;
        settings.contactRestitution = 0.3f;
        FluidRigidCoupler coupler(settings);
        const Momentum before = TotalMomentum(particles, body);
        const auto statistics = coupler.couple(particles, {&body}, 0.01f);
        const Momentum after = TotalMomentum(particles, body);
        REQUIRE(statistics.contactCorrectionCount == particles.size() / 2);
        REQUIRE(statistics.viscosityImpulseMagnitude > 0.0f);
        REQUIRE(after.x == Catch::Approx(before.x).margin(1e-5));
        REQUIRE(after.y == Catch::Approx(before.y).margin(1e-5));
        // Coupling applies impulses only, so the body has not moved, and every
        // projected contact sits the contact radius outside the surface.
        for (std::size_t index = 0; index < particles.size(); index += 2) {
            const FluidParticle& particle = particles[index];
            const double d = SpineDistance(frame, particle.position.x, particle.position.y);
            REQUIRE(d == Catch::Approx(frame.r + settings.particleRadius).margin(1e-5));
        }
    }
}

TEST_CASE("A particle hitting a rotated capsule is projected out without rebound", "[fluid][capsule][coupling][contact]") {
    const float orientation = GENERATE(0.0f, 0.8f, 2.0f);
    Capsule shape(0.5f, 0.25f);
    RigidBody body(&shape, Material{1.0f, 0.0f, 0.0f, 0.0f}, Vector2(0.1f, 0.2f));
    body.SetOrientation(orientation);
    body.SetAngularVelocity(0.4f);
    const SpineFrame frame = FrameOf(body);
    // One particle on the side, one beyond a cap, each 0.02 m inside the
    // surface and moving into it.
    for (const double along : {0.2, 0.73}) {
        const double side = along < frame.h ? 1.0 : 0.0;
        const double px = frame.cx + frame.ux * along - frame.uy * side * 0.23;
        const double py = frame.cy + frame.uy * along + frame.ux * side * 0.23;
        double nx = 0.0, ny = 0.0;
        SpineDistance(frame, px, py, &nx, &ny);
        FluidParticle particle(Vector2(float(px), float(py)), Vector2(-float(nx), -float(ny)),
                               Properties(0.3f, 0.0f));
        particle.pressure = 0.0f;
        std::vector<FluidParticle> particles = {particle};
        RigidBody target(&shape, Material{1.0f, 0.0f, 0.0f, 0.0f}, body.GetPosition());
        target.SetOrientation(orientation);
        target.SetAngularVelocity(0.4f);
        FluidRigidCouplingSettings settings;
        settings.particleRadius = 0.1f;
        settings.pressureScale = 0.0f;
        settings.viscosityScale = 0.0f;
        FluidRigidCoupler coupler(settings);
        const Momentum before = TotalMomentum(particles, target);
        const auto statistics = coupler.couple(particles, {&target}, 0.001f);
        const Momentum after = TotalMomentum(particles, target);
        REQUIRE(statistics.contactCorrectionCount == 1);
        REQUIRE(statistics.maximumPenetration == Catch::Approx(0.12f).margin(1e-5));
        const double d = SpineDistance(frame, particles.front().position.x, particles.front().position.y);
        REQUIRE(d == Catch::Approx(frame.r + 0.1).margin(1e-5));
        const Vector2 surfacePoint(float(px - nx * 0.23 + nx * frame.r), float(py - ny * 0.23 + ny * frame.r));
        const Vector2 relative = particles.front().velocity - target.GetVelocityAtPoint(surfacePoint);
        REQUIRE(relative.x * nx + relative.y * ny >= -1e-5);
        REQUIRE(after.x == Catch::Approx(before.x).margin(1e-5));
        REQUIRE(after.y == Catch::Approx(before.y).margin(1e-5));
    }
}

TEST_CASE("Off-axis pressure under a capsule turns it the expected way", "[fluid][capsule][coupling][torque]") {
    Capsule shape(0.5f, 0.2f);
    RigidBody body(&shape, Material{1.0f, 0.0f, 0.0f, 0.0f}, Vector2());
    FluidParticle particle(Vector2(0.4f, -0.21f), Vector2(), Properties(0.3f, 0.0f));
    particle.pressure = 1000.0f;
    std::vector<FluidParticle> particles = {particle};
    FluidRigidCouplingSettings settings;
    settings.particleRadius = 0.001f;
    settings.viscosityScale = 0.0f;
    FluidRigidCoupler coupler(settings);
    coupler.couple(particles, {&body}, 0.001f);
    // Pushed up at x = +0.4: rises and turns counterclockwise; the flat side
    // gives no sideways push.
    REQUIRE(body.GetVelocity().y > 0.0f);
    REQUIRE(body.GetVelocity().x == 0.0f);
    REQUIRE(body.GetAngularVelocity() > 0.0f);
    REQUIRE(particles.front().velocity.y < 0.0f);
}

TEST_CASE("Capsule fluid coupling and sampling keep their validation", "[fluid][capsule][validation]") {
    Capsule shape(0.5f, 0.25f);
    Material material{1.0f, 0.0f, 0.0f, 0.0f};
    RigidBody capsule(&shape, material, Vector2());
    Circle circleShape(0.5f);
    RigidBody circle(&circleShape, material, Vector2(3.0f, 0.0f));
    FluidParticle particle(Vector2(2.6f, 0.0f), Vector2(1.0f, 0.0f));
    std::vector<FluidParticle> particles = {particle};
    FluidRigidCoupler coupler;

    // Accepted now, and a null body after a capsule still throws before any
    // particle or body is changed.
    {
        RigidBody acceptedCapsule(&shape, material, Vector2());
        RigidBody acceptedCircle(&circleShape, material, Vector2(3.0f, 0.0f));
        REQUIRE_NOTHROW(coupler.couple(particles, {&acceptedCapsule, &acceptedCircle}, 0.01f));
        REQUIRE(acceptedCircle.GetVelocity().x > 0.0f);
    }
    particles = {particle};
    REQUIRE_THROWS_AS(coupler.couple(particles, {&capsule, &circle, nullptr}, 0.01f), std::invalid_argument);
    REQUIRE(particles.front().velocity == particle.velocity);
    REQUIRE(circle.GetVelocity() == Vector2());
    REQUIRE(capsule.GetVelocity() == Vector2());
    REQUIRE_THROWS_AS(coupler.couple(particles, {&capsule}, -0.001f), std::invalid_argument);
    REQUIRE_THROWS_AS(coupler.couple(particles, {&capsule}, std::numeric_limits<float>::infinity()),
                      std::invalid_argument);
    particles.front().pressure = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(coupler.couple(particles, {&capsule}, 0.001f), std::invalid_argument);

    // Sampling: invalid settings, work budget and unrepresentable velocities.
    REQUIRE_THROWS_AS(SampleRigidBodyBoundaries({&capsule}, {0.0f, 0.5f}), std::invalid_argument);
    REQUIRE_THROWS_AS(SampleRigidBodyBoundaries({&capsule, nullptr}, {0.1f, 0.5f}), std::invalid_argument);
    REQUIRE_THROWS_AS(SampleRigidBodyBoundaries({&capsule}, {0.0001f, 0.25f}), std::length_error);
    std::vector<RigidBody*> repeated(5000, &capsule);
    REQUIRE_THROWS_AS(SampleRigidBodyBoundaries(repeated, {0.01f, 0.01f}), std::length_error);
    Capsule large(4.0f, 2.0f);
    RigidBody spinning(&large, material, Vector2(), true);
    spinning.SetAngularVelocity(std::numeric_limits<float>::max());
    REQUIRE_THROWS_AS(SampleRigidBodyBoundaries({&spinning}, {0.1f, 0.1f}), std::overflow_error);
    // A finite capsule whose tip lies beyond float range.
    const float maximum = std::numeric_limits<float>::max();
    Capsule huge(1e24f, 1.0f);
    RigidBody far(&huge, material, Vector2(maximum, 0.0f), true);
    REQUIRE_THROWS_AS(SampleRigidBodyBoundaries({&far}, {1e19f, 1e19f}), std::overflow_error);

    // A sampling failure publishes nothing from the failed body.
    CoupledFluidSimulation simulation(0.3f);
    std::vector<FluidParticle> none;
    REQUIRE_THROWS_AS(simulation.step(none, {&spinning}, 0.001f), std::overflow_error);
}
