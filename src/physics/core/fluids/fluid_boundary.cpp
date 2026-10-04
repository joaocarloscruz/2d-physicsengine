#include "physics/core/fluids/fluid_boundary.h"

#include "physics/core/rigidbody.h"
#include "../checked_grid.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {

constexpr float BoundaryTolerance = 1e-6f;
constexpr double Pi = 3.14159265358979323846;

float CheckedGeometryFloat(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
        throw std::overflow_error("Fluid boundary geometry exceeds float range.");
    }
    return static_cast<float>(value);
}

Vector2 CheckedGeometryVector(double x, double y) {
    return Vector2(CheckedGeometryFloat(x), CheckedGeometryFloat(y));
}

float RoundInward(double value, double direction) {
    float rounded = CheckedGeometryFloat(value);
    // A nearest float can land outside the permitted half-plane. Round toward
    // the correction direction by one ULP when necessary, retaining the radius.
    if ((direction > 0.0 && rounded < value) || (direction < 0.0 && rounded > value)) {
        rounded = std::nextafter(rounded, direction > 0.0
            ? std::numeric_limits<float>::infinity()
            : -std::numeric_limits<float>::infinity());
        CheckedGeometryFloat(rounded);
    }
    return rounded;
}

double EdgeSide(const Vector2& start, const Vector2& end, double x, double y) {
    const double dx = static_cast<double>(end.x) - start.x;
    const double dy = static_cast<double>(end.y) - start.y;
    return dx * (y - start.y) - dy * (x - start.x);
}

bool CounterClockwise(const std::vector<Vector2>& vertices) {
    return EdgeSide(vertices[0], vertices[1], vertices[2].x, vertices[2].y) > 0.0;
}

void RemoveOutwardVelocity(FluidParticle& particle, double normalX, double normalY,
                          const FluidBoundarySettings& settings) {
    double velocityX = particle.velocity.x;
    double velocityY = particle.velocity.y;
    const double outwardSpeed = velocityX * normalX + velocityY * normalY;
    if (outwardSpeed > 0.0) {
        const double impulse = (1.0 + settings.restitution) * outwardSpeed;
        velocityX -= normalX * impulse;
        velocityY -= normalY * impulse;
    }
    const double normalSpeed = velocityX * normalX + velocityY * normalY;
    const double tangentX = velocityX - normalX * normalSpeed;
    const double tangentY = velocityY - normalY * normalSpeed;
    const double tangentSpeed = std::hypot(tangentX, tangentY);
    if (tangentSpeed > 0.0 && outwardSpeed > 0.0) {
        const double frictionDelta = std::min(tangentSpeed,
            settings.friction * (1.0 + settings.restitution) * outwardSpeed);
        velocityX -= tangentX * (frictionDelta / tangentSpeed);
        velocityY -= tangentY * (frictionDelta / tangentSpeed);
    }
    particle.velocity = CheckedGeometryVector(velocityX, velocityY);
}

void FiniteVector(const Vector2& value) {
    if (!std::isfinite(value.x) || !std::isfinite(value.y)) {
        throw std::invalid_argument("Fluid boundary geometry state must be finite.");
    }
}

void ValidateSamplingState(const Vector2& position, const Vector2& velocity,
                           float angularVelocity) {
    FiniteVector(position);
    FiniteVector(velocity);
    if (!std::isfinite(angularVelocity)) {
        throw std::invalid_argument("Fluid boundary angular velocity must be finite.");
    }
}

void PublishSamples(const std::vector<FluidBoundaryParticle>& generated,
                    std::vector<FluidBoundaryParticle>& particles) {
    for (const auto& sample : generated) {
        FiniteVector(sample.position);
        FiniteVector(sample.velocity);
    }
    particles.insert(particles.end(), generated.begin(), generated.end());
}

void AppendCircleSamples(
    const Vector2& center,
    float radius,
    bool sampleInside,
    float pressureScale,
    const Vector2& linearVelocity,
    float angularVelocity,
    const FluidBoundarySamplingSettings& settings,
    std::vector<FluidBoundaryParticle>& particles,
    std::uint64_t& remainingSamples
) {
    ValidateSamplingState(center, linearVelocity, angularVelocity);
    CheckedGrid::PositiveFinite(radius);
    const int layerCount = CheckedGrid::Extent(settings.supportRadius, settings.spacing);
    if (static_cast<std::uint64_t>(layerCount) > remainingSamples) {
        throw std::length_error("Fluid boundary sampling exceeds its layer budget.");
    }
    std::vector<std::pair<double, int>> layers;
    for (int layer = 0; layer < layerCount; ++layer) {
        const double layerRadius = sampleInside
            ? static_cast<double>(radius) - static_cast<double>(layer) * settings.spacing
            : static_cast<double>(radius) + static_cast<double>(layer) * settings.spacing;
        if (layerRadius <= BoundaryTolerance) {
            CheckedGrid::Charge(1, remainingSamples);
            layers.emplace_back(0.0f, 1);
            break;
        }
        if (layerRadius > std::numeric_limits<float>::max()) {
            throw std::overflow_error("Fluid boundary layer radius exceeds float range.");
        }
        const int sampleCount = std::max(1, CheckedGrid::Integer(
            std::ceil(2.0 * Pi * layerRadius / settings.spacing)
        ));
        CheckedGrid::Charge(static_cast<std::uint64_t>(sampleCount), remainingSamples);
        layers.emplace_back(layerRadius, sampleCount);
    }
    std::vector<FluidBoundaryParticle> generated;
    for (const auto& [layerRadius, sampleCount] : layers) {
        if (layerRadius == 0.0f) {
            generated.push_back({center, linearVelocity,
                settings.spacing * settings.spacing, Vector2(), pressureScale});
            break;
        }
        for (int index = 0; index < sampleCount; ++index) {
            const double angle = 2.0 * Pi * index / sampleCount;
            const double offsetX = layerRadius * std::cos(angle);
            const double offsetY = layerRadius * std::sin(angle);
            generated.push_back({
                CheckedGeometryVector(center.x + offsetX, center.y + offsetY),
                CheckedGeometryVector(linearVelocity.x - angularVelocity * offsetY,
                                      linearVelocity.y + angularVelocity * offsetX),
                settings.spacing * settings.spacing,
                Vector2(),
                pressureScale
            });
        }
    }
    PublishSamples(generated, particles);
}

bool IsInsideConvex(const std::vector<Vector2>& vertices, double x, double y,
                    bool counterClockwise, std::size_t sourceEdge) {
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        // Sampling constructs points on this edge plus an inward normal offset.
        // Re-evaluating that same plane can lose surface samples to cancellation.
        if (index == sourceEdge) {
            continue;
        }
        const double side = EdgeSide(vertices[index], vertices[(index + 1) % vertices.size()], x, y);
        if ((counterClockwise && side < -BoundaryTolerance)
            || (!counterClockwise && side > BoundaryTolerance)) {
            return false;
        }
    }
    return true;
}

void AppendPolygonSamples(
    const std::vector<Vector2>& vertices,
    const Vector2& position,
    float orientation,
    bool sampleInside,
    float pressureScale,
    const Vector2& linearVelocity,
    float angularVelocity,
    const FluidBoundarySamplingSettings& settings,
    std::vector<FluidBoundaryParticle>& particles,
    std::uint64_t& remainingSamples
) {
    ValidateSamplingState(position, linearVelocity, angularVelocity);
    if (!std::isfinite(orientation) || vertices.size() < 3) {
        throw std::invalid_argument("Fluid boundary polygon must have finite orientation and vertices.");
    }
    for (const auto& vertex : vertices) {
        FiniteVector(vertex);
    }
    const bool counterClockwise = CounterClockwise(vertices);
    const double cosine = std::cos(static_cast<double>(orientation));
    const double sine = std::sin(static_cast<double>(orientation));
    const int layerCount = CheckedGrid::Extent(settings.supportRadius, settings.spacing);
    std::vector<int> edgeCounts;
    for (std::size_t edgeIndex = 0; edgeIndex < vertices.size(); ++edgeIndex) {
        const auto& start = vertices[edgeIndex];
        const auto& end = vertices[(edgeIndex + 1) % vertices.size()];
        const double length = std::hypot(static_cast<double>(end.x) - start.x,
                                         static_cast<double>(end.y) - start.y);
        const int sampleCount = std::max(1, CheckedGrid::Integer(
            std::ceil(length / settings.spacing)
        ));
        const auto attempts = static_cast<std::uint64_t>(sampleCount) * layerCount;
        // Inside tests visit every polygon edge for each attempted point.
        const auto costPerSample = sampleInside ? vertices.size() : 1;
        if (attempts > remainingSamples / costPerSample) {
            throw std::length_error("Fluid boundary sampling exceeds its work budget.");
        }
        CheckedGrid::Charge(attempts * costPerSample, remainingSamples);
        edgeCounts.push_back(sampleCount);
    }
    std::vector<FluidBoundaryParticle> generated;
    for (std::size_t edgeIndex = 0; edgeIndex < vertices.size(); ++edgeIndex) {
        const Vector2 start = vertices[edgeIndex];
        const Vector2 end = vertices[(edgeIndex + 1) % vertices.size()];
        const double edgeX = static_cast<double>(end.x) - start.x;
        const double edgeY = static_cast<double>(end.y) - start.y;
        const double length = std::hypot(edgeX, edgeY);
        const double inwardX = (counterClockwise ? -edgeY : edgeY) / length;
        const double inwardY = (counterClockwise ? edgeX : -edgeX) / length;
        const int sampleCount = edgeCounts[edgeIndex];
        for (int layer = 0; layer < layerCount; ++layer) {
            const double layerDistance = static_cast<double>(layer) * settings.spacing
                * (sampleInside ? 1.0 : -1.0);
            for (int index = 0; index < sampleCount; ++index) {
                const double parameter = (static_cast<double>(index) + 0.5) / sampleCount;
                const double localX = start.x + edgeX * parameter + inwardX * layerDistance;
                const double localY = start.y + edgeY * parameter + inwardY * layerDistance;
                if (sampleInside && !IsInsideConvex(vertices, localX, localY, counterClockwise, edgeIndex)) {
                    continue;
                }
                const double offsetX = cosine * localX - sine * localY;
                const double offsetY = sine * localX + cosine * localY;
                generated.push_back({
                    CheckedGeometryVector(position.x + offsetX, position.y + offsetY),
                    CheckedGeometryVector(linearVelocity.x - angularVelocity * offsetY,
                                          linearVelocity.y + angularVelocity * offsetX),
                    settings.spacing * settings.spacing, Vector2(), pressureScale
                });
            }
        }
    }
    PublishSamples(generated, particles);
}

} // namespace

void FluidBoundarySamplingSettings::Validate() const {
    if (!std::isfinite(spacing) || spacing <= 0.0f
        || !std::isfinite(supportRadius) || supportRadius <= 0.0f
        || !std::isfinite(spacing * spacing) || spacing * spacing <= 0.0f) {
        throw std::invalid_argument(
            "Fluid boundary sampling dimensions and sample volume must be positive and finite."
        );
    }
}

void FluidBoundarySettings::Validate() const {
    if (!std::isfinite(particleRadius) || particleRadius <= 0.0f) {
        throw std::invalid_argument(
            "Fluid boundary particle radius must be positive and finite."
        );
    }
    if (!std::isfinite(restitution) || restitution < 0.0f || restitution > 1.0f) {
        throw std::invalid_argument(
            "Fluid boundary restitution must be finite and between zero and one."
        );
    }
    if (!std::isfinite(friction) || friction < 0.0f || friction > 1.0f) {
        throw std::invalid_argument(
            "Fluid boundary friction must be finite and between zero and one."
        );
    }
}

void IFluidContainer::appendBoundaryParticles(
    const FluidBoundarySamplingSettings&,
    std::vector<FluidBoundaryParticle>&
) const {}

FluidCircleContainer::FluidCircleContainer(
    const Vector2& containerCenter,
    float containerRadius,
    const FluidBoundarySettings& boundarySettings
) : center(containerCenter),
    radius(containerRadius),
    settings(boundarySettings) {
    settings.Validate();
    if (!std::isfinite(center.x) || !std::isfinite(center.y)
        || !std::isfinite(radius) || radius <= settings.particleRadius) {
        throw std::invalid_argument(
            "Fluid circle container must be finite and larger than the particle radius."
        );
    }
}

bool FluidCircleContainer::contains(const Vector2& position) const {
    FiniteVector(position);
    const double permittedRadius = static_cast<double>(radius) - settings.particleRadius;
    const double dx = static_cast<double>(position.x) - center.x;
    const double dy = static_cast<double>(position.y) - center.y;
    return dx * dx + dy * dy <= permittedRadius * permittedRadius + BoundaryTolerance;
}

void FluidCircleContainer::appendBoundaryParticles(
    const FluidBoundarySamplingSettings& sampling,
    std::vector<FluidBoundaryParticle>& particles
) const {
    sampling.Validate();
    std::uint64_t remainingSamples = CheckedGrid::MaximumSamples;
    CheckedGrid::Charge(particles.size(), remainingSamples);
    AppendCircleSamples(
        center,
        radius,
        false,
        1.0f,
        Vector2(),
        0.0f,
        sampling,
        particles,
        remainingSamples
    );
}

FluidBoundaryCorrection FluidCircleContainer::enforce(
    FluidParticle& particle
) const {
    FiniteVector(particle.position);
    FiniteVector(particle.velocity);
    const double dx = static_cast<double>(particle.position.x) - center.x;
    const double dy = static_cast<double>(particle.position.y) - center.y;
    const double distance = std::hypot(dx, dy);
    const double permittedRadius = static_cast<double>(radius) - settings.particleRadius;
    if (distance <= permittedRadius) {
        return FluidBoundaryCorrection{};
    }

    const double normalX = dx / distance;
    const double normalY = dy / distance;
    const float penetration = CheckedGeometryFloat(distance - permittedRadius);
    FluidParticle projected = particle;
    projected.position = Vector2(
        RoundInward(center.x + normalX * permittedRadius, -normalX),
        RoundInward(center.y + normalY * permittedRadius, -normalY)
    );
    RemoveOutwardVelocity(projected, normalX, normalY, settings);
    if (!contains(projected.position)) {
        throw std::runtime_error("Fluid circle boundary projection is not representable.");
    }
    particle = projected;
    return FluidBoundaryCorrection{true, penetration};
}

FluidConvexPolygonContainer::FluidConvexPolygonContainer(
    std::vector<Vector2> containerVertices,
    const FluidBoundarySettings& boundarySettings
) : vertices(std::move(containerVertices)),
    settings(boundarySettings) {
    settings.Validate();
    // Reuse the rigid shape's global half-plane validation. It rejects stars,
    // repeated/collinear vertices and concavity without a unit-dependent cutoff.
    const Polygon validatedOutline(vertices);
    if (!CounterClockwise(vertices)) {
        std::reverse(vertices.begin(), vertices.end());
    }
    inwardNormals.reserve(vertices.size());
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        const auto& start = vertices[index];
        const auto& end = vertices[(index + 1) % vertices.size()];
        const double dx = static_cast<double>(end.x) - start.x;
        const double dy = static_cast<double>(end.y) - start.y;
        const double length = std::hypot(dx, dy);
        inwardNormals.emplace_back(-dy / length, dx / length);
    }
}

bool FluidConvexPolygonContainer::contains(const Vector2& position) const {
    FiniteVector(position);
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        const double distance = (static_cast<double>(position.x) - vertices[index].x)
                * inwardNormals[index].first
            + (static_cast<double>(position.y) - vertices[index].y)
                * inwardNormals[index].second;
        if (distance + BoundaryTolerance < settings.particleRadius) {
            return false;
        }
    }
    return true;
}

void FluidConvexPolygonContainer::appendBoundaryParticles(
    const FluidBoundarySamplingSettings& sampling,
    std::vector<FluidBoundaryParticle>& particles
) const {
    sampling.Validate();
    std::uint64_t remainingSamples = CheckedGrid::MaximumSamples;
    CheckedGrid::Charge(particles.size(), remainingSamples);
    AppendPolygonSamples(
        vertices,
        Vector2(),
        0.0f,
        false,
        1.0f,
        Vector2(),
        0.0f,
        sampling,
        particles,
        remainingSamples
    );
}

FluidBoundaryCorrection FluidConvexPolygonContainer::enforce(
    FluidParticle& particle
) const {
    FiniteVector(particle.position);
    FiniteVector(particle.velocity);
    FluidParticle projected = particle;
    FluidBoundaryCorrection result;
    const std::size_t maximumPasses = vertices.size() * 2;
    for (std::size_t pass = 0; pass < maximumPasses; ++pass) {
        double minimumDistance = std::numeric_limits<double>::max();
        std::size_t edgeIndex = 0;
        for (std::size_t index = 0; index < vertices.size(); ++index) {
            const double distance = (static_cast<double>(projected.position.x) - vertices[index].x)
                    * inwardNormals[index].first
                + (static_cast<double>(projected.position.y) - vertices[index].y)
                    * inwardNormals[index].second;
            if (distance < minimumDistance) {
                minimumDistance = distance;
                edgeIndex = index;
            }
        }
        if (minimumDistance + BoundaryTolerance >= settings.particleRadius) {
            break;
        }
        const double penetration = settings.particleRadius - minimumDistance;
        const auto& normal = inwardNormals[edgeIndex];
        projected.position = Vector2(
            RoundInward(projected.position.x + normal.first * penetration, normal.first),
            RoundInward(projected.position.y + normal.second * penetration, normal.second)
        );
        RemoveOutwardVelocity(projected, -normal.first, -normal.second, settings);
        result.corrected = true;
        result.penetration = std::max(result.penetration, CheckedGeometryFloat(penetration));
    }
    if (!contains(projected.position)) {
        throw std::runtime_error(
            "Fluid polygon boundary could not project particle into its valid region."
        );
    }
    particle = projected;
    return result;
}

FluidBoundaryStatistics EnforceFluidBoundary(
    const IFluidContainer& boundary,
    std::vector<FluidParticle>& particles
) {
    FluidBoundaryStatistics statistics;
    for (FluidParticle& particle : particles) {
        const FluidBoundaryCorrection correction = boundary.enforce(particle);
        if (correction.corrected) {
            ++statistics.correctedParticleCount;
            statistics.maximumPenetration = std::max(
                statistics.maximumPenetration,
                correction.penetration
            );
        }
    }
    return statistics;
}

std::vector<FluidBoundaryParticle> SampleFluidContainerBoundary(
    const IFluidContainer& boundary,
    const FluidBoundarySamplingSettings& settings
) {
    settings.Validate();
    std::vector<FluidBoundaryParticle> particles;
    boundary.appendBoundaryParticles(settings, particles);
    return particles;
}

std::vector<FluidBoundaryParticle> SampleRigidBodyBoundaries(
    const std::vector<RigidBody*>& bodies,
    const FluidBoundarySamplingSettings& settings
) {
    settings.Validate();
    std::vector<FluidBoundaryParticle> particles;
    std::uint64_t remainingSamples = CheckedGrid::MaximumSamples;
    for (const RigidBody* body : bodies) {
        if (body == nullptr || !body->shape) {
            throw std::invalid_argument(
                "Fluid boundary sampling requires valid rigid bodies."
            );
        }
        if (body->shape->type == ShapeType::CIRCLE) {
            AppendCircleSamples(
                body->GetPosition(),
                body->shape->GetRadius(),
                true,
                0.0f,
                body->GetVelocity(),
                body->GetAngularVelocity(),
                settings,
                particles,
                remainingSamples
            );
        } else if (body->shape->type == ShapeType::POLYGON) {
            const auto* polygon = static_cast<const Polygon*>(body->shape.get());
            AppendPolygonSamples(
                polygon->getVertices(),
                body->GetPosition(),
                body->GetOrientation(),
                true,
                0.0f,
                body->GetVelocity(),
                body->GetAngularVelocity(),
                settings,
                particles,
                remainingSamples
            );
        } else {
            throw std::invalid_argument(
                "Fluid boundary sampling received an unsupported shape."
            );
        }
    }
    return particles;
}

}
