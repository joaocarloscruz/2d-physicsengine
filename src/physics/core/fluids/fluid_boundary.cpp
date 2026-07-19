#include "physics/core/fluids/fluid_boundary.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {

constexpr float BoundaryTolerance = 1e-6f;

void RemoveOutwardVelocity(
    FluidParticle& particle,
    const Vector2& outwardNormal,
    const FluidBoundarySettings& settings
) {
    const float outwardSpeed = particle.velocity.dot(outwardNormal);
    if (outwardSpeed > 0.0f) {
        particle.velocity = particle.velocity
            - outwardNormal * ((1.0f + settings.restitution) * outwardSpeed);
    }
    const float remainingNormalSpeed = particle.velocity.dot(outwardNormal);
    const Vector2 tangentVelocity = particle.velocity
        - outwardNormal * remainingNormalSpeed;
    particle.velocity = particle.velocity - tangentVelocity * settings.friction;
}

float SignedDoubleArea(const std::vector<Vector2>& vertices) {
    float area = 0.0f;
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        area += vertices[index].cross(vertices[(index + 1) % vertices.size()]);
    }
    return area;
}

} // namespace

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
    const float permittedRadius = radius - settings.particleRadius;
    return (position - center).magnitudeSquared()
        <= permittedRadius * permittedRadius + BoundaryTolerance;
}

FluidBoundaryCorrection FluidCircleContainer::enforce(
    FluidParticle& particle
) const {
    const Vector2 offset = particle.position - center;
    const float distance = offset.magnitude();
    const float permittedRadius = radius - settings.particleRadius;
    if (distance <= permittedRadius) {
        return FluidBoundaryCorrection{};
    }

    const Vector2 outwardNormal = distance > 0.0f
        ? offset / distance
        : Vector2(1.0f, 0.0f);
    const float penetration = distance - permittedRadius;
    particle.position = center + outwardNormal * permittedRadius;
    RemoveOutwardVelocity(particle, outwardNormal, settings);
    return FluidBoundaryCorrection{true, penetration};
}

FluidConvexPolygonContainer::FluidConvexPolygonContainer(
    std::vector<Vector2> containerVertices,
    const FluidBoundarySettings& boundarySettings
) : vertices(std::move(containerVertices)),
    settings(boundarySettings) {
    settings.Validate();
    if (vertices.size() < 3) {
        throw std::invalid_argument(
            "Fluid polygon container requires at least three vertices."
        );
    }
    for (const Vector2& vertex : vertices) {
        if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y)) {
            throw std::invalid_argument(
                "Fluid polygon vertices must be finite."
            );
        }
    }
    const float area = SignedDoubleArea(vertices);
    if (std::abs(area) <= BoundaryTolerance) {
        throw std::invalid_argument(
            "Fluid polygon container must have nonzero area."
        );
    }
    if (area < 0.0f) {
        std::reverse(vertices.begin(), vertices.end());
    }

    inwardNormals.reserve(vertices.size());
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        const Vector2 edge = vertices[(index + 1) % vertices.size()]
            - vertices[index];
        if (edge.magnitudeSquared() <= BoundaryTolerance * BoundaryTolerance) {
            throw std::invalid_argument(
                "Fluid polygon edges must be non-degenerate."
            );
        }
        inwardNormals.push_back(Vector2(-edge.y, edge.x).normalized());

        const Vector2 nextEdge = vertices[(index + 2) % vertices.size()]
            - vertices[(index + 1) % vertices.size()];
        if (edge.cross(nextEdge) <= BoundaryTolerance) {
            throw std::invalid_argument(
                "Fluid polygon container must be strictly convex."
            );
        }
    }
}

bool FluidConvexPolygonContainer::contains(const Vector2& position) const {
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        const float distance = (position - vertices[index]).dot(
            inwardNormals[index]
        );
        if (distance + BoundaryTolerance < settings.particleRadius) {
            return false;
        }
    }
    return true;
}

FluidBoundaryCorrection FluidConvexPolygonContainer::enforce(
    FluidParticle& particle
) const {
    FluidBoundaryCorrection result;
    const std::size_t maximumPasses = vertices.size() * 2;
    for (std::size_t pass = 0; pass < maximumPasses; ++pass) {
        float minimumDistance = std::numeric_limits<float>::max();
        std::size_t edgeIndex = 0;
        for (std::size_t index = 0; index < vertices.size(); ++index) {
            const float distance = (particle.position - vertices[index]).dot(
                inwardNormals[index]
            );
            if (distance < minimumDistance) {
                minimumDistance = distance;
                edgeIndex = index;
            }
        }
        if (minimumDistance + BoundaryTolerance >= settings.particleRadius) {
            break;
        }

        const float penetration = settings.particleRadius - minimumDistance;
        particle.position = particle.position
            + inwardNormals[edgeIndex] * penetration;
        RemoveOutwardVelocity(
            particle,
            inwardNormals[edgeIndex] * -1.0f,
            settings
        );
        result.corrected = true;
        result.penetration = std::max(result.penetration, penetration);
    }
    if (!contains(particle.position)) {
        throw std::runtime_error(
            "Fluid polygon boundary could not project particle into its valid region."
        );
    }
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

}
