#include "physics/core/fluids/fluid_boundary.h"

#include "physics/core/rigidbody.h"
#include "physics/math/matrix2x2.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {

constexpr float BoundaryTolerance = 1e-6f;
constexpr float Pi = 3.14159265358979323846f;

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
    const float tangentSpeed = tangentVelocity.magnitude();
    if (tangentSpeed > 0.0f && outwardSpeed > 0.0f) {
        const float frictionDelta = std::min(
            tangentSpeed,
            settings.friction * (1.0f + settings.restitution) * outwardSpeed
        );
        particle.velocity = particle.velocity
            - tangentVelocity * (frictionDelta / tangentSpeed);
    }
}

float SignedDoubleArea(const std::vector<Vector2>& vertices) {
    float area = 0.0f;
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        area += vertices[index].cross(vertices[(index + 1) % vertices.size()]);
    }
    return area;
}

void AppendCircleSamples(
    const Vector2& center,
    float radius,
    bool sampleInside,
    float pressureScale,
    const Vector2& linearVelocity,
    float angularVelocity,
    const FluidBoundarySamplingSettings& settings,
    std::vector<FluidBoundaryParticle>& particles
) {
    const int layerCount = std::max(
        1,
        static_cast<int>(std::ceil(settings.supportRadius / settings.spacing))
    );
    for (int layer = 0; layer < layerCount; ++layer) {
        const float layerRadius = sampleInside
            ? radius - static_cast<float>(layer) * settings.spacing
            : radius + static_cast<float>(layer) * settings.spacing;
        if (layerRadius <= BoundaryTolerance) {
            particles.push_back({
                center,
                linearVelocity,
                settings.spacing * settings.spacing,
                Vector2(),
                pressureScale
            });
            break;
        }
        const int sampleCount = std::max(
            1,
            static_cast<int>(std::ceil(2.0f * Pi * layerRadius / settings.spacing))
        );
        for (int index = 0; index < sampleCount; ++index) {
            const float angle = 2.0f * Pi * static_cast<float>(index)
                / static_cast<float>(sampleCount);
            const Vector2 offset(
                layerRadius * std::cos(angle),
                layerRadius * std::sin(angle)
            );
            particles.push_back({
                center + offset,
                linearVelocity + Vector2::cross(angularVelocity, offset),
                settings.spacing * settings.spacing,
                Vector2(),
                pressureScale
            });
        }
    }
}

bool IsInsideConvex(
    const std::vector<Vector2>& vertices,
    const Vector2& position,
    bool counterClockwise
) {
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        const Vector2 edge = vertices[(index + 1) % vertices.size()]
            - vertices[index];
        const float side = edge.cross(position - vertices[index]);
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
    std::vector<FluidBoundaryParticle>& particles
) {
    const bool counterClockwise = SignedDoubleArea(vertices) > 0.0f;
    const Matrix2x2 rotation = Matrix2x2::rotation(orientation);
    const int layerCount = std::max(
        1,
        static_cast<int>(std::ceil(settings.supportRadius / settings.spacing))
    );
    for (std::size_t edgeIndex = 0; edgeIndex < vertices.size(); ++edgeIndex) {
        const Vector2 start = vertices[edgeIndex];
        const Vector2 edge = vertices[(edgeIndex + 1) % vertices.size()] - start;
        const float length = edge.magnitude();
        const int sampleCount = std::max(
            1,
            static_cast<int>(std::ceil(length / settings.spacing))
        );
        const Vector2 inward = counterClockwise
            ? Vector2(-edge.y, edge.x).normalized()
            : Vector2(edge.y, -edge.x).normalized();
        for (int layer = 0; layer < layerCount; ++layer) {
            const Vector2 layerOffset = inward
                * (static_cast<float>(layer) * settings.spacing
                    * (sampleInside ? 1.0f : -1.0f));
            for (int index = 0; index < sampleCount; ++index) {
                const float parameter = (static_cast<float>(index) + 0.5f)
                    / static_cast<float>(sampleCount);
                const Vector2 localPoint = start + edge * parameter + layerOffset;
                if (sampleInside
                    && !IsInsideConvex(vertices, localPoint, counterClockwise)) {
                    continue;
                }
                const Vector2 worldOffset = rotation * localPoint;
                particles.push_back({
                    position + worldOffset,
                    linearVelocity + Vector2::cross(angularVelocity, worldOffset),
                    settings.spacing * settings.spacing,
                    Vector2(),
                    pressureScale
                });
            }
        }
    }
}

} // namespace

void FluidBoundarySamplingSettings::Validate() const {
    if (!std::isfinite(spacing) || spacing <= 0.0f
        || !std::isfinite(supportRadius) || supportRadius <= 0.0f) {
        throw std::invalid_argument(
            "Fluid boundary sampling dimensions must be positive and finite."
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
    const float permittedRadius = radius - settings.particleRadius;
    return (position - center).magnitudeSquared()
        <= permittedRadius * permittedRadius + BoundaryTolerance;
}

void FluidCircleContainer::appendBoundaryParticles(
    const FluidBoundarySamplingSettings& sampling,
    std::vector<FluidBoundaryParticle>& particles
) const {
    sampling.Validate();
    AppendCircleSamples(
        center,
        radius,
        false,
        1.0f,
        Vector2(),
        0.0f,
        sampling,
        particles
    );
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

void FluidConvexPolygonContainer::appendBoundaryParticles(
    const FluidBoundarySamplingSettings& sampling,
    std::vector<FluidBoundaryParticle>& particles
) const {
    sampling.Validate();
    AppendPolygonSamples(
        vertices,
        Vector2(),
        0.0f,
        false,
        1.0f,
        Vector2(),
        0.0f,
        sampling,
        particles
    );
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
    for (const RigidBody* body : bodies) {
        if (body == nullptr) {
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
                particles
            );
        } else if (body->shape->type == ShapeType::POLYGON) {
            const auto* polygon = static_cast<const Polygon*>(body->shape);
            AppendPolygonSamples(
                polygon->getVertices(),
                body->GetPosition(),
                body->GetOrientation(),
                true,
                0.0f,
                body->GetVelocity(),
                body->GetAngularVelocity(),
                settings,
                particles
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
