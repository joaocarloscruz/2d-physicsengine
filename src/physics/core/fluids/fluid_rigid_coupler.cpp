#include "physics/core/fluids/fluid_rigid_coupler.h"

#include "physics/math/matrix2x2.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {

constexpr float CouplingTolerance = 1e-6f;

struct SurfaceSample {
    Vector2 point;
    Vector2 outwardNormal;
    float signedDistance;
};

SurfaceSample SampleCircle(const RigidBody& body, const Vector2& position) {
    const Vector2 offset = position - body.GetPosition();
    const float distance = offset.magnitude();
    const Vector2 normal = distance > CouplingTolerance
        ? offset / distance
        : Vector2(1.0f, 0.0f);
    const float radius = body.shape->GetRadius();
    return SurfaceSample{
        body.GetPosition() + normal * radius,
        normal,
        distance - radius
    };
}

float SignedDoubleArea(const std::vector<Vector2>& vertices) {
    float area = 0.0f;
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        area += vertices[index].cross(vertices[(index + 1) % vertices.size()]);
    }
    return area;
}

SurfaceSample SamplePolygon(const RigidBody& body, const Vector2& position) {
    const auto* polygon = static_cast<const Polygon*>(body.shape);
    const auto& vertices = polygon->getVertices();
    const Matrix2x2 toLocal = Matrix2x2::rotation(-body.GetOrientation());
    const Matrix2x2 toWorld = Matrix2x2::rotation(body.GetOrientation());
    const Vector2 localPosition = toLocal * (position - body.GetPosition());
    const bool counterClockwise = SignedDoubleArea(vertices) > 0.0f;

    bool inside = true;
    float minimumDistanceSquared = std::numeric_limits<float>::max();
    Vector2 closestPoint;
    Vector2 closestOutwardNormal(1.0f, 0.0f);
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        const Vector2& start = vertices[index];
        const Vector2 edge = vertices[(index + 1) % vertices.size()] - start;
        const float side = edge.cross(localPosition - start);
        if ((counterClockwise && side < -CouplingTolerance)
            || (!counterClockwise && side > CouplingTolerance)) {
            inside = false;
        }

        const float edgeLengthSquared = edge.magnitudeSquared();
        const float parameter = std::clamp(
            (localPosition - start).dot(edge) / edgeLengthSquared,
            0.0f,
            1.0f
        );
        const Vector2 candidate = start + edge * parameter;
        const float distanceSquared = (localPosition - candidate).magnitudeSquared();
        if (distanceSquared < minimumDistanceSquared) {
            minimumDistanceSquared = distanceSquared;
            closestPoint = candidate;
            closestOutwardNormal = counterClockwise
                ? Vector2(edge.y, -edge.x).normalized()
                : Vector2(-edge.y, edge.x).normalized();
        }
    }

    const float distance = std::sqrt(minimumDistanceSquared);
    Vector2 localNormal = closestOutwardNormal;
    if (!inside && distance > CouplingTolerance) {
        localNormal = (localPosition - closestPoint) / distance;
    }
    return SurfaceSample{
        body.GetPosition() + toWorld * closestPoint,
        (toWorld * localNormal).normalized(),
        inside ? -distance : distance
    };
}

SurfaceSample SampleSurface(const RigidBody& body, const Vector2& position) {
    if (body.shape->type == ShapeType::CIRCLE) {
        return SampleCircle(body, position);
    }
    if (body.shape->type == ShapeType::POLYGON) {
        return SamplePolygon(body, position);
    }
    throw std::invalid_argument("Fluid-rigid coupling received an unsupported shape.");
}

void ApplyPairImpulse(
    FluidParticle& particle,
    RigidBody& body,
    const Vector2& impulseOnParticle,
    const Vector2& surfacePoint
) {
    particle.velocity = particle.velocity + impulseOnParticle * particle.inverseMass;
    body.ApplyImpulse(
        impulseOnParticle * -1.0f,
        surfacePoint - body.GetPosition()
    );
}

} // namespace

void FluidRigidCouplingSettings::Validate() const {
    if (!std::isfinite(particleRadius) || particleRadius <= 0.0f) {
        throw std::invalid_argument(
            "Fluid-rigid particle radius must be positive and finite."
        );
    }
    if (!std::isfinite(pressureScale) || pressureScale < 0.0f
        || !std::isfinite(viscosityScale) || viscosityScale < 0.0f) {
        throw std::invalid_argument(
            "Fluid-rigid force scales must be finite and non-negative."
        );
    }
    if (!std::isfinite(contactRestitution)
        || contactRestitution < 0.0f || contactRestitution > 1.0f) {
        throw std::invalid_argument(
            "Fluid-rigid restitution must be finite and between zero and one."
        );
    }
}

FluidRigidCoupler::FluidRigidCoupler(
    const FluidRigidCouplingSettings& couplingSettings
) : settings(couplingSettings) {
    settings.Validate();
}

FluidRigidCouplingStatistics FluidRigidCoupler::couple(
    std::vector<FluidParticle>& particles,
    const std::vector<RigidBody*>& bodies,
    float deltaTime
) const {
    if (!std::isfinite(deltaTime) || deltaTime < 0.0f) {
        throw std::invalid_argument(
            "Fluid-rigid delta time must be finite and non-negative."
        );
    }
    for (const RigidBody* body : bodies) {
        if (body == nullptr) {
            throw std::invalid_argument(
                "Fluid-rigid coupling requires valid rigid bodies."
            );
        }
    }

    FluidRigidCouplingStatistics statistics;
    for (FluidParticle& particle : particles) {
        if (!std::isfinite(particle.position.x)
            || !std::isfinite(particle.position.y)
            || !std::isfinite(particle.velocity.x)
            || !std::isfinite(particle.velocity.y)
            || !std::isfinite(particle.pressure)
            || !std::isfinite(particle.viscosity)
            || particle.viscosity < 0.0f
            || !std::isfinite(particle.volume)
            || particle.volume <= 0.0f
            || !std::isfinite(particle.smoothingLength)
            || particle.smoothingLength <= 0.0f
            || !std::isfinite(particle.inverseMass)
            || particle.inverseMass <= 0.0f) {
            throw std::invalid_argument(
                "Fluid-rigid coupling requires finite, positive particle state."
            );
        }
        for (RigidBody* body : bodies) {
            const SurfaceSample surface = SampleSurface(*body, particle.position);
            if (surface.signedDistance > particle.smoothingLength) {
                continue;
            }
            ++statistics.interactionCount;

            const float influence = std::max(
                1.0f - std::max(surface.signedDistance, 0.0f)
                    / particle.smoothingLength,
                0.0f
            );
            const float boundaryLength = particle.volume / particle.smoothingLength;
            const Vector2 pressureImpulse = surface.outwardNormal
                * (particle.pressure * boundaryLength * influence
                    * settings.pressureScale * deltaTime);
            ApplyPairImpulse(particle, *body, pressureImpulse, surface.point);
            statistics.pressureImpulseMagnitude += pressureImpulse.magnitude();

            const Vector2 surfaceVelocity = body->GetVelocityAtPoint(surface.point);
            const Vector2 viscosityImpulse = (surfaceVelocity - particle.velocity)
                * (particle.viscosity * particle.volume
                    / (particle.smoothingLength * particle.smoothingLength)
                    * influence * settings.viscosityScale * deltaTime);
            ApplyPairImpulse(particle, *body, viscosityImpulse, surface.point);
            statistics.viscosityImpulseMagnitude += viscosityImpulse.magnitude();

            if (surface.signedDistance >= settings.particleRadius) {
                continue;
            }
            ++statistics.contactCorrectionCount;
            const float penetration = settings.particleRadius - surface.signedDistance;
            statistics.maximumPenetration = std::max(
                statistics.maximumPenetration,
                penetration
            );
            particle.position = surface.point
                + surface.outwardNormal * settings.particleRadius;

            const Vector2 contactVector = surface.point - body->GetPosition();
            const Vector2 relativeVelocity = particle.velocity
                - body->GetVelocityAtPoint(surface.point);
            const float normalSpeed = relativeVelocity.dot(surface.outwardNormal);
            if (normalSpeed >= 0.0f) {
                continue;
            }
            const float angularTerm = contactVector.cross(surface.outwardNormal);
            const float effectiveInverseMass = particle.inverseMass
                + body->GetInverseMass()
                + angularTerm * angularTerm * body->GetInverseInertia();
            if (effectiveInverseMass <= CouplingTolerance) {
                continue;
            }
            const float impulseMagnitude = -(1.0f + settings.contactRestitution)
                * normalSpeed / effectiveInverseMass;
            ApplyPairImpulse(
                particle,
                *body,
                surface.outwardNormal * impulseMagnitude,
                surface.point
            );
        }
    }
    return statistics;
}

} // namespace PhysicsEngine
