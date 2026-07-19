#include "physics/core/collisions/collision_resolver.h"

#include "physics/core/rigidbody.h"
#include "physics/math/matrix2x2.h"

#include <algorithm>
#include <cmath>

namespace PhysicsEngine {
namespace {

Vector2 ContactTangent(const Vector2& normal) {
    return Vector2(-normal.y, normal.x);
}

std::uint8_t EffectiveContactCount(const CollisionManifold& manifold) {
    return manifold.contactCount > 0
        ? std::min<std::uint8_t>(manifold.contactCount, 2)
        : static_cast<std::uint8_t>(manifold.hasCollision ? 1 : 0);
}

ContactPoint EffectiveContact(
    const CollisionManifold& manifold,
    std::uint8_t index
) {
    if (manifold.contactCount > 0) {
        return manifold.contacts[index];
    }
    return ContactPoint{manifold.contactPoint, manifold.penetration, 0};
}

Vector2 ToLocalPoint(const RigidBody* body, const Vector2& worldPoint) {
    return Matrix2x2::rotation(-body->GetOrientation())
        * (worldPoint - body->GetPosition());
}

Vector2 ToWorldPoint(const RigidBody* body, const Vector2& localPoint) {
    return body->GetPosition()
        + Matrix2x2::rotation(body->GetOrientation()) * localPoint;
}

Vector2 ConstraintPointPosition(
    const ContactConstraint& constraint,
    const ContactConstraintPoint& point
) {
    return (ToWorldPoint(constraint.bodyA, point.localAnchorA)
        + ToWorldPoint(constraint.bodyB, point.localAnchorB)) * 0.5f;
}

float EffectiveMass(
    const RigidBody* bodyA,
    const RigidBody* bodyB,
    const Vector2& ra,
    const Vector2& rb,
    const Vector2& direction
) {
    const float raCrossDirection = ra.cross(direction);
    const float rbCrossDirection = rb.cross(direction);
    const float inverseMass = bodyA->GetInverseMass()
        + bodyB->GetInverseMass()
        + raCrossDirection * raCrossDirection * bodyA->GetInverseInertia()
        + rbCrossDirection * rbCrossDirection * bodyB->GetInverseInertia();
    return inverseMass > 0.0f ? 1.0f / inverseMass : 0.0f;
}

void SynchronizeCache(
    const CollisionManifold& manifold,
    ContactImpulseCache& cache
) {
    const ContactImpulseCache previous = cache;
    cache = ContactImpulseCache{};
    cache.contactCount = EffectiveContactCount(manifold);
    for (std::uint8_t i = 0; i < cache.contactCount; ++i) {
        const ContactPoint point = EffectiveContact(manifold, i);
        cache.contacts[i].featureId = point.featureId;
        for (std::uint8_t oldIndex = 0;
             oldIndex < previous.contactCount;
             ++oldIndex) {
            if (previous.contacts[oldIndex].featureId == point.featureId) {
                cache.contacts[i].impulse = previous.contacts[oldIndex].impulse;
                break;
            }
        }
    }
}

void ApplyVelocityImpulse(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vector2& point,
    const Vector2& impulse
) {
    bodyA->ApplyImpulse(impulse * -1.0f, point - bodyA->GetPosition());
    bodyB->ApplyImpulse(impulse, point - bodyB->GetPosition());
}

void ApplyPositionImpulse(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vector2& ra,
    const Vector2& rb,
    const Vector2& impulse
) {
    if (!bodyA->IsStatic()) {
        bodyA->SetPosition(
            bodyA->GetPosition() - impulse * bodyA->GetInverseMass()
        );
        bodyA->SetOrientation(
            bodyA->GetOrientation()
                - bodyA->GetInverseInertia() * ra.cross(impulse)
        );
    }
    if (!bodyB->IsStatic()) {
        bodyB->SetPosition(
            bodyB->GetPosition() + impulse * bodyB->GetInverseMass()
        );
        bodyB->SetOrientation(
            bodyB->GetOrientation()
                + bodyB->GetInverseInertia() * rb.cross(impulse)
        );
    }
}

} // namespace

ContactConstraint CollisionResolver::PrepareConstraint(
    const CollisionManifold& manifold,
    ContactImpulseCache& cache,
    const SimulationConfig& config
) {
    ContactConstraint constraint;
    if (!manifold.hasCollision || !manifold.A || !manifold.B) {
        cache = ContactImpulseCache{};
        return constraint;
    }

    SynchronizeCache(manifold, cache);
    constraint.bodyA = manifold.A;
    constraint.bodyB = manifold.B;
    constraint.normal = manifold.normal;
    constraint.tangent = ContactTangent(manifold.normal);
    constraint.pointCount = cache.contactCount;
    constraint.staticFriction = std::sqrt(
        manifold.A->material.staticFriction
            * manifold.B->material.staticFriction
    );
    constraint.dynamicFriction = std::sqrt(
        manifold.A->material.dynamicFriction
            * manifold.B->material.dynamicFriction
    );
    constraint.positionCorrectionFactor = config.positionCorrectionFactor;
    constraint.penetrationSlop = config.penetrationSlop;
    constraint.maxPositionCorrection = config.maxPositionCorrection;
    constraint.velocityTolerance = config.velocityTolerance;
    constraint.cache = &cache;

    for (std::uint8_t i = 0; i < constraint.pointCount; ++i) {
        const ContactPoint contact = EffectiveContact(manifold, i);
        ContactConstraintPoint& point = constraint.points[i];
        point.localAnchorA = ToLocalPoint(manifold.A, contact.position);
        point.localAnchorB = ToLocalPoint(manifold.B, contact.position);
        point.penetration = contact.penetration > 0.0f
            ? contact.penetration
            : manifold.penetration;
        point.featureId = contact.featureId;
        point.impulse = cache.contacts[i].impulse;

        const Vector2 ra = contact.position - manifold.A->GetPosition();
        const Vector2 rb = contact.position - manifold.B->GetPosition();
        point.normalMass = EffectiveMass(
            manifold.A,
            manifold.B,
            ra,
            rb,
            constraint.normal
        );
        point.tangentMass = EffectiveMass(
            manifold.A,
            manifold.B,
            ra,
            rb,
            constraint.tangent
        );

        const float normalVelocity = (
            manifold.B->GetVelocityAtPoint(contact.position)
                - manifold.A->GetVelocityAtPoint(contact.position)
        ).dot(constraint.normal);
        if (normalVelocity < -config.restitutionVelocityThreshold) {
            const float restitution = std::max(
                manifold.A->material.restitution,
                manifold.B->material.restitution
            );
            point.velocityBias = -restitution * normalVelocity;
        }
    }
    return constraint;
}

void CollisionResolver::WarmStart(ContactConstraint& constraint) {
    if (!constraint.bodyA || !constraint.bodyB) {
        return;
    }
    for (std::uint8_t i = 0; i < constraint.pointCount; ++i) {
        ContactConstraintPoint& point = constraint.points[i];
        const Vector2 impulse = constraint.normal * point.impulse.normal
            + constraint.tangent * point.impulse.tangent;
        ApplyVelocityImpulse(
            constraint.bodyA,
            constraint.bodyB,
            ConstraintPointPosition(constraint, point),
            impulse
        );
    }
}

void CollisionResolver::SolveVelocity(ContactConstraint& constraint) {
    if (!constraint.bodyA || !constraint.bodyB) {
        return;
    }
    for (std::uint8_t i = 0; i < constraint.pointCount; ++i) {
        ContactConstraintPoint& point = constraint.points[i];
        const Vector2 worldPoint = ConstraintPointPosition(constraint, point);
        Vector2 relativeVelocity = constraint.bodyB->GetVelocityAtPoint(worldPoint)
            - constraint.bodyA->GetVelocityAtPoint(worldPoint);
        const float normalVelocity = relativeVelocity.dot(constraint.normal);
        const float previousNormalImpulse = point.impulse.normal;
        point.impulse.normal = std::max(
            previousNormalImpulse
                + point.normalMass * (-normalVelocity + point.velocityBias),
            0.0f
        );
        ApplyVelocityImpulse(
            constraint.bodyA,
            constraint.bodyB,
            worldPoint,
            constraint.normal
                * (point.impulse.normal - previousNormalImpulse)
        );

        relativeVelocity = constraint.bodyB->GetVelocityAtPoint(worldPoint)
            - constraint.bodyA->GetVelocityAtPoint(worldPoint);
        const float tangentVelocity = relativeVelocity.dot(constraint.tangent);
        if (std::abs(tangentVelocity) > constraint.velocityTolerance) {
            const float previousTangentImpulse = point.impulse.tangent;
            const float candidate = previousTangentImpulse
                - point.tangentMass * tangentVelocity;
            const float maximumStatic = point.impulse.normal
                * constraint.staticFriction;
            if (std::abs(candidate) <= maximumStatic) {
                point.impulse.tangent = candidate;
            } else {
                const float maximumDynamic = point.impulse.normal
                    * constraint.dynamicFriction;
                point.impulse.tangent = std::clamp(
                    candidate,
                    -maximumDynamic,
                    maximumDynamic
                );
            }
            ApplyVelocityImpulse(
                constraint.bodyA,
                constraint.bodyB,
                worldPoint,
                constraint.tangent
                    * (point.impulse.tangent - previousTangentImpulse)
            );
        }

        if (constraint.cache) {
            constraint.cache->contacts[i].impulse = point.impulse;
        }
    }
}

bool CollisionResolver::SolvePosition(ContactConstraint& constraint) {
    if (!constraint.bodyA || !constraint.bodyB) {
        return true;
    }
    float minimumSeparation = 0.0f;
    for (std::uint8_t i = 0; i < constraint.pointCount; ++i) {
        ContactConstraintPoint& point = constraint.points[i];
        const Vector2 anchorA = ToWorldPoint(
            constraint.bodyA,
            point.localAnchorA
        );
        const Vector2 anchorB = ToWorldPoint(
            constraint.bodyB,
            point.localAnchorB
        );
        const Vector2 ra = anchorA - constraint.bodyA->GetPosition();
        const Vector2 rb = anchorB - constraint.bodyB->GetPosition();
        const float separation = (anchorB - anchorA).dot(constraint.normal)
            - point.penetration;
        minimumSeparation = std::min(minimumSeparation, separation);
        const float correction = std::clamp(
            constraint.positionCorrectionFactor
                * (separation + constraint.penetrationSlop),
            -constraint.maxPositionCorrection,
            0.0f
        );
        const float positionMass = EffectiveMass(
            constraint.bodyA,
            constraint.bodyB,
            ra,
            rb,
            constraint.normal
        );
        ApplyPositionImpulse(
            constraint.bodyA,
            constraint.bodyB,
            ra,
            rb,
            constraint.normal * (-positionMass * correction)
        );
    }
    return minimumSeparation >= -3.0f * constraint.penetrationSlop;
}

void CollisionResolver::WarmStart(
    const CollisionManifold& manifold,
    const ContactImpulse& accumulatedImpulse
) {
    if (!manifold.hasCollision || !manifold.A || !manifold.B) {
        return;
    }
    const ContactPoint point = EffectiveContact(manifold, 0);
    const Vector2 impulse = manifold.normal * accumulatedImpulse.normal
        + ContactTangent(manifold.normal) * accumulatedImpulse.tangent;
    ApplyVelocityImpulse(manifold.A, manifold.B, point.position, impulse);
}

void CollisionResolver::WarmStart(
    const CollisionManifold& manifold,
    ContactImpulseCache& cache
) {
    ContactConstraint constraint = PrepareConstraint(
        manifold,
        cache,
        SimulationConfig{}
    );
    WarmStart(constraint);
}

void CollisionResolver::Resolve(const CollisionManifold& manifold) {
    ContactImpulseCache cache;
    Resolve(manifold, cache, SimulationConfig{});
}

void CollisionResolver::Resolve(
    const CollisionManifold& manifold,
    ContactImpulse& accumulatedImpulse,
    bool reduceWarmStart
) {
    (void)reduceWarmStart;
    ContactImpulseCache cache;
    cache.contactCount = 1;
    cache.contacts[0] = CachedContactImpulse{
        EffectiveContact(manifold, 0).featureId,
        accumulatedImpulse
    };
    Resolve(manifold, cache, SimulationConfig{});
    accumulatedImpulse = cache.contactCount > 0
        ? cache.contacts[0].impulse
        : ContactImpulse{};
}

void CollisionResolver::Resolve(
    const CollisionManifold& manifold,
    ContactImpulseCache& cache,
    const SimulationConfig& config,
    bool reduceWarmStart
) {
    (void)reduceWarmStart;
    ContactConstraint constraint = PrepareConstraint(manifold, cache, config);
    SolveVelocity(constraint);
    SolvePosition(constraint);
}

} // namespace PhysicsEngine
