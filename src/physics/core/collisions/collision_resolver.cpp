#include "physics/core/collisions/collision_resolver.h"

#include "physics/core/rigidbody.h"

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

void ApplyContactImpulse(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vector2& contactPoint,
    const Vector2& impulse
) {
    const Vector2 ra = contactPoint - bodyA->GetPosition();
    const Vector2 rb = contactPoint - bodyB->GetPosition();
    bodyA->ApplyImpulse(impulse * -1.0f, ra);
    bodyB->ApplyImpulse(impulse, rb);
}

void ResolveContactVelocity(
    const CollisionManifold& manifold,
    const ContactPoint& point,
    ContactImpulse& accumulatedImpulse,
    bool reduceWarmStart
) {
    RigidBody* bodyA = manifold.A;
    RigidBody* bodyB = manifold.B;
    const float totalInverseMass = bodyA->GetInverseMass()
        + bodyB->GetInverseMass();
    const Vector2 ra = point.position - bodyA->GetPosition();
    const Vector2 rb = point.position - bodyB->GetPosition();
    Vector2 relativeVelocity = bodyB->GetVelocityAtPoint(point.position)
        - bodyA->GetVelocityAtPoint(point.position);
    const float relativeVelocityAlongNormal = relativeVelocity.dot(manifold.normal);

    if (relativeVelocityAlongNormal > 0.0f
        && (!reduceWarmStart || accumulatedImpulse.normal <= 0.0f)) {
        return;
    }

    const float restitution = relativeVelocityAlongNormal < 0.0f
        ? std::max(bodyA->material.restitution, bodyB->material.restitution)
        : 0.0f;
    const float raCrossN = ra.cross(manifold.normal);
    const float rbCrossN = rb.cross(manifold.normal);
    const float denominator = totalInverseMass
        + raCrossN * raCrossN * bodyA->GetInverseInertia()
        + rbCrossN * rbCrossN * bodyB->GetInverseInertia();
    if (denominator <= 0.0f) {
        return;
    }

    float impulseScalar = -(1.0f + restitution)
        * relativeVelocityAlongNormal / denominator;
    const float previousNormalImpulse = accumulatedImpulse.normal;
    accumulatedImpulse.normal = std::max(
        previousNormalImpulse + impulseScalar,
        0.0f
    );
    impulseScalar = accumulatedImpulse.normal - previousNormalImpulse;
    ApplyContactImpulse(
        bodyA,
        bodyB,
        point.position,
        manifold.normal * impulseScalar
    );

    relativeVelocity = bodyB->GetVelocityAtPoint(point.position)
        - bodyA->GetVelocityAtPoint(point.position);
    const Vector2 tangent = ContactTangent(manifold.normal);
    if (std::abs(relativeVelocity.dot(tangent)) <= 0.0001f) {
        return;
    }

    const float raCrossT = ra.cross(tangent);
    const float rbCrossT = rb.cross(tangent);
    const float tangentDenominator = totalInverseMass
        + raCrossT * raCrossT * bodyA->GetInverseInertia()
        + rbCrossT * rbCrossT * bodyB->GetInverseInertia();
    if (tangentDenominator <= 0.0f) {
        return;
    }

    const float tangentImpulse = -relativeVelocity.dot(tangent)
        / tangentDenominator;
    const float staticFriction = std::sqrt(
        bodyA->material.staticFriction * bodyB->material.staticFriction
    );
    const float dynamicFriction = std::sqrt(
        bodyA->material.dynamicFriction * bodyB->material.dynamicFriction
    );
    const float previousTangentImpulse = accumulatedImpulse.tangent;
    const float candidate = previousTangentImpulse + tangentImpulse;
    const float maximumStatic = accumulatedImpulse.normal * staticFriction;
    if (std::abs(candidate) <= maximumStatic) {
        accumulatedImpulse.tangent = candidate;
    } else {
        const float maximumDynamic = accumulatedImpulse.normal * dynamicFriction;
        accumulatedImpulse.tangent = std::clamp(
            candidate,
            -maximumDynamic,
            maximumDynamic
        );
    }
    ApplyContactImpulse(
        bodyA,
        bodyB,
        point.position,
        tangent * (accumulatedImpulse.tangent - previousTangentImpulse)
    );
}

} // namespace

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
    ApplyContactImpulse(manifold.A, manifold.B, point.position, impulse);
}

void CollisionResolver::WarmStart(
    const CollisionManifold& manifold,
    ContactImpulseCache& cache
) {
    if (!manifold.hasCollision || !manifold.A || !manifold.B) {
        return;
    }
    SynchronizeCache(manifold, cache);
    for (std::uint8_t i = 0; i < cache.contactCount; ++i) {
        const ContactPoint point = EffectiveContact(manifold, i);
        const ContactImpulse& accumulated = cache.contacts[i].impulse;
        const Vector2 impulse = manifold.normal * accumulated.normal
            + ContactTangent(manifold.normal) * accumulated.tangent;
        ApplyContactImpulse(manifold.A, manifold.B, point.position, impulse);
    }
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
    ContactImpulseCache cache;
    cache.contactCount = 1;
    cache.contacts[0] = CachedContactImpulse{
        EffectiveContact(manifold, 0).featureId,
        accumulatedImpulse
    };
    Resolve(manifold, cache, SimulationConfig{}, reduceWarmStart);
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
    if (!manifold.hasCollision || !manifold.A || !manifold.B) {
        cache = ContactImpulseCache{};
        return;
    }
    SynchronizeCache(manifold, cache);
    RigidBody* bodyA = manifold.A;
    RigidBody* bodyB = manifold.B;
    const float totalInverseMass = bodyA->GetInverseMass()
        + bodyB->GetInverseMass();
    if (totalInverseMass == 0.0f) {
        return;
    }

    const float correctionMagnitude = std::max(
        manifold.penetration - config.penetrationSlop,
        0.0f
    ) / totalInverseMass * config.positionCorrectionFactor;
    const Vector2 correction = manifold.normal * correctionMagnitude;
    if (!bodyA->IsStatic()) {
        bodyA->SetPosition(
            bodyA->GetPosition() - correction * bodyA->GetInverseMass()
        );
    }
    if (!bodyB->IsStatic()) {
        bodyB->SetPosition(
            bodyB->GetPosition() + correction * bodyB->GetInverseMass()
        );
    }

    for (std::uint8_t i = 0; i < cache.contactCount; ++i) {
        ResolveContactVelocity(
            manifold,
            EffectiveContact(manifold, i),
            cache.contacts[i].impulse,
            reduceWarmStart
        );
    }
}

} // namespace PhysicsEngine
