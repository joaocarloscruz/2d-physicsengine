#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/rigidbody.h"
#include <algorithm> // Required for std::max

namespace PhysicsEngine {
    namespace {
        Vector2 ContactTangent(const Vector2& normal) {
            return Vector2(-normal.y, normal.x);
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
    }

    void CollisionResolver::WarmStart(
        const CollisionManifold& manifold,
        const ContactImpulse& accumulatedImpulse
    ) {
        if (!manifold.hasCollision || !manifold.A || !manifold.B) {
            return;
        }

        const Vector2 tangent = ContactTangent(manifold.normal);
        const Vector2 impulse = manifold.normal * accumulatedImpulse.normal
            + tangent * accumulatedImpulse.tangent;
        ApplyContactImpulse(manifold.A, manifold.B, manifold.contactPoint, impulse);
    }

    void CollisionResolver::Resolve(const CollisionManifold& manifold) {
        ContactImpulse accumulatedImpulse;
        Resolve(manifold, accumulatedImpulse);
    }

    void CollisionResolver::Resolve(
        const CollisionManifold& manifold,
        ContactImpulse& accumulatedImpulse,
        bool reduceWarmStart
    ) {
        RigidBody* bodyA = manifold.A;
        RigidBody* bodyB = manifold.B;


        const float percent = 0.8f; // How much of the penetration to correct.
        const float slop = 0.005f;   // A small allowance for penetration to prevent jittering at rest.

        // Calculate the correction vector based on penetration depth and inverse masses.
        float totalInverseMass = bodyA->GetInverseMass() + bodyB->GetInverseMass();
        if (totalInverseMass == 0) return;

        float scalarPart = std::max(manifold.penetration - slop, 0.0f) / totalInverseMass * percent;
        Vector2 correction = manifold.normal * scalarPart;

        // Apply the positional correction to the dynamic bodies.
        if (!bodyA->IsStatic()) {
            bodyA->SetPosition(bodyA->GetPosition() - correction * bodyA->GetInverseMass());
        }
        if (!bodyB->IsStatic()) {
            bodyB->SetPosition(bodyB->GetPosition() + correction * bodyB->GetInverseMass());
        }

        // Calculates and applies the change in velocity (the "bounce").

        Vector2 ra = manifold.contactPoint - bodyA->GetPosition();
        Vector2 rb = manifold.contactPoint - bodyB->GetPosition();

        Vector2 relativeVelocity = bodyB->GetVelocityAtPoint(manifold.contactPoint) - bodyA->GetVelocityAtPoint(manifold.contactPoint);
        float relativeVelocityAlongNormal = relativeVelocity.dot(manifold.normal);

        // A fresh separating contact needs no impulse. A warm-started contact
        // may need a negative delta to reduce its previously accumulated value.
        if (relativeVelocityAlongNormal > 0) {
            if (!reduceWarmStart || accumulatedImpulse.normal <= 0.0f) {
                return;
            }
        }

        const float restitution = relativeVelocityAlongNormal < 0.0f
            ? std::max(bodyA->material.restitution, bodyB->material.restitution)
            : 0.0f;

        // Calculate the impulse magnitude (scalar).
        float raCrossN = ra.cross(manifold.normal);
        float rbCrossN = rb.cross(manifold.normal);

        float denom = totalInverseMass + 
                      (raCrossN * raCrossN) * bodyA->GetInverseInertia() + 
                      (rbCrossN * rbCrossN) * bodyB->GetInverseInertia();

        if (denom <= 0.0f) {
            return;
        }

        float impulseScalar = -(1 + restitution) * relativeVelocityAlongNormal / denom;
        const float previousNormalImpulse = accumulatedImpulse.normal;
        accumulatedImpulse.normal = std::max(previousNormalImpulse + impulseScalar, 0.0f);
        impulseScalar = accumulatedImpulse.normal - previousNormalImpulse;

        // Apply the impulse.
        Vector2 impulse = manifold.normal * impulseScalar;
        ApplyContactImpulse(bodyA, bodyB, manifold.contactPoint, impulse);

        // --- Friction ---
        // Re-calculate relative velocity after normal impulse
        relativeVelocity = bodyB->GetVelocityAtPoint(manifold.contactPoint) - bodyA->GetVelocityAtPoint(manifold.contactPoint);
        
        Vector2 tangent = ContactTangent(manifold.normal);
        if (std::abs(relativeVelocity.dot(tangent)) > 0.0001f) {
            float jt = -relativeVelocity.dot(tangent);
            
            float raCrossT = ra.cross(tangent);
            float rbCrossT = rb.cross(tangent);
            
            float denomT = totalInverseMass +
                           (raCrossT * raCrossT) * bodyA->GetInverseInertia() +
                           (rbCrossT * rbCrossT) * bodyB->GetInverseInertia();
            
            if (denomT <= 0.0f) {
                return;
            }

            jt /= denomT;
            
            // Coulomb's law combine metric (using Pythagoras mean or SQRT)
            float muStatic = std::sqrt(bodyA->material.staticFriction * bodyB->material.staticFriction);
            float muDynamic = std::sqrt(bodyA->material.dynamicFriction * bodyB->material.dynamicFriction);
            
            const float previousTangentImpulse = accumulatedImpulse.tangent;
            const float candidateTangentImpulse = previousTangentImpulse + jt;
            const float maxStaticImpulse = accumulatedImpulse.normal * muStatic;

            if (std::abs(candidateTangentImpulse) <= maxStaticImpulse) {
                accumulatedImpulse.tangent = candidateTangentImpulse;
            } else {
                const float maxDynamicImpulse = accumulatedImpulse.normal * muDynamic;
                accumulatedImpulse.tangent = std::clamp(
                    candidateTangentImpulse,
                    -maxDynamicImpulse,
                    maxDynamicImpulse
                );
            }

            const float tangentImpulseDelta = accumulatedImpulse.tangent - previousTangentImpulse;
            ApplyContactImpulse(
                bodyA,
                bodyB,
                manifold.contactPoint,
                tangent * tangentImpulseDelta
            );
        }
    }
}
