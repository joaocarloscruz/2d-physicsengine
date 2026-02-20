#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/rigidbody.h"
#include <algorithm> // Required for std::max

namespace PhysicsEngine {
    void CollisionResolver::Resolve(const CollisionManifold& manifold) {
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

        // Do nothing if velocities are already separating.
        if (relativeVelocityAlongNormal > 0) {
            return;
        }

        float restitution = std::max(bodyA->material.restitution, bodyB->material.restitution);

        // Calculate the impulse magnitude (scalar).
        float raCrossN = ra.cross(manifold.normal);
        float rbCrossN = rb.cross(manifold.normal);

        float denom = totalInverseMass + 
                      (raCrossN * raCrossN) * bodyA->GetInverseInertia() + 
                      (rbCrossN * rbCrossN) * bodyB->GetInverseInertia();

        float impulseScalar = -(1 + restitution) * relativeVelocityAlongNormal;
        impulseScalar /= denom;

        // Apply the impulse.
        Vector2 impulse = manifold.normal * impulseScalar;
        bodyA->ApplyImpulse(impulse * -1.0f, ra);
        bodyB->ApplyImpulse(impulse, rb);

        // --- Friction ---
        // Re-calculate relative velocity after normal impulse
        relativeVelocity = bodyB->GetVelocityAtPoint(manifold.contactPoint) - bodyA->GetVelocityAtPoint(manifold.contactPoint);
        
        Vector2 tangent = relativeVelocity - manifold.normal * relativeVelocity.dot(manifold.normal);
        
        if (tangent.magnitudeSquared() > 0.0001f) {
            tangent = tangent.normalized();
            
            float jt = -relativeVelocity.dot(tangent);
            
            float raCrossT = ra.cross(tangent);
            float rbCrossT = rb.cross(tangent);
            
            float denomT = totalInverseMass +
                           (raCrossT * raCrossT) * bodyA->GetInverseInertia() +
                           (rbCrossT * rbCrossT) * bodyB->GetInverseInertia();
            
            jt /= denomT;
            
            // Coulomb's law combine metric (using Pythagoras mean or SQRT)
            float muStatic = std::sqrt(bodyA->material.staticFriction * bodyB->material.staticFriction);
            float muDynamic = std::sqrt(bodyA->material.dynamicFriction * bodyB->material.dynamicFriction);
            
            Vector2 frictionImpulse;
            if (std::abs(jt) <= impulseScalar * muStatic) {
                // Static friction: apply the exact impulse needed to stop sliding
                frictionImpulse = tangent * jt;
            } else {
                // Dynamic friction: apply bounded impulse in direction of jt
                float dynamicImpulse = impulseScalar * muDynamic;
                frictionImpulse = tangent * (jt > 0 ? dynamicImpulse : -dynamicImpulse);
            }
            
            bodyA->ApplyImpulse(frictionImpulse * -1.0f, ra);
            bodyB->ApplyImpulse(frictionImpulse, rb);
        }
    }
}