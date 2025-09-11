#include "physics/core/collisions/collision_resolver.h"
#include <algorithm> // Required for std::max

namespace PhysicsEngine {
    void CollisionResolver::Resolve(const CollisionManifold& manifold) {
        RigidBody* bodyA = manifold.A;
        RigidBody* bodyB = manifold.B;


        const float percent = 0.4f; // How much of the penetration to correct.
        const float slop = 0.01f;   // A small allowance for penetration to prevent jittering at rest.

        // Calculate the correction vector based on penetration depth and inverse masses.
        Vector2 correction = std::max(manifold.penetration - slop, 0.0f) / 
                             (bodyA->GetInverseMass() + bodyB->GetInverseMass()) * 
                             percent * manifold.normal;

        // Apply the positional correction to the dynamic bodies.
        if (!bodyA->IsStatic()) {
            bodyA->SetPosition(bodyA->GetPosition() - correction * bodyA->GetInverseMass());
        }
        if (!bodyB->IsStatic()) {
            bodyB->SetPosition(bodyB->GetPosition() + correction * bodyB->GetInverseMass());
        }

        // Calculates and applies the change in velocity (the "bounce").

        Vector2 relativeVelocity = bodyB->GetVelocity() - bodyA->GetVelocity();
        float relativeVelocityAlongNormal = relativeVelocity.dot(manifold.normal);

        // Do nothing if velocities are already separating.
        if (relativeVelocityAlongNormal > 0) {
            return;
        }

        // Coefficient of restitution (bounciness). A value less than 1 allows objects to come to rest.
        float restitution = 0.5f;

        // Calculate the impulse magnitude (scalar).
        float impulseScalar = -(1 + restitution) * relativeVelocityAlongNormal;
        impulseScalar /= bodyA->GetInverseMass() + bodyB->GetInverseMass();

        // Apply the impulse as a change in velocity.
        Vector2 impulse = manifold.normal * impulseScalar;
        if (!bodyA->IsStatic()) {
            bodyA->SetVelocity(bodyA->GetVelocity() - impulse * bodyA->GetInverseMass());
        }
        if (!bodyB->IsStatic()) {
            bodyB->SetVelocity(bodyB->GetVelocity() + impulse * bodyB->GetInverseMass());
        }
    }
}