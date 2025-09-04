#include "physics/core/collisions/collision_resolver.h"

namespace PhysicsEngine {
    void CollisionResolver::Resolve(const CollisionManifold& manifold) {
        RigidBody* bodyA = manifold.A;
        RigidBody* bodyB = manifold.B;

        // --- Positional Correction ---
        const float percent = 0.2f; // Penetration percentage to correct
        const float slop = 0.01f; // Penetration allowance
        Vector2 correction = manifold.normal * (std::max(manifold.penetration - slop, 0.0f) / (bodyA->GetInverseMass() + bodyB->GetInverseMass())) * percent;
        
        if (!bodyA->IsStatic()) {
            bodyA->SetPosition(bodyA->GetPosition() - correction * bodyA->GetInverseMass());
        }
        if (!bodyB->IsStatic()) {
            bodyB->SetPosition(bodyB->GetPosition() + correction * bodyB->GetInverseMass());
        }

        // --- Impulse Resolution ---
        Vector2 relativeVelocity = bodyB->GetVelocity() - bodyA->GetVelocity();
        float relativeVelocityAlongNormal = relativeVelocity.dot(manifold.normal);

        // Do not resolve if velocities are separating
        if (relativeVelocityAlongNormal > 0) {
            return;
        }

        // Coefficient of restitution (bounciness)
        float restitution = 0.8f; // A value between 0 and 1

        // Calculate impulse scalar
        float impulseScalar = -(1 + restitution) * relativeVelocityAlongNormal;
        impulseScalar /= bodyA->GetInverseMass() + bodyB->GetInverseMass();

        // Apply impulse
        Vector2 impulse = manifold.normal * impulseScalar;
        if (!bodyA->IsStatic()) {
            bodyA->SetVelocity(bodyA->GetVelocity() - impulse * bodyA->GetInverseMass());
        }
        if (!bodyB->IsStatic()) {
            bodyB->SetVelocity(bodyB->GetVelocity() + impulse * bodyB->GetInverseMass());
        }
    }
}
