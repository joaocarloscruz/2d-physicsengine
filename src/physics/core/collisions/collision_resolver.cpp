#include "physics/core/collisions/collision_resolver.h"

namespace PhysicsEngine {
    void CollisionResolver::Resolve(const CollisionManifold& manifold) {
        RigidBody* bodyA = manifold.A;
        RigidBody* bodyB = manifold.B;

        // --- Impulse Resolution ---
        Vector2 relativeVelocity = bodyB->GetVelocity() - bodyA->GetVelocity();
        float relativeVelocityAlongNormal = relativeVelocity.dot(manifold.normal);

        // Do not resolve if velocities are separating
        if (relativeVelocityAlongNormal > 0) {
            return;
        }

        // Coefficient of restitution (bounciness)
        float restitution = 1.0f; // A value between 0 and 1

        // Calculate impulse scalar
        float impulseScalar = -(1 + restitution) * relativeVelocityAlongNormal;
        impulseScalar /= bodyA->GetInverseMass() + bodyB->GetInverseMass();

        // Apply impulse
        Vector2 impulse = manifold.normal * impulseScalar;
        if (!bodyA->IsStatic()) {
            bodyA->SetVelocity(PhysicsEngine::Vector2(-10.0f, 0.0f));
            //bodyA->SetVelocity(bodyA->GetVelocity() - impulse * bodyA->GetInverseMass());
        }
        if (!bodyB->IsStatic()) {
            bodyB->SetVelocity(PhysicsEngine::Vector2(10.0f, 0.0f));
            //bodyB->SetVelocity(bodyB->GetVelocity() + impulse * bodyB->GetInverseMass());
        }
    }
}
