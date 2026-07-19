#ifndef COLLISION_RESOLVER_H
#define COLLISION_RESOLVER_H

#include "collision_manifold.h"
#include "../simulation_config.h"
#include <array>
#include <cstdint>

namespace PhysicsEngine {
    struct ContactImpulse {
        float normal = 0.0f;
        float tangent = 0.0f;
    };

    struct CachedContactImpulse {
        std::uint32_t featureId = 0;
        ContactImpulse impulse;
    };

    struct ContactImpulseCache {
        std::array<CachedContactImpulse, 2> contacts{};
        std::uint8_t contactCount = 0;
    };

    struct ContactConstraintPoint {
        Vector2 localAnchorA;
        Vector2 localAnchorB;
        float penetration = 0.0f;
        float normalMass = 0.0f;
        float tangentMass = 0.0f;
        float velocityBias = 0.0f;
        std::uint32_t featureId = 0;
        ContactImpulse impulse;
    };

    struct ContactConstraint {
        RigidBody* bodyA = nullptr;
        RigidBody* bodyB = nullptr;
        Vector2 normal;
        Vector2 tangent;
        std::array<ContactConstraintPoint, 2> points{};
        std::uint8_t pointCount = 0;
        float staticFriction = 0.0f;
        float dynamicFriction = 0.0f;
        float positionCorrectionFactor = 0.0f;
        float penetrationSlop = 0.0f;
        float maxPositionCorrection = 0.0f;
        float velocityTolerance = 0.0f;
        ContactImpulseCache* cache = nullptr;
    };

    class CollisionResolver {
    public:
        static void Resolve(const CollisionManifold& manifold);
        static void Resolve(
            const CollisionManifold& manifold,
            ContactImpulse& accumulatedImpulse,
            bool reduceWarmStart = false
        );
        static void WarmStart(const CollisionManifold& manifold, const ContactImpulse& accumulatedImpulse);

    private:
        friend class World;

        static void WarmStart(
            const CollisionManifold& manifold,
            ContactImpulseCache& cache
        );

        static void Resolve(
            const CollisionManifold& manifold,
            ContactImpulseCache& cache,
            const SimulationConfig& config,
            bool reduceWarmStart = false
        );

        static ContactConstraint PrepareConstraint(
            const CollisionManifold& manifold,
            ContactImpulseCache& cache,
            const SimulationConfig& config
        );

        static void WarmStart(ContactConstraint& constraint);
        static void SolveVelocity(ContactConstraint& constraint);
        static bool SolvePosition(ContactConstraint& constraint);
    };
}

#endif // COLLISION_RESOLVER_H
