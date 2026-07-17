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
    };
}

#endif // COLLISION_RESOLVER_H
