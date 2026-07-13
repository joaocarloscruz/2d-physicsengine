#ifndef COLLISION_RESOLVER_H
#define COLLISION_RESOLVER_H

#include "collision_manifold.h"

namespace PhysicsEngine {
    struct ContactImpulse {
        float normal = 0.0f;
        float tangent = 0.0f;
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
    };
}

#endif // COLLISION_RESOLVER_H
