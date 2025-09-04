#ifndef COLLISION_RESOLVER_H
#define COLLISION_RESOLVER_H

#include "collision_manifold.h"

namespace PhysicsEngine {
    class CollisionResolver {
    public:
        static void Resolve(const CollisionManifold& manifold);
    };
}

#endif // COLLISION_RESOLVER_H