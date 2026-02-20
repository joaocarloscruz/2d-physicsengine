#ifndef COLLISION_LISTENER_H
#define COLLISION_LISTENER_H

#include "collision_manifold.h"

namespace PhysicsEngine {

    // Interface for receiving collision events.
    // Subclass this and register with World::addCollisionListener().
    // onCollision() is called once per collision pair per step(), after resolution.
    class ICollisionListener {
    public:
        virtual ~ICollisionListener() = default;

        virtual void onCollision(const CollisionManifold& manifold) = 0;
    };

}

#endif // COLLISION_LISTENER_H
