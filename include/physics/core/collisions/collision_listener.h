#ifndef COLLISION_LISTENER_H
#define COLLISION_LISTENER_H

#include "collision_manifold.h"

namespace PhysicsEngine {

    // Value snapshot: safe to retain after either body leaves the world.
    struct CollisionEvent {
        std::uint64_t bodyAId = 0;
        std::uint64_t bodyBId = 0;
        Vector2 normal;
        float penetration = 0.0f;
        std::array<ContactPoint, 2> contacts{};
        std::uint8_t contactCount = 0;
    };

    // Interface for receiving collision events.
    // Subclass this and register with World::addCollisionListener().
    // onCollision() is called once per collision pair per step(), after resolution.
    class ICollisionListener {
    public:
        virtual ~ICollisionListener() = default;

        virtual void onCollision(const CollisionManifold&) {}
        virtual void onCollisionBegin(const CollisionEvent&) {}
        virtual void onCollisionPersist(const CollisionEvent&) {}
        // End carries the last observed contact geometry.
        virtual void onCollisionEnd(const CollisionEvent&) {}
    };

}

#endif // COLLISION_LISTENER_H
