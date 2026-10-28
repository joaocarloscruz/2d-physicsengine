#ifndef COLLISION_DISPATCHER_H
#define COLLISION_DISPATCHER_H

#include "collision_manifold.h"
#include "../rigidbody.h"

namespace PhysicsEngine {
    CollisionManifold CheckCollision(RigidBody* a, RigidBody* b);
    // Speculative query: also returns pairs separated by less than margin,
    // with negative penetration for separated contact points.
    CollisionManifold CheckCollision(RigidBody* a, RigidBody* b, float margin);
}

#endif // COLLISION_DISPATCHER_H