#ifndef COLLISION_DISPATCHER_H
#define COLLISION_DISPATCHER_H

#include "collision_manifold.h"
#include "../rigidbody.h"

namespace PhysicsEngine {
    CollisionManifold CheckCollision(RigidBody* a, RigidBody* b);
}

#endif // COLLISION_DISPATCHER_H