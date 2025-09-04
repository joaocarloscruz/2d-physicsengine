#ifndef COLLISION_CIRCLE_H
#define COLLISION_CIRCLE_H

#include "../collision_manifold.h"
#include "../../rigidbody.h"

namespace PhysicsEngine {
    CollisionManifold CollisionCircleCircle(RigidBody* a, RigidBody* b);
}

#endif // COLLISION_CIRCLE_H