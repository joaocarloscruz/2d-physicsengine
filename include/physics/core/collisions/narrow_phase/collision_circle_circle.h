#ifndef COLLISION_CIRCLE_H
#define COLLISION_CIRCLE_H

#include "../collision_manifold.h"
#include "../../rigidbody.h"

namespace PhysicsEngine {
    // A positive margin also reports separated contacts closer than the
    // margin; their penetration is negative (minus the gap).
    CollisionManifold CollisionCircleCircle(RigidBody* a, RigidBody* b, float margin = 0.0f);
}

#endif // COLLISION_CIRCLE_H