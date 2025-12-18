#ifndef COLLISION_CIRCLE_POLYGON_H
#define COLLISION_CIRCLE_POLYGON_H

#include "../collision_manifold.h"
#include "../../rigidbody.h"

namespace PhysicsEngine {
    CollisionManifold CollisionCirclePolygon(RigidBody* a, RigidBody* b);
}

#endif