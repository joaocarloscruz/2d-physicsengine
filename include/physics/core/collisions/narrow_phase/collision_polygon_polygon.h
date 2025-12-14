#ifndef COLLISION_POLYGON_POLYGON_H
#define COLLISION_POLYGON_POLYGON_H

#include "../collision_manifold.h"
#include "../../rigidbody.h"

namespace PhysicsEngine {
    CollisionManifold CollisionPolygonPolygon(RigidBody* a, RigidBody* b);
}

#endif // COLLISION_POLYGON_POLYGON_H