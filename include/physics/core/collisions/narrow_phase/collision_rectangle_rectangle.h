#ifndef COLLISION_RECTANGLE_H
#define COLLISION_RECTANGLE_H

#include "../collision_manifold.h"
#include "../../rigidbody.h"

namespace PhysicsEngine {
    CollisionManifold CollisionRectangleRectangle(RigidBody* a, RigidBody* b);
}

#endif // COLLISION_RECTANGLE_H
