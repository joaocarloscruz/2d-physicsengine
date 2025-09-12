#ifndef COLLISION_CIRCLE_RECTANGLE_H
#define COLLISION_CIRCLE_RECTANGLE_H

#include "../collision_manifold.h"
#include "../../rigidbody.h"

namespace PhysicsEngine
{
    CollisionManifold CollisionCircleRectangle(RigidBody *a, RigidBody *b);
}

#endif // COLLISION_CIRCLE_RECTANGLE_H
