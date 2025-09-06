#ifndef COLLISION_MANIFOLD_H
#define COLLISION_MANIFOLD_H

#include "../rigidbody.h"
#include "../../math/vector2.h"

namespace PhysicsEngine {
    struct CollisionManifold {
        RigidBody* A;
        RigidBody* B;
        bool hasCollision = false;
        Vector2 normal;
        float penetration;
        Vector2 contactPoint;
    };
}

#endif // COLLISION_MANIFOLD_H