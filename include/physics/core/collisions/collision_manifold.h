#ifndef COLLISION_MANIFOLD_H
#define COLLISION_MANIFOLD_H

#include "../rigidbody.h"
#include "../../math/Vector2.h"

struct CollisionManifold {
    RigidBody* A;
    RigidBody* B;
    bool hasCollision = false;
    Vector2 normal;
    float penetration;
};

#endif // COLLISION_MANIFOLD_H