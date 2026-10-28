#ifndef COLLISION_CAPSULE_H
#define COLLISION_CAPSULE_H

#include "../collision_manifold.h"
#include "../../rigidbody.h"

namespace PhysicsEngine {
    // Capsule manifolds accept their two bodies in either order; the normal
    // always points from A toward B. Contact positions are midpoints between
    // the two surfaces, so swapping bodies only negates the normal. Exact
    // touching is not overlap. A positive margin also reports separated
    // features closer than the margin, with negative penetration (the gap).
    CollisionManifold CollisionCircleCapsule(RigidBody* a, RigidBody* b, float margin = 0.0f);
    CollisionManifold CollisionPolygonCapsule(RigidBody* a, RigidBody* b, float margin = 0.0f);
    CollisionManifold CollisionCapsuleCapsule(RigidBody* a, RigidBody* b, float margin = 0.0f);
}

#endif
