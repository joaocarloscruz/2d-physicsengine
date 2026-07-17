#ifndef COLLISION_MANIFOLD_H
#define COLLISION_MANIFOLD_H

#include "../rigidbody.h"
#include "../../math/vector2.h"
#include <array>
#include <cstdint>

namespace PhysicsEngine {
    struct ContactPoint {
        Vector2 position;
        float penetration = 0.0f;
        std::uint32_t featureId = 0;
    };

    struct CollisionManifold {
        RigidBody* A = nullptr;
        RigidBody* B = nullptr;
        bool hasCollision = false;
        Vector2 normal;
        float penetration = 0.0f;
        Vector2 contactPoint;
        std::array<ContactPoint, 2> contacts{};
        std::uint8_t contactCount = 0;
    };
}

#endif // COLLISION_MANIFOLD_H
