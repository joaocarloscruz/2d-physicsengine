#ifndef AABB_H
#define AABB_H

#include "../../../math/vector2.h"

namespace PhysicsEngine {
    struct AABB {
        Vector2 min;
        Vector2 max;

        bool IsOverlapping(const AABB& other) const {
            return (max.x > other.min.x && min.x < other.max.x && max.y > other.min.y && min.y < other.max.y);
        }
    };
}

#endif // AABB_H