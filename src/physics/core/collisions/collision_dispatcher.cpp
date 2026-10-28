#include "physics/core/collisions/collision_dispatcher.h"
#include "physics/core/collisions/narrow_phase/collision_circle_circle.h"
#include "physics/core/collisions/narrow_phase/collision_circle_polygon.h"
#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"
#include "physics/core/collisions/narrow_phase/collision_capsule.h"
#include "physics/core/shape.h"
#include "physics/core/rigidbody.h"
#include <vector>
#include <algorithm>

namespace PhysicsEngine {

    using CollisionFunc = CollisionManifold (*)(RigidBody*, RigidBody*, float);

    constexpr int SHAPE_COUNT = static_cast<int>(ShapeType::COUNT);

    static const CollisionFunc collisionLookup[SHAPE_COUNT][SHAPE_COUNT] = {
        // CIRCLE
        {
            CollisionCircleCircle,    // Circle vs Circle
            CollisionCirclePolygon,   // Circle vs Polygon
            CollisionCircleCapsule    // Circle vs Capsule
        },
        // POLYGON
        {
            nullptr,                  // Polygon vs Circle (Handled by swap)
            CollisionPolygonPolygon,  // Polygon vs Polygon
            CollisionPolygonCapsule   // Polygon vs Capsule
        },
        // CAPSULE
        {
            nullptr,                  // Capsule vs Circle (Handled by swap)
            nullptr,                  // Capsule vs Polygon (Handled by swap)
            CollisionCapsuleCapsule   // Capsule vs Capsule
        }
    };

    CollisionManifold CheckCollision(RigidBody* a, RigidBody* b) {
        return CheckCollision(a, b, 0.0f);
    }

    CollisionManifold CheckCollision(RigidBody* a, RigidBody* b, float margin) {
        if (a->IsStatic() && b->IsStatic()) {
            return {};
        }

        const bool swapped = a->shape->type > b->shape->type;
        if (swapped) {
            std::swap(a, b);
        }

        int typeA = static_cast<int>(a->shape->type);
        int typeB = static_cast<int>(b->shape->type);

        // Custom Shape implementations may carry an unsupported signed tag.
        // Bound both indices before using them to access the dispatch table.
        if (typeA < 0 || typeA >= SHAPE_COUNT || typeB < 0 || typeB >= SHAPE_COUNT) {
            return {};
        }

        CollisionFunc func = collisionLookup[typeA][typeB];
        if (func) {
            CollisionManifold manifold = func(a, b, margin);
            if (swapped) {
                std::swap(manifold.A, manifold.B);
                manifold.normal = manifold.normal * -1.0f;
            }
            return manifold;
        }

        return {};
    }
}
