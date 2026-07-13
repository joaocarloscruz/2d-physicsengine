#include "physics/core/collisions/collision_dispatcher.h"
#include "physics/core/collisions/narrow_phase/collision_circle_circle.h"
#include "physics/core/collisions/narrow_phase/collision_circle_polygon.h"
#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"
#include "physics/core/shape.h"
#include "physics/core/rigidbody.h"
#include <vector>
#include <algorithm>

namespace PhysicsEngine {

    using CollisionFunc = CollisionManifold (*)(RigidBody*, RigidBody*);

    constexpr int SHAPE_COUNT = static_cast<int>(ShapeType::COUNT);

    static const CollisionFunc collisionLookup[SHAPE_COUNT][SHAPE_COUNT] = {
        // CIRCLE
        {
            CollisionCircleCircle,    // Circle vs Circle
            CollisionCirclePolygon    // Circle vs Polygon
        },
        // POLYGON
        {
            nullptr,                  // Polygon vs Circle (Handled by swap)
            CollisionPolygonPolygon   // Polygon vs Polygon
        }
    };

    CollisionManifold CheckCollision(RigidBody* a, RigidBody* b) {
        if (a->IsStatic() && b->IsStatic()) {
            return {};
        }

        const bool swapped = a->shape->type > b->shape->type;
        if (swapped) {
            std::swap(a, b);
        }

        int typeA = static_cast<int>(a->shape->type);
        int typeB = static_cast<int>(b->shape->type);

        if (typeA >= SHAPE_COUNT || typeB >= SHAPE_COUNT) {
            return {};
        }

        CollisionFunc func = collisionLookup[typeA][typeB];
        if (func) {
            CollisionManifold manifold = func(a, b);
            if (swapped) {
                std::swap(manifold.A, manifold.B);
                manifold.normal = manifold.normal * -1.0f;
            }
            return manifold;
        }

        return {};
    }
}
