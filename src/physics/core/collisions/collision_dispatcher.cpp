#include "physics/core/collisions/collisions_dispatcher.h"
#include "physics/core/collisions/narrow_phase/collision_circle_circle.h"
#include "physics/core/collisions/narrow_phase/collision_rectangle_rectangle.h"
#include "physics/core/collisions/narrow_phase/collision_circle_rectangle.h"
#include "physics/core/shape.h"
#include "physics/core/rigidbody.h"
#include "physics/core/collisions/collision_manifold.h"

#include <utility>

namespace PhysicsEngine {

    // TODO: Implement Triangle-Circle collision in collision_triangle_circle.cpp
    CollisionManifold CollisionCircleTriangle(RigidBody* a, RigidBody* b) {
        CollisionManifold manifold;
        manifold.hasCollision = false;
        return manifold;
    }

    // TODO: Implement Triangle-Rectangle collision in collision_triangle_rectangle.cpp
    CollisionManifold CollisionRectangleTriangle(RigidBody* a, RigidBody* b) {
        CollisionManifold manifold;
        manifold.hasCollision = false;
        return manifold;
    }

    // TODO: Implement Triangle-Triangle collision in collision_triangle_triangle.cpp
    CollisionManifold CollisionTriangleTriangle(RigidBody* a, RigidBody* b) {
        CollisionManifold manifold;
        manifold.hasCollision = false;
        return manifold;
    }

    using CollisionFunc = CollisionManifold (*)(RigidBody*, RigidBody*);

    constexpr int SHAPE_COUNT = static_cast<int>(ShapeType::COUNT);

    static const CollisionFunc collisionLookup[SHAPE_COUNT][SHAPE_COUNT] = {
        // CIRCLE (Row 0)
        {
            CollisionCircleCircle,    // Col 0: Circle vs Circle
            CollisionCircleRectangle, // Col 1: Circle vs Rectangle
            CollisionCircleTriangle   // Col 2: Circle vs Triangle
        },
        // RECTANGLE (Row 1)
        {
            nullptr,                  // Col 0: Rectangle vs Circle (swap)
            CollisionRectangleRectangle,
            CollisionRectangleTriangle
        },
        // TRIANGLE (Row 2)
        {
            nullptr,                  // Col 0: Triangle vs Circle (swap)
            nullptr,                  // Col 1: Triangle vs Rectangle (swap)
            CollisionTriangleTriangle
        }
    };

    CollisionManifold CheckCollision(RigidBody* a, RigidBody* b) {
        if (a->IsStatic() && b->IsStatic()) {
            return {}; // Return empty manifold
        }

        // Ensure A is always the "smaller" enum type (e.g., Circle < Rectangle)
        if (a->shape->type > b->shape->type) {
            std::swap(a, b);
        }

        int typeA = static_cast<int>(a->shape->type);
        int typeB = static_cast<int>(b->shape->type);

        // Safety check for array bounds
        if (typeA >= SHAPE_COUNT || typeB >= SHAPE_COUNT) {
            return {}; 
        }

        CollisionFunc func = collisionLookup[typeA][typeB];
        if (func) {
            return func(a, b);
        }

        return {};
    }
}