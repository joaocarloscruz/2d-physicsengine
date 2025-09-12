#include "physics/core/collisions/narrow_phase/collision_circle_rectangle.h"
#include "physics/core/shape.h"
#include "physics/math/vector2.h"
#include <cmath>
#include <algorithm>

namespace PhysicsEngine
{
    CollisionManifold CollisionCircleRectangle(RigidBody *a, RigidBody *b)
    {
        Shape *shapeA = a->shape;
        Shape *shapeB = b->shape;

        Shape *circle_shape = nullptr;
        Shape *rectangle_shape = nullptr;
        RigidBody *circle_body = nullptr;
        RigidBody *rectangle_body = nullptr;

        if (shapeA->type == ShapeType::CIRCLE && shapeB->type == ShapeType::RECTANGLE){
            circle_shape = shapeA;
            rectangle_shape = shapeB;
            circle_body = a;
            rectangle_body = b;
        } else if (shapeA->type == ShapeType::RECTANGLE && shapeB->type == ShapeType::CIRCLE) {
            circle_shape = shapeB;
            rectangle_shape = shapeA;
            circle_body = b;
            rectangle_body = a;
        } else {     
            return CollisionManifold();
        }

        Vector2 circleCenter = circle_body->position;
        float circleRadius = circle_shape->GetRadius();

        Vector2 rectCenter = rectangle_body->position;
        float rectWidth = rectangle_shape->GetWidth();
        float rectHeight = rectangle_shape->GetHeight();

        Vector2 closestPoint = circleCenter;
        closestPoint.x = std::max(rectCenter.x - rectWidth / 2.0f, std::min(closestPoint.x, rectCenter.x + rectWidth / 2.0f));
        closestPoint.y = std::max(rectCenter.y - rectHeight / 2.0f, std::min(closestPoint.y, rectCenter.y + rectHeight / 2.0f));

        Vector2 distance = circleCenter - closestPoint;
        float distanceSquared = distance.magnitudeSquared();

        if (distanceSquared < (circleRadius * circleRadius)){
            float dist = std::sqrt(distanceSquared);
            Vector2 normal = distance / dist;
            float penetration = circleRadius - dist;
            
            CollisionManifold manifold;
            manifold.A = a;
            manifold.B = b;
            manifold.normal = normal;
            manifold.penetration = penetration;
            manifold.hasCollision = true;
            return manifold;
        }

        return CollisionManifold();
    }
}