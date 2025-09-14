#include "physics/core/collisions/narrow_phase/collision_circle_rectangle.h"
#include "physics/core/shape.h"
#include "physics/math/vector2.h"
#include "physics/math/matrix2x2.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace PhysicsEngine
{
    static inline float clampf(float v, float lo, float hi) {
        return std::max(lo, std::min(v, hi));
    }

    CollisionManifold CollisionCircleRectangle(RigidBody *a, RigidBody *b)
    {
        // Identify which is circle and which is rectangle, but keep original A/B
        Shape *shapeA = a->shape;
        Shape *shapeB = b->shape;

        if (!shapeA || !shapeB) return CollisionManifold();

        RigidBody *circleBody = nullptr;
        RigidBody *rectBody   = nullptr;
        Circle    *circle     = nullptr;
        Rectangle *rect       = nullptr;

        if (shapeA->type == ShapeType::CIRCLE && shapeB->type == ShapeType::RECTANGLE) {
            circleBody = a; rectBody = b;
            circle = static_cast<Circle*>(shapeA);
            rect   = static_cast<Rectangle*>(shapeB);
        } else if (shapeA->type == ShapeType::RECTANGLE && shapeB->type == ShapeType::CIRCLE) {
            circleBody = b; rectBody = a;
            circle = static_cast<Circle*>(shapeB);
            rect   = static_cast<Rectangle*>(shapeA);
        } else {
            return CollisionManifold();
        }

        CollisionManifold manifold;
        manifold.A = a;
        manifold.B = b;
        manifold.hasCollision = false;

        // Rectangle half extents
        const float hx = rect->GetWidth()  * 0.5f;
        const float hy = rect->GetHeight() * 0.5f;

        const Vector2 cCenter = circleBody->GetPosition();
        const Vector2 rCenter = rectBody->GetPosition();

        // Transform circle center into rectangle's local space
        Matrix2x2 R = Matrix2x2::rotation(rectBody->GetOrientation());
        Matrix2x2 Rinv = R.inverse();
        Vector2 localC = Rinv * (cCenter - rCenter);

        // Closest point on rectangle in local space
        Vector2 closestLocal(
            clampf(localC.x, -hx, hx),
            clampf(localC.y, -hy, hy)
        );

        Vector2 deltaLocal = localC - closestLocal;
        float distSq = deltaLocal.magnitudeSquared();
        const float r = circle->GetRadius();

        Vector2 normalLocal;
        float penetration = 0.0f;
        Vector2 contactLocal;

        if (distSq > r * r) {
            // No overlap
            return manifold;
        }

        if (distSq > 1e-12f) {
            // Circle center is outside the rectangle hull
            float dist = std::sqrt(distSq);
            normalLocal = deltaLocal / dist;  // from rect -> circle in local space
            penetration = r - dist;
            contactLocal = closestLocal;      // point on rect hull
        } else {
            // Circle center is inside rectangle; push out along nearest face
            float rightDist  =  hx - localC.x;
            float leftDist   =  localC.x + hx;
            float topDist    =  hy - localC.y;
            float bottomDist =  localC.y + hy;

            float minDist = rightDist;
            normalLocal = Vector2(1.0f, 0.0f);
            contactLocal = Vector2(hx, localC.y);

            if (leftDist < minDist) {
                minDist = leftDist;
                normalLocal = Vector2(-1.0f, 0.0f);
                contactLocal = Vector2(-hx, localC.y);
            }
            if (topDist < minDist) {
                minDist = topDist;
                normalLocal = Vector2(0.0f, 1.0f);
                contactLocal = Vector2(localC.x, hy);
            }
            if (bottomDist < minDist) {
                minDist = bottomDist;
                normalLocal = Vector2(0.0f, -1.0f);
                contactLocal = Vector2(localC.x, -hy);
            }

            // Move the circle out by (radius + distance to face)
            penetration = r + minDist;
        }

        // Transform normal and contact back to world space
        Vector2 normalWorld = R * normalLocal;                
        Vector2 contactWorld = rCenter + (R * contactLocal);

        // Fill manifold
        manifold.hasCollision = true;
        manifold.normal = normalWorld;
        manifold.penetration = penetration;
        manifold.contactPoint = contactWorld;

        Vector2 toB = manifold.B->GetPosition() - manifold.A->GetPosition();
        if (toB.dot(manifold.normal) < 0.0f) {
            manifold.normal = manifold.normal * -1.0f;
        }

        return manifold;
    }
}
