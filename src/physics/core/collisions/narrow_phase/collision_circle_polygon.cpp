#include "physics/core/collisions/narrow_phase/collision_circle_polygon.h"
#include "physics/core/shape.h"
#include "physics/math/matrix2x2.h"
#include <limits>
#include <algorithm>
#include <cmath>

namespace PhysicsEngine {

    // Helper: Transform polygon vertices to world space
    static std::vector<Vector2> GetWorldVertices(RigidBody* body) {
        Polygon* poly = static_cast<Polygon*>(body->shape);
        const std::vector<Vector2>& localVerts = poly->getVertices();
        
        std::vector<Vector2> worldVerts;
        worldVerts.reserve(localVerts.size());

        Matrix2x2 rot = Matrix2x2::rotation(body->GetOrientation());
        Vector2 pos = body->GetPosition();

        for (const auto& v : localVerts) {
            worldVerts.push_back(pos + (rot * v));
        }
        return worldVerts;
    }

    // Helper: Project vertices onto an axis
    static void ProjectVertices(const std::vector<Vector2>& vertices, const Vector2& axis, float& min, float& max) {
        min = std::numeric_limits<float>::max();
        max = -std::numeric_limits<float>::max();

        for (const auto& v : vertices) {
            float projection = v.dot(axis);
            if (projection < min) min = projection;
            if (projection > max) max = projection;
        }
    }

    // Helper: Get closest vertex on polygon to a point
    static Vector2 GetClosestVertex(const std::vector<Vector2>& vertices, const Vector2& point) {
        float minDistSq = std::numeric_limits<float>::max();
        Vector2 closest = vertices[0];

        for (const auto& v : vertices) {
            float distSq = (point - v).magnitudeSquared();
            if (distSq < minDistSq) {
                minDistSq = distSq;
                closest = v;
            }
        }
        return closest;
    }

    CollisionManifold CollisionCirclePolygon(RigidBody* a, RigidBody* b) {
        CollisionManifold manifold;
        manifold.A = a;
        manifold.B = b;
        manifold.hasCollision = false;

        // 1. Identify which is Circle and which is Polygon
        RigidBody* circleBody = (a->shape->type == ShapeType::CIRCLE) ? a : b;
        RigidBody* polyBody = (a->shape->type == ShapeType::POLYGON) ? a : b;

        Circle* circleShape = static_cast<Circle*>(circleBody->shape);

        std::vector<Vector2> polyVertices = GetWorldVertices(polyBody);
        Vector2 circleCenter = circleBody->GetPosition();
        float radius = circleShape->GetRadius();

        float minOverlap = std::numeric_limits<float>::max();
        Vector2 smallestAxis;
        Vector2 containmentEscapeDirection;
        bool smallestAxisUsesContainment = false;

        // Returns false when the projections are separated or only touching.
        // For containment, the ordinary intersection width is not the distance
        // needed to separate the shapes, so measure both escape directions.
        auto testAxis = [&](const Vector2& axis) {
            float minP, maxP;
            ProjectVertices(polyVertices, axis, minP, maxP);

            const float projC = circleCenter.dot(axis);
            const float minC = projC - radius;
            const float maxC = projC + radius;

            if (maxP <= minC || maxC <= minP) {
                return false;
            }

            const bool contains =
                (minP <= minC && maxP >= maxC) ||
                (minC <= minP && maxC >= maxP);

            float overlap;
            Vector2 escapeDirection;
            if (contains) {
                const float moveNegative = maxC - minP;
                const float movePositive = maxP - minC;
                if (moveNegative < movePositive) {
                    overlap = moveNegative;
                    escapeDirection = axis * -1.0f;
                } else {
                    overlap = movePositive;
                    escapeDirection = axis;
                }
            } else {
                overlap = std::min(maxP, maxC) - std::max(minP, minC);
            }

            if (overlap < minOverlap) {
                minOverlap = overlap;
                smallestAxis = axis;
                smallestAxisUsesContainment = contains;
                if (contains) {
                    containmentEscapeDirection = escapeDirection;
                }
            }
            return true;
        };

        // 2. Test Polygon Axes (Face Normals)
        for (size_t i = 0; i < polyVertices.size(); ++i) {
            Vector2 p1 = polyVertices[i];
            Vector2 p2 = polyVertices[(i + 1) % polyVertices.size()];
            Vector2 edge = p2 - p1;
            Vector2 axis = Vector2(-edge.y, edge.x).normalized(); // Normal

            if (!testAxis(axis)) return manifold;
        }

        // 3. Test Circle Axis (Closest Vertex to Center)
        Vector2 closestVertex = GetClosestVertex(polyVertices, circleCenter);
        Vector2 axis = (circleCenter - closestVertex);
        
        // Only normalize if distinct, otherwise center is ON vertex
        if (axis.magnitudeSquared() > 0.0001f) {
             axis = axis.normalized();
             
             if (!testAxis(axis)) return manifold;
        }

        // 4. Resolve Collision
        manifold.hasCollision = true;
        manifold.penetration = minOverlap;
        if (smallestAxisUsesContainment) {
            // The solver moves A along -normal and B along +normal.
            manifold.normal = (a == circleBody)
                ? containmentEscapeDirection * -1.0f
                : containmentEscapeDirection;
        } else {
            manifold.normal = smallestAxis;

            // Ensure normal points from A to B
            Vector2 direction = b->GetPosition() - a->GetPosition();
            if (direction.dot(manifold.normal) < 0.0f) {
                manifold.normal = manifold.normal * -1.0f;
            }
        }

        // Contact point is the point on the circle surface along the collision normal (towards the other body)
        Vector2 pointOnCircle;
        if (a == circleBody) {
             // Normal points A -> B (Circle -> Poly)
             pointOnCircle = circleCenter + (manifold.normal * radius);
        } else {
             // Normal points A -> B (Poly -> Circle)
             // We want point on Circle (B) facing A.
             pointOnCircle = circleCenter - (manifold.normal * radius);
        }
        
        manifold.contactPoint = pointOnCircle;

        return manifold;
    }
}
