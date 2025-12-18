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

        // 2. Test Polygon Axes (Face Normals)
        for (size_t i = 0; i < polyVertices.size(); ++i) {
            Vector2 p1 = polyVertices[i];
            Vector2 p2 = polyVertices[(i + 1) % polyVertices.size()];
            Vector2 edge = p2 - p1;
            Vector2 axis = Vector2(-edge.y, edge.x).normalized(); // Normal

            // Project Polygon
            float minP, maxP;
            ProjectVertices(polyVertices, axis, minP, maxP);

            // Project Circle (Center projection +/- radius)
            float projC = circleCenter.dot(axis);
            float minC = projC - radius;
            float maxC = projC + radius;

            if (maxP < minC || maxC < minP) return manifold; // Gap found

            float overlap = std::min(maxP, maxC) - std::max(minP, minC);
            if (overlap < minOverlap) {
                minOverlap = overlap;
                smallestAxis = axis;
            }
        }

        // 3. Test Circle Axis (Closest Vertex to Center)
        Vector2 closestVertex = GetClosestVertex(polyVertices, circleCenter);
        Vector2 axis = (circleCenter - closestVertex);
        
        // Only normalize if distinct, otherwise center is ON vertex
        if (axis.magnitudeSquared() > 0.0001f) {
             axis = axis.normalized();
             
             float minP, maxP;
             ProjectVertices(polyVertices, axis, minP, maxP);
             
             float projC = circleCenter.dot(axis);
             float minC = projC - radius;
             float maxC = projC + radius;

             if (maxP < minC || maxC < minP) return manifold;

             float overlap = std::min(maxP, maxC) - std::max(minP, minC);
             if (overlap < minOverlap) {
                 minOverlap = overlap;
                 smallestAxis = axis;
             }
        }

        // 4. Resolve Collision
        manifold.hasCollision = true;
        manifold.penetration = minOverlap;
        manifold.normal = smallestAxis;

        // Ensure normal points from A to B
        Vector2 direction = b->GetPosition() - a->GetPosition();
        if (direction.dot(manifold.normal) < 0.0f) {
            manifold.normal = manifold.normal * -1.0f;
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