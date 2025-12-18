#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"
#include "physics/core/shape.h"
#include "physics/math/matrix2x2.h"
#include <limits>
#include <algorithm>
#include <cmath>

namespace PhysicsEngine {

    // --- Helpers from before ---

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

    static void ProjectVertices(const std::vector<Vector2>& vertices, const Vector2& axis, float& min, float& max) {
        min = std::numeric_limits<float>::max();
        max = -std::numeric_limits<float>::max();

        for (const auto& v : vertices) {
            float projection = v.dot(axis);
            if (projection < min) min = projection;
            if (projection > max) max = projection;
        }
    }

    // Find the vertex on 'poly' that is furthest along 'dir'
    static Vector2 GetSupportPoint(const std::vector<Vector2>& vertices, Vector2 dir) {
        float maxDist = -std::numeric_limits<float>::max();
        Vector2 bestVertex = vertices[0];

        for (const auto& v : vertices) {
            float dist = v.dot(dir);
            if (dist > maxDist) {
                maxDist = dist;
                bestVertex = v;
            }
        }
        return bestVertex;
    }

    // Determine which vertex is colliding
    static Vector2 FindContactPoint(const std::vector<Vector2>& vertsA, const std::vector<Vector2>& vertsB, Vector2 normal) {
        // 1. Determine which polygon is the "Reference" (provides the face) and which is the "Incident"
        // 2. The Reference polygon is the one whose edge normal is most parallel to the collision normal

        float bestDotA = -std::numeric_limits<float>::max();
        
        // Check alignment of A's edges with the normal
        for (size_t i = 0; i < vertsA.size(); ++i) {
            Vector2 p1 = vertsA[i];
            Vector2 p2 = vertsA[(i + 1) % vertsA.size()];
            Vector2 edge = p2 - p1;
            Vector2 edgeNormal = Vector2(-edge.y, edge.x).normalized(); // CCW
            
            // get the edge that is pointing in the direction of the normal
            float dot = edgeNormal.dot(normal);
            if (dot > bestDotA) {
                bestDotA = dot;
            }
        }

        float bestDotB = -std::numeric_limits<float>::max();
        // Check alignment of B's edges with the normal 
        // Note: Normal points A->B. So B's normals should point opposite to collision normal to be the reference face.
        for (size_t i = 0; i < vertsB.size(); ++i) {
            Vector2 p1 = vertsB[i];
            Vector2 p2 = vertsB[(i + 1) % vertsB.size()];
            Vector2 edge = p2 - p1;
            Vector2 edgeNormal = Vector2(-edge.y, edge.x).normalized(); 

            float dot = edgeNormal.dot(normal * -1.0f); // Check against negative normal
            if (dot > bestDotB) {
                bestDotB = dot;
            }
        }

        // If A is the reference face, B provides the incident vertex (deepest point).
        // If B is the reference face, A provides the incident vertex.
        
        if (bestDotA >= bestDotB) {
            // A is Reference, B is Incident.
            // Find vertex on B that is furthest in direction -Normal (towards A)
            return GetSupportPoint(vertsB, normal * -1.0f);
        } else {
            // B is Reference, A is Incident.
            // Find vertex on A that is furthest in direction Normal (towards B)
            return GetSupportPoint(vertsA, normal);
        }
    }

    CollisionManifold CollisionPolygonPolygon(RigidBody* a, RigidBody* b) {
        CollisionManifold manifold;
        manifold.A = a;
        manifold.B = b;
        manifold.hasCollision = false;

        std::vector<Vector2> verticesA = GetWorldVertices(a);
        std::vector<Vector2> verticesB = GetWorldVertices(b);

        float minOverlap = std::numeric_limits<float>::max();
        Vector2 smallestAxis;

        // Loop through edges of A and B
        const std::vector<Vector2>* polygons[] = { &verticesA, &verticesB };
        
        for (const auto* poly : polygons) {
            for (size_t i = 0; i < poly->size(); ++i) {
                Vector2 p1 = (*poly)[i];
                Vector2 p2 = (*poly)[(i + 1) % poly->size()];
                Vector2 edge = p2 - p1;
                Vector2 axis = Vector2(-edge.y, edge.x).normalized();

                float minA, maxA, minB, maxB;
                ProjectVertices(verticesA, axis, minA, maxA);
                ProjectVertices(verticesB, axis, minB, maxB);

                if (maxA < minB || maxB < minA) {
                    return manifold; // No collision
                }

                float overlap = std::min(maxA, maxB) - std::max(minA, minB);
                if (overlap < minOverlap) {
                    minOverlap = overlap;
                    smallestAxis = axis;
                }
            }
        }

        manifold.hasCollision = true;
        manifold.penetration = minOverlap;
        manifold.normal = smallestAxis;

        // Ensure normal points from A to B
        Vector2 direction = b->GetPosition() - a->GetPosition();
        if (direction.dot(manifold.normal) < 0.0f) {
            manifold.normal = manifold.normal * -1.0f;
        }

        manifold.contactPoint = FindContactPoint(verticesA, verticesB, manifold.normal);

        return manifold;
    }
}