#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"
#include "physics/core/shape.h"
#include "physics/math/matrix2x2.h"
#include <limits>
#include <algorithm>

namespace PhysicsEngine {

    // Helper: Transform local polygon vertices to world space
    // R = Rotation Matrix, T = Translation Vector
    // V_world = R * V_local + T
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

    // Project vertices onto a generic axis and return [min, max] interval
    static void ProjectVertices(const std::vector<Vector2>& vertices, const Vector2& axis, float& min, float& max) {
        min = std::numeric_limits<float>::max();
        max = -std::numeric_limits<float>::max();

        for (const auto& v : vertices) {
            float projection = v.dot(axis);
            if (projection < min) min = projection;
            if (projection > max) max = projection;
        }
    }

    CollisionManifold CollisionPolygonPolygon(RigidBody* a, RigidBody* b) {
        CollisionManifold manifold;
        manifold.A = a;
        manifold.B = b;
        manifold.hasCollision = false;

        std::vector<Vector2> verticesA = GetWorldVertices(a);
        std::vector<Vector2> verticesB = GetWorldVertices(b);

        // Variables to track the Minimum Translation Vector (MTV)
        float minOverlap = std::numeric_limits<float>::max();
        Vector2 smallestAxis;

        // 2. Loop through all edges of A, then all edges of B
        //    Total axes to test = Edges(A) + Edges(B)
        const std::vector<Vector2>* polygons[] = { &verticesA, &verticesB };
        
        for (const auto* poly : polygons) {
            for (size_t i = 0; i < poly->size(); ++i) {
                // Get edge vector: v[i+1] - v[i]
                Vector2 p1 = (*poly)[i];
                Vector2 p2 = (*poly)[(i + 1) % poly->size()]; // Wrap around
                
                Vector2 edge = p2 - p1;
                
                // Get normal (perpendicular) to the edge
                // For CCW winding, normal is (-y, x)
                Vector2 axis = Vector2(-edge.y, edge.x).normalized();

                // 3. Project both shapes onto this axis
                float minA, maxA, minB, maxB;
                ProjectVertices(verticesA, axis, minA, maxA);
                ProjectVertices(verticesB, axis, minB, maxB);

                // 4. Check for Gap (Seperating Axis Theorem)
                if (maxA < minB || maxB < minA) {
                    // Gap found! No collision possible.
                    return manifold; 
                }

                // 5. Calculate Overlap
                float overlap = std::min(maxA, maxB) - std::max(minA, minB);

                // 6. Track the smallest overlap (MTV)
                if (overlap < minOverlap) {
                    minOverlap = overlap;
                    smallestAxis = axis;
                }
            }
        }

        // If we reach here, no gaps were found on ANY axis. Collision is confirmed.
        manifold.hasCollision = true;
        manifold.penetration = minOverlap;
        manifold.normal = smallestAxis;

        // 7. Ensure normal points from A to B
        Vector2 direction = b->GetPosition() - a->GetPosition();
        if (direction.dot(manifold.normal) < 0.0f) {
            manifold.normal = manifold.normal * -1.0f;
        }

        // Note: Contact Point finding is deliberately skipped here
        // as it belongs to the next sub-issue.
        manifold.contactPoint = Vector2(0,0); 

        return manifold;
    }
}