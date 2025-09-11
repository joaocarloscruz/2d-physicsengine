
#include "physics/core/collisions/narrow_phase/collision_rectangle_rectangle.h"
#include "physics/core/shape.h"
#include "physics/math/matrix2x2.h"
#include <cmath>
#include <limits>
#include <iostream>


#include <fstream>

namespace PhysicsEngine {

    // Helper function to project vertices onto an axis and return the min and max projection
    void projectVertices(const Vector2 vertices[4], const Vector2& axis, float& min, float& max) {
        min = std::numeric_limits<float>::max();
        max = -std::numeric_limits<float>::max();

        for (int i = 0; i < 4; ++i) {
            float projection = vertices[i].dot(axis);
            if (projection < min) {
                min = projection;
            }
            if (projection > max) {
                max = projection;
            }
        }
    }

    CollisionManifold CollisionRectangleRectangle(RigidBody* a, RigidBody* b) {

        CollisionManifold manifold;
        // set velocity to -1 and 1
        manifold.A = a;
        manifold.B = b;
        manifold.hasCollision = false;

        Rectangle* rectA = static_cast<Rectangle*>(a->shape);
        Rectangle* rectB = static_cast<Rectangle*>(b->shape);

        // Get vertices of both rectangles in world coordinates
        Vector2 verticesA[4];
        Vector2 verticesB[4];

        float halfWidthA = rectA->GetWidth() / 2.0f;
        float halfHeightA = rectA->GetHeight() / 2.0f;
        Matrix2x2 rotA = Matrix2x2::rotation(a->orientation);
        verticesA[0] = a->position + rotA * Vector2(-halfWidthA, -halfHeightA);
        verticesA[1] = a->position + rotA * Vector2( halfWidthA, -halfHeightA);
        verticesA[2] = a->position + rotA * Vector2( halfWidthA,  halfHeightA);
        verticesA[3] = a->position + rotA * Vector2(-halfWidthA,  halfHeightA);

        float halfWidthB = rectB->GetWidth() / 2.0f;
        float halfHeightB = rectB->GetHeight() / 2.0f;
        Matrix2x2 rotB = Matrix2x2::rotation(b->orientation);
        verticesB[0] = b->position + rotB * Vector2(-halfWidthB, -halfHeightB);
        verticesB[1] = b->position + rotB * Vector2( halfWidthB, -halfHeightB);
        verticesB[2] = b->position + rotB * Vector2( halfWidthB,  halfHeightB);
        verticesB[3] = b->position + rotB * Vector2(-halfWidthB,  halfHeightB);

        // Get the axes to test (normals of the edges)
        Vector2 axes[4];
        Vector2 edgeA = verticesA[1] - verticesA[0];
        axes[0] = Vector2(-edgeA.y, edgeA.x).normalized();
        edgeA = verticesA[2] - verticesA[1];
        axes[1] = Vector2(-edgeA.y, edgeA.x).normalized();

        Vector2 edgeB = verticesB[1] - verticesB[0];
        axes[2] = Vector2(-edgeB.y, edgeB.x).normalized();
        edgeB = verticesB[2] - verticesB[1];
        axes[3] = Vector2(-edgeB.y, edgeB.x).normalized();

        float minOverlap = std::numeric_limits<float>::max();
        Vector2 smallestAxis;

        for (int i = 0; i < 4; ++i) {
            Vector2 axis = axes[i];

            float minA, maxA, minB, maxB;
            projectVertices(verticesA, axis, minA, maxA);
            projectVertices(verticesB, axis, minB, maxB);

            // Check for non-overlap
            if (maxA < minB || maxB < minA) {
                return manifold; // Found a separating axis, no collision
            }

            // Calculate overlap
            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < minOverlap) {
                minOverlap = overlap;
                smallestAxis = axis;
            }
        }

        // If we are here, there is a collision
        manifold.hasCollision = true;
        manifold.penetration = minOverlap;
        manifold.normal = smallestAxis;

        // Ensure the normal is pointing from A to B
        Vector2 toB = b->position - a->position;
        if (toB.dot(manifold.normal) < 0) {
            manifold.normal = manifold.normal * -1.0f;
        }

        // Find the contact point
        float maxPenetration = -std::numeric_limits<float>::max();
        Vector2 contactPoint;

        for (int i = 0; i < 4; ++i) {
            float penetration = (verticesB[i] - a->position).dot(manifold.normal);
            if (penetration > maxPenetration) {
                maxPenetration = penetration;
                contactPoint = verticesB[i];
            }
        }
        manifold.contactPoint = contactPoint;

        return manifold;
    }
}
