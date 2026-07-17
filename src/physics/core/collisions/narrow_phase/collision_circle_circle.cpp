#include "physics/core/collisions/narrow_phase/collision_circle_circle.h"
#include "physics/core/shape.h"
#include <cmath>

PhysicsEngine::CollisionManifold PhysicsEngine::CollisionCircleCircle(RigidBody* a, RigidBody* b){
    CollisionManifold manifold;
    manifold.A = a;
    manifold.B = b;

    Circle* circleA = static_cast<Circle*>(a->shape);
    Circle* circleB = static_cast<Circle*>(b->shape);
    
    Vector2 normal = b->position - a->position;
    float distSq = normal.magnitudeSquared();
    float sumRadii = circleA->GetRadius() + circleB->GetRadius();
    
    if (distSq >= sumRadii * sumRadii) {
        manifold.hasCollision = false;
        return manifold;
    }
    
    float distance = sqrt(distSq);
    
    if (distance == 0.0f) {
        manifold.penetration = sumRadii;
        manifold.normal = Vector2(1.0f, 0.0f);
        manifold.contactPoint = a->position;
    } else {
        manifold.penetration = sumRadii - distance;
        manifold.normal = normal * (1.0f / distance);
        // Contact point is on the surface of A, towards B
        manifold.contactPoint = a->position + manifold.normal * circleA->GetRadius();
    }

    manifold.hasCollision = true;
    manifold.contactCount = 1;
    manifold.contacts[0] = ContactPoint{
        manifold.contactPoint,
        manifold.penetration,
        0x20000001u
    };
    return manifold;
}
