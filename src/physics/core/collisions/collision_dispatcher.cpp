#include "physics/core/collisions/collisions_dispatcher.h"
#include "physics/core/collisions/narrow_phase/collision_circle_circle.h"
#include "physics/core/shape.h"

#include <utility>

PhysicsEngine::CollisionManifold PhysicsEngine::CheckCollision(RigidBody* a, RigidBody* b) {
    if (a->IsStatic() && b->IsStatic()) {
        CollisionManifold manifold;
        manifold.hasCollision = false;
        return manifold;
    }
    
    if (a->shape->type > b->shape->type) {
        std::swap(a, b);
    }

    if (a->shape->type == ShapeType::CIRCLE && b->shape->type == ShapeType::CIRCLE) {
        return CollisionCircleCircle(a, b);
    }
    if (a->shape->type == ShapeType::RECTANGLE && b->shape->type == ShapeType::RECTANGLE) {
        // TODO: Call rectangle-rectangle collision function
    }
    if (a->shape->type == ShapeType::CIRCLE && b->shape->type == ShapeType::RECTANGLE) {
        // TODO: Call circle-rectangle collision function
    }
    if(a->shape->type == ShapeType::CIRCLE && b->shape->type == ShapeType::TRIANGLE){
        // TODO: Call circle-triangle collision function
    }
    if(a->shape->type == ShapeType::RECTANGLE && b->shape->type == ShapeType::TRIANGLE){
        // TODO: Call rectangle-triangle collision function
    }
    
    CollisionManifold noCollision;
    noCollision.hasCollision = false;
    return noCollision;
}