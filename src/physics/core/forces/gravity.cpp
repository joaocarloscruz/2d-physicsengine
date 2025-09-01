#include "physics/core/forces/gravity.h"

namespace PhysicsEngine {

    Gravity::Gravity(const Vector2& gravity) : gravity(gravity) {}

    void Gravity::applyForce(RigidBody* body) {
        if (body->GetInverseMass() == 0) {
            return; // Infinite mass objects are not affected by gravity
        }

        body->ApplyForce(gravity * body->GetMass());
    }

    void Gravity::setGravity(const Vector2& new_gravity) {
        gravity = new_gravity;
    }

}