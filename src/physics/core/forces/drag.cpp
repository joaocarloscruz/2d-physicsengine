#include "physics/core/forces/drag.h"
#include "physics/math/vector2.h"

namespace PhysicsEngine {

    Drag::Drag(float k1, float k2) : k1(k1), k2(k2) {}

    void Drag::applyForce(RigidBody* body) {
        Vector2 force = body->GetVelocity();

        // Calculate the total drag force
        float dragCoeff = force.magnitude();
        dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

        // Apply the force in the opposite direction of velocity
        force.normalize();
        body->ApplyForce(force * -dragCoeff);
    }

}
