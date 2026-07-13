#include "physics/core/forces/drag.h"
#include "physics/math/vector2.h"
#include <cmath>
#include <stdexcept>

namespace PhysicsEngine {

    Drag::Drag(float k1, float k2) : k1(k1), k2(k2) {
        if (!std::isfinite(k1) || !std::isfinite(k2) || k1 < 0.0f || k2 < 0.0f) {
            throw std::invalid_argument("Drag coefficients must be finite and non-negative.");
        }
    }

    void Drag::applyForce(RigidBody* body) {
        Vector2 force = body->GetVelocity();

        // Calculate the total drag force
        float dragCoeff = force.magnitude();
        dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

        // Apply the force in the opposite direction of velocity
        force = force.normalized();
        body->ApplyForce(force * -dragCoeff);
    }

}
