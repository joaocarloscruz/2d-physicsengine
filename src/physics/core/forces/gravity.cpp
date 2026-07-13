#include "physics/core/forces/gravity.h"
#include <cmath>
#include <stdexcept>

namespace PhysicsEngine {

    namespace {
        void ValidateGravity(const Vector2& gravity) {
            if (!std::isfinite(gravity.x) || !std::isfinite(gravity.y)) {
                throw std::invalid_argument("Gravity must be finite.");
            }
        }
    }

    Gravity::Gravity(const Vector2& gravity) : gravity(gravity) {
        ValidateGravity(gravity);
    }

    void Gravity::applyForce(RigidBody* body) {
        if (body->GetInverseMass() == 0) {
            return; // Infinite mass objects are not affected by gravity
        }

        body->ApplyForce(gravity * body->GetMass());
    }

    void Gravity::setGravity(const Vector2& new_gravity) {
        ValidateGravity(new_gravity);
        gravity = new_gravity;
    }

}
