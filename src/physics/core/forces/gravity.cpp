#include "physics/core/forces/gravity.h"
#include "force_arithmetic.h"
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
        if (!body) throw std::invalid_argument("Gravity requires a body.");
        if (body->GetInverseMass() == 0) {
            return; // Infinite mass objects are not affected by gravity
        }

        const float mass = body->GetMass();
        if (!std::isfinite(mass) || mass <= 0)
            throw std::invalid_argument("Gravity requires positive finite dynamic mass.");
        body->ApplyForce(ForceArithmetic::CheckedForce(
            static_cast<double>(gravity.x) * mass, static_cast<double>(gravity.y) * mass));
    }

    void Gravity::setGravity(const Vector2& new_gravity) {
        ValidateGravity(new_gravity);
        gravity = new_gravity;
    }

}
