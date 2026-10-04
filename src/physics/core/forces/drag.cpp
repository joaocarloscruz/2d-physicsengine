#include "physics/core/forces/drag.h"
#include "physics/math/vector2.h"
#include "force_arithmetic.h"
#include <cmath>
#include <stdexcept>

namespace PhysicsEngine {

    Drag::Drag(float k1, float k2) : k1(k1), k2(k2) {
        if (!std::isfinite(k1) || !std::isfinite(k2) || k1 < 0.0f || k2 < 0.0f) {
            throw std::invalid_argument("Drag coefficients must be finite and non-negative.");
        }
    }

    void Drag::applyForce(RigidBody* body) {
        if (!body) throw std::invalid_argument("Drag requires a body.");
        const Vector2 velocity = body->GetVelocity();
        if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y))
            throw std::invalid_argument("Drag requires finite velocity.");
        const double speed = std::hypot(static_cast<double>(velocity.x), velocity.y);
        const double gain = k1 + static_cast<double>(k2) * speed;
        body->ApplyForce(ForceArithmetic::CheckedForce(-gain * velocity.x, -gain * velocity.y));
    }

}
