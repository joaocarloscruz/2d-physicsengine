#include "physics/core/particles/particle.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {

namespace {
void ValidateVector(const Vector2& value) {
    if (!std::isfinite(value.x) || !std::isfinite(value.y))
        throw std::invalid_argument("Particle state and forces must be finite.");
}
float CheckedFloat(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::overflow_error("Particle result exceeds finite float range.");
    return static_cast<float>(value);
}
Vector2 CheckedVector(double x, double y) { return {CheckedFloat(x), CheckedFloat(y)}; }
}

Particle::Particle(
    const Vector2& initialPosition,
    const Vector2& initialVelocity,
    float particleMass
) : position(initialPosition),
    velocity(initialVelocity),
    force(),
    mass(particleMass),
    inverseMass(0.0f) {
    if (!std::isfinite(mass) || mass <= 0.0f || !std::isfinite(1.0f / mass)) {
        throw std::invalid_argument("Particle mass must be positive and finite.");
    }
    inverseMass = 1.0f / mass;
    ValidateVector(position);
    ValidateVector(velocity);
}

void Particle::ApplyForce(const Vector2& appliedForce) {
    ValidateVector(appliedForce);
    ValidateVector(force);
    const Vector2 nextForce = CheckedVector(static_cast<double>(force.x) + appliedForce.x,
                                           static_cast<double>(force.y) + appliedForce.y);
    force = nextForce;
}

void Particle::Integrate(float deltaTime) {
    if (!std::isfinite(deltaTime) || deltaTime < 0.0f) {
        throw std::invalid_argument("Particle delta time must be finite and non-negative.");
    }

    ValidateVector(position); ValidateVector(velocity); ValidateVector(force);
    if (!std::isfinite(mass) || mass <= 0 || !std::isfinite(1.0f / mass))
        throw std::invalid_argument("Particle mass must have a finite positive reciprocal.");
    const float nextInverseMass = 1.0f / mass;
    const double dt = deltaTime;
    const double ax = static_cast<double>(force.x) / mass;
    const double ay = static_cast<double>(force.y) / mass;
    const Vector2 nextPosition = CheckedVector(
        position.x + velocity.x * dt + 0.5 * ax * dt * dt,
        position.y + velocity.y * dt + 0.5 * ay * dt * dt);
    const Vector2 nextVelocity = CheckedVector(velocity.x + ax * dt, velocity.y + ay * dt);
    position = nextPosition;
    velocity = nextVelocity;
    inverseMass = nextInverseMass;
    force = Vector2();
}

}
