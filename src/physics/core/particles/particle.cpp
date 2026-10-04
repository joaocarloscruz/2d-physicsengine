#include "physics/core/particles/particle.h"

#include <cmath>
#include <stdexcept>

namespace PhysicsEngine {

namespace {
void ValidateVector(const Vector2& value) {
    if (!std::isfinite(value.x) || !std::isfinite(value.y))
        throw std::invalid_argument("Particle state and forces must be finite.");
}
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
    const Vector2 nextForce = force + appliedForce;
    ValidateVector(nextForce);
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
    const Vector2 acceleration = force * nextInverseMass;
    const Vector2 nextPosition = position + velocity * deltaTime
        + acceleration * (0.5f * deltaTime * deltaTime);
    const Vector2 nextVelocity = velocity + acceleration * deltaTime;
    ValidateVector(nextPosition); ValidateVector(nextVelocity);
    position = nextPosition;
    velocity = nextVelocity;
    inverseMass = nextInverseMass;
    force = Vector2();
}

}
