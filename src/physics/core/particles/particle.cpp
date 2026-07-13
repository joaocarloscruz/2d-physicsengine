#include "physics/core/particles/particle.h"

#include <cmath>
#include <stdexcept>

namespace PhysicsEngine {

Particle::Particle(
    const Vector2& initialPosition,
    const Vector2& initialVelocity,
    float particleMass
) : position(initialPosition),
    velocity(initialVelocity),
    force(),
    mass(particleMass),
    inverseMass(0.0f) {
    if (!std::isfinite(mass) || mass <= 0.0f) {
        throw std::invalid_argument("Particle mass must be positive and finite.");
    }
    inverseMass = 1.0f / mass;
}

void Particle::ApplyForce(const Vector2& appliedForce) {
    force = force + appliedForce;
}

void Particle::Integrate(float deltaTime) {
    if (!std::isfinite(deltaTime) || deltaTime < 0.0f) {
        throw std::invalid_argument("Particle delta time must be finite and non-negative.");
    }

    const Vector2 acceleration = force * inverseMass;
    position = position + velocity * deltaTime
        + acceleration * (0.5f * deltaTime * deltaTime);
    velocity = velocity + acceleration * deltaTime;
    force = Vector2();
}

}
