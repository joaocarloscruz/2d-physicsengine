#include "physics/core/fluids/fluid_particle.h"

#include <cmath>
#include <stdexcept>

namespace PhysicsEngine {

void FluidParticleProperties::Validate() const {
    if (!std::isfinite(mass) || mass <= 0.0f) {
        throw std::invalid_argument(
            "Fluid particle mass must be positive and finite."
        );
    }
    if (!std::isfinite(restDensity) || restDensity <= 0.0f) {
        throw std::invalid_argument(
            "Fluid rest density must be positive and finite."
        );
    }
    if (!std::isfinite(smoothingLength) || smoothingLength <= 0.0f) {
        throw std::invalid_argument(
            "Fluid smoothing length must be positive and finite."
        );
    }
    if (!std::isfinite(viscosity) || viscosity < 0.0f) {
        throw std::invalid_argument(
            "Fluid viscosity must be finite and non-negative."
        );
    }
}

FluidParticle::FluidParticle(
    const Vector2& initialPosition,
    const Vector2& initialVelocity,
    const FluidParticleProperties& properties
) : position(initialPosition),
    velocity(initialVelocity),
    force(),
    mass(properties.mass),
    inverseMass(0.0f),
    density(properties.restDensity),
    pressure(0.0f),
    smoothingLength(properties.smoothingLength),
    restDensity(properties.restDensity),
    viscosity(properties.viscosity),
    volume(0.0f) {
    properties.Validate();
    if (!std::isfinite(position.x) || !std::isfinite(position.y)
        || !std::isfinite(velocity.x) || !std::isfinite(velocity.y)) {
        throw std::invalid_argument(
            "Fluid particle position and velocity must be finite."
        );
    }
    inverseMass = 1.0f / mass;
    volume = mass / restDensity;
}

}
