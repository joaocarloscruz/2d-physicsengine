#pragma once

#include "physics/core/fluids/fluid_particle.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace PhysicsEngine::SphViscosity {

inline float CheckedFloat(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
        throw std::overflow_error("SPH force or integrated state exceeds float range.");
    }
    return static_cast<float>(value);
}

inline Vector2 CheckedVector(double x, double y) {
    return Vector2(CheckedFloat(x), CheckedFloat(y));
}

inline float TimeStep(double value) {
    float result = CheckedFloat(value);
    if (result > value) {
        result = std::nextafter(result, 0.0f);
    }
    if (result <= 0.0f) {
        throw std::runtime_error("SPH stable timestep is not representable as a positive float.");
    }
    return result;
}

inline double ContinuumLimit(const FluidParticle& particle) {
    return particle.viscosity > 0.0f
        ? 0.125 * particle.smoothingLength * particle.smoothingLength
            * particle.density / particle.viscosity
        : std::numeric_limits<double>::infinity();
}

inline double Coupling(const FluidParticle& first, const FluidParticle& second,
                       float laplacian) {
    const double viscosity = 0.5 * (static_cast<double>(first.viscosity) + second.viscosity);
    const double coefficient = viscosity * first.mass * second.mass
        / (static_cast<double>(first.density) * second.density) * laplacian;
    if (!std::isfinite(coefficient) || coefficient < 0.0) {
        throw std::overflow_error("SPH viscosity coupling is not finite and non-negative.");
    }
    return coefficient;
}

inline double RowLimit(const std::vector<double>& rows) {
    double maximum = 0.0;
    for (double row : rows) {
        if (!std::isfinite(row) || row < 0.0) {
            throw std::overflow_error("SPH viscosity diffusion rate exceeds double range.");
        }
        maximum = std::max(maximum, row);
    }
    // M^-1 L is similar to a symmetric positive-semidefinite diffusion operator.
    // Gershgorin bounds its largest eigenvalue by 2*max(row). Taking half the
    // reciprocal row rate keeps explicit Euler conservative and dissipative.
    return maximum > 0.0 ? 0.5 / maximum : std::numeric_limits<double>::infinity();
}

inline Vector2 PairForce(const FluidParticle& first, const FluidParticle& second,
                         double coefficient) {
    return CheckedVector((static_cast<double>(second.velocity.x) - first.velocity.x) * coefficient,
                         (static_cast<double>(second.velocity.y) - first.velocity.y) * coefficient);
}

inline Vector2 Add(const Vector2& first, const Vector2& second, double scale = 1.0) {
    return CheckedVector(first.x + static_cast<double>(second.x) * scale,
                         first.y + static_cast<double>(second.y) * scale);
}

inline Vector2 AdvanceVelocity(const FluidParticle& particle, float timeStep) {
    const double factor = static_cast<double>(timeStep) / particle.mass;
    return Add(particle.velocity, particle.force, factor);
}

} // namespace PhysicsEngine::SphViscosity
