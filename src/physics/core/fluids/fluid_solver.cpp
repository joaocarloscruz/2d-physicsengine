#include "physics/core/fluids/fluid_solver.h"
#include "physics/core/fluids/sph_kernels.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
void ValidateDiagnosticParticle(const FluidParticle& p) {
    if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y)
        || !std::isfinite(p.velocity.x) || !std::isfinite(p.velocity.y)
        || !std::isfinite(p.mass) || p.mass <= 0
        || !std::isfinite(p.density) || p.density <= 0
        || !std::isfinite(p.restDensity) || p.restDensity <= 0
        || !std::isfinite(p.smoothingLength) || p.smoothingLength <= 0)
        throw std::invalid_argument("Fluid diagnostic particle data is invalid.");
}
float CheckedMeasurement(float value) {
    if (!std::isfinite(value))
        throw std::overflow_error("Fluid diagnostic arithmetic exceeds float range.");
    return value;
}
} // namespace
FluidDiagnostics MeasureFluidDiagnostics(const std::vector<FluidParticle>& particles,
    const std::vector<FluidParticleSpatialGrid::ParticlePair>& pairs) {
    return MeasureFluidDiagnostics(particles, pairs, SphKernelFamily::Poly6Spiky);
}
FluidDiagnostics MeasureFluidDiagnostics(const std::vector<FluidParticle>& particles,
    const std::vector<FluidParticleSpatialGrid::ParticlePair>& pairs,
    SphKernelFamily family, const std::vector<float>& additionalDensityRates) {
    SphKernels2D::ValidateFamily(family);
    if (!additionalDensityRates.empty() && additionalDensityRates.size() != particles.size())
        throw std::invalid_argument("Diagnostic additional density rates have the wrong size.");
    for (const auto& particle : particles) ValidateDiagnosticParticle(particle);
    FluidDiagnostics result;
    std::vector<float> rates(particles.size(), 0);
    if (!additionalDensityRates.empty()) rates = additionalDensityRates;
    for (float rate : rates)
        if (!std::isfinite(rate)) throw std::invalid_argument("Diagnostic density rates must be finite.");
    for (const auto& pair : pairs) {
        if (pair.first >= particles.size() || pair.second >= particles.size())
            throw std::out_of_range("Fluid diagnostic pair index is out of range.");
        const auto& a = particles[pair.first]; const auto& b = particles[pair.second];
        const Vector2 displacement = a.position-b.position;
        const Vector2 relativeVelocity = a.velocity-b.velocity;
        CheckedMeasurement(displacement.x); CheckedMeasurement(displacement.y);
        CheckedMeasurement(relativeVelocity.x); CheckedMeasurement(relativeVelocity.y);
        const Vector2 gradient = SphKernels2D::PressureGradient(displacement,
            static_cast<float>(0.5*(static_cast<double>(a.smoothingLength)+b.smoothingLength)), family);
        const float rate = CheckedMeasurement(relativeVelocity.dot(gradient));
        rates[pair.first] = CheckedMeasurement(rates[pair.first] + b.mass*rate);
        rates[pair.second] = CheckedMeasurement(rates[pair.second] + a.mass*rate);
    }
    for (std::size_t i=0; i<particles.size(); ++i) {
        const auto& p = particles[i];
        const float error = CheckedMeasurement(p.density/p.restDensity-1);
        const float normalizedRate = CheckedMeasurement(rates[i]/p.restDensity);
        result.maximumDensityError = std::max(result.maximumDensityError, std::abs(error));
        result.maximumCompression = std::max(result.maximumCompression, error);
        result.maximumAbsoluteDensityRate = std::max(result.maximumAbsoluteDensityRate, std::abs(normalizedRate));
        result.maximumCompressionRate = std::max(result.maximumCompressionRate, normalizedRate);
    }
    return result;
}
}
