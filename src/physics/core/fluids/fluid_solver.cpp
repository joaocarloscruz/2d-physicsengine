#include "physics/core/fluids/fluid_solver.h"
#include "physics/core/fluids/sph_kernels.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace PhysicsEngine {
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
    FluidDiagnostics result;
    std::vector<float> rates(particles.size(), 0);
    if (!additionalDensityRates.empty()) rates = additionalDensityRates;
    for (float rate : rates)
        if (!std::isfinite(rate)) throw std::invalid_argument("Diagnostic density rates must be finite.");
    for (const auto& pair : pairs) {
        const auto& a = particles[pair.first]; const auto& b = particles[pair.second];
        const Vector2 gradient = SphKernels2D::PressureGradient(a.position-b.position,
            static_cast<float>(0.5*(static_cast<double>(a.smoothingLength)+b.smoothingLength)), family);
        const float rate = (a.velocity-b.velocity).dot(gradient);
        rates[pair.first] += b.mass*rate; rates[pair.second] += a.mass*rate;
    }
    for (std::size_t i=0; i<particles.size(); ++i) {
        const auto& p = particles[i];
        const float error = p.density/p.restDensity-1;
        result.maximumDensityError = std::max(result.maximumDensityError, std::abs(error));
        result.maximumCompression = std::max(result.maximumCompression, error);
        result.maximumAbsoluteDensityRate = std::max(result.maximumAbsoluteDensityRate, std::abs(rates[i])/p.restDensity);
        result.maximumCompressionRate = std::max(result.maximumCompressionRate, rates[i]/p.restDensity);
    }
    return result;
}
}
