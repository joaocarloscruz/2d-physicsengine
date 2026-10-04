#pragma once
#include "fluid_particle.h"
#include "sph_kernels.h"
#include "fluid_particle_spatial_grid.h"
#include <cstdint>

namespace PhysicsEngine {
struct FluidDiagnostics {
    float maximumDensityError = 0; // |rho/rho0 - 1|, including free surfaces
    float maximumCompression = 0; // max(rho/rho0 - 1, 0)
    // Common mean-support pressure-gradient comparison, including supplied wall
    // rates. Not generally the derivative of the summation density weight.
    float maximumAbsoluteDensityRate = 0; // |comparison rate| / rho0, s^-1
    float maximumCompressionRate = 0; // max(comparison rate / rho0, 0), s^-1
    float densityResidual = 0; // final predicted compression in pressure solve
    float divergenceResidual = 0; // final positive density-rate residual
    std::uint32_t densityIterations = 0;
    std::uint32_t divergenceIterations = 0;
    std::uint32_t substeps = 0;
    bool converged = true;
};

class IFluidSolver {
public:
    virtual ~IFluidSolver() = default;
    virtual void step(std::vector<FluidParticle>& particles, float deltaTime) = 0;
    virtual FluidDiagnostics getDiagnostics() const = 0;
};

// Same measurement operator for both methods; does not mutate any input. Checks
// used particle fields, pair indices and finite output arithmetic. Repeated pairs
// are accumulated repeatedly in caller order; self pairs contribute zero.
// Malformed data/indices throw; unsupported float arithmetic throws overflow_error.
FluidDiagnostics MeasureFluidDiagnostics(const std::vector<FluidParticle>& particles,
    const std::vector<FluidParticleSpatialGrid::ParticlePair>& pairs);
// Prepared wall rates exclude density diffusion; empty means no wall contribution.
FluidDiagnostics MeasureFluidDiagnostics(const std::vector<FluidParticle>& particles,
    const std::vector<FluidParticleSpatialGrid::ParticlePair>& pairs,
    SphKernelFamily family, const std::vector<float>& additionalDensityRates = {});
}
