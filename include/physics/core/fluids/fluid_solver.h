#pragma once
#include "fluid_particle.h"
#include "fluid_particle_spatial_grid.h"
#include <cstdint>

namespace PhysicsEngine {
struct FluidDiagnostics {
    float maximumDensityError = 0; // |rho/rho0 - 1|, including free surfaces
    float maximumCompression = 0; // max(rho/rho0 - 1, 0)
    float maximumAbsoluteDensityRate = 0; // |D rho / Dt| / rho0, s^-1
    float maximumCompressionRate = 0; // max(D rho / Dt / rho0, 0), s^-1
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

// Same measurement operator for both methods; does not mutate particle state.
FluidDiagnostics MeasureFluidDiagnostics(const std::vector<FluidParticle>& particles,
    const std::vector<FluidParticleSpatialGrid::ParticlePair>& pairs);
}
