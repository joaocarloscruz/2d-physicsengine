#ifndef WCSPH_SOLVER_H
#define WCSPH_SOLVER_H

#include "fluid_particle.h"
#include "fluid_particle_spatial_grid.h"
#include "fluid_boundary.h"

#include <cstdint>
#include <functional>
#include <vector>

namespace PhysicsEngine {

enum class WcsphDensityMode {
    Summation,
    Continuity
};

struct WcsphConfig {
    Vector2 externalAcceleration = Vector2(0.0f, -9.81f);
    float speedOfSound = 20.0f;
    float equationOfStateExponent = 7.0f;
    float cflFactor = 0.25f;
    float maximumTimeStep = 1.0f / 60.0f;
    int maximumSubsteps = 1024;
    bool clampNegativePressure = true;
    WcsphDensityMode densityMode = WcsphDensityMode::Summation;
    float densityDiffusion = 0.1f;

    void Validate() const;
};

struct WcsphStatistics {
    std::uint32_t substepCount = 0;
    FluidNeighborStatistics neighbors;
    float minimumDensity = 0.0f;
    float maximumDensity = 0.0f;
    float maximumSpeed = 0.0f;
    float stableTimeStep = 0.0f;
    std::size_t boundaryParticleCount = 0;
    std::uint64_t boundaryCandidateCount = 0;
    std::uint64_t boundaryCorrectionCount = 0;
    float maximumBoundaryPenetration = 0.0f;
};

class WcsphSolver {
public:
    using SubstepCallback = std::function<void(float)>;

    explicit WcsphSolver(
        float referenceSmoothingLength,
        const WcsphConfig& config = WcsphConfig{}
    );

    void prepare(std::vector<FluidParticle>& particles);
    void prepare(
        std::vector<FluidParticle>& particles,
        const std::vector<FluidBoundaryParticle>& boundaryParticles
    );
    void step(std::vector<FluidParticle>& particles, float deltaTime);
    void step(
        std::vector<FluidParticle>& particles,
        float deltaTime,
        const IFluidContainer& boundary
    );
    void step(
        std::vector<FluidParticle>& particles,
        float deltaTime,
        const SubstepCallback& afterSubstep
    );
    void step(
        std::vector<FluidParticle>& particles,
        float deltaTime,
        const std::vector<FluidBoundaryParticle>& boundaryParticles,
        const SubstepCallback& afterSubstep
    );
    void step(
        std::vector<FluidParticle>& particles,
        float deltaTime,
        const IFluidContainer& boundary,
        const SubstepCallback& afterSubstep
    );
    void step(
        std::vector<FluidParticle>& particles,
        float deltaTime,
        const IFluidContainer& boundary,
        const std::vector<FluidBoundaryParticle>& boundaryParticles,
        const SubstepCallback& afterSubstep
    );
    float getStableTimeStep(
        const std::vector<FluidParticle>& particles
    ) const;

    const WcsphConfig& getConfig() const;
    const WcsphStatistics& getLastStatistics() const;

private:
    void prepareState(
        std::vector<FluidParticle>& particles,
        const std::vector<FluidBoundaryParticle>* boundaryParticles
    );
    void integrate(std::vector<FluidParticle>& particles, float deltaTime);
    void stepInternal(
        std::vector<FluidParticle>& particles,
        float deltaTime,
        const IFluidContainer* boundary,
        const std::vector<FluidBoundaryParticle>* boundaryParticles,
        const SubstepCallback* afterSubstep
    );

    WcsphConfig config;
    FluidParticleSpatialGrid grid;
    WcsphStatistics lastStatistics;
};

}

#endif // WCSPH_SOLVER_H
