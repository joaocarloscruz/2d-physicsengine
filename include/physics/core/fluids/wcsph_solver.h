#ifndef WCSPH_SOLVER_H
#define WCSPH_SOLVER_H

#include "fluid_particle.h"
#include "fluid_particle_spatial_grid.h"
#include "fluid_boundary.h"
#include "fluid_solver.h"

#include <cstdint>
#include <functional>
#include <vector>

namespace PhysicsEngine {

enum class WcsphDensityMode {
    Summation,
    Continuity
};

enum class WcsphWallPressureMode {
    LegacyPositiveIncrement,
    SignedBodyForce
};

// Order of the continuity-density update inside one semi-implicit Euler step.
enum class WcsphTimeIntegration {
    // Legacy: density rate is evaluated from pre-kick velocities, which makes
    // the velocity/density acoustic pair a forward-Euler (undamped-unstable) map.
    ExplicitDensity,
    // Kick velocity, evaluate the continuity rate from the kicked velocity at
    // the old positions, then drift position and density together. Density is
    // treated like position, matching the summation-density update order.
    SymplecticEuler
};

// Constant-pressure consistency of the dummy wall force. RawSamples applies the
// sampled wall terms as they are. ConstantPressureCompletion adds, for fluid
// particles whose wall samples nearly complete their kernel stencil, the small
// correction p_i (S_fluid + S_wall) that makes a uniform pressure exert no
// force; it removes sub-spacing quadrature defects (sample staggering, gaps,
// compressed fluid beside rest-spaced samples). See docs/wcsph-solver.md.
enum class WcsphWallConsistency {
    RawSamples,
    ConstantPressureCompletion
};

struct WcsphConfig {
    Vector2 externalAcceleration = Vector2(0.0f, -9.81f);
    float speedOfSound = 20.0f;
    float equationOfStateExponent = 7.0f;
    float cflFactor = 0.25f;
    float maximumTimeStep = 1.0f / 60.0f;
    int maximumSubsteps = 1024;
    bool clampNegativePressure = true;
    WcsphDensityMode densityMode = WcsphDensityMode::Continuity;
    float densityDiffusion = 0.1f;
    // Dimensionless bulk-viscosity coefficient beta; zeta = beta * h * c.
    float acousticDamping = 0.5f;
    // Dimensionless particle-shifting coefficient s: fully supported particles
    // drift by -s*h*c*dt*grad(C), C = sum_j V_j W_ij (fluid and wall volumes).
    float particleShifting = 0.02f;

    // Wendland C2 keeps the pressurized square lattice stable at h = 2 dx: the
    // symmetric pressure term's uniform-pressure stiffness sum_j H(r_ij)(1 - cos k.r_ij)
    // is positive semidefinite for it, but has negative transverse eigenvalues
    // for the spiky and cubic gradients, which then buckle under hydrostatic load.
    SphKernelFamily kernelFamily = SphKernelFamily::WendlandC2;
    WcsphWallPressureMode wallPressureMode = WcsphWallPressureMode::LegacyPositiveIncrement;
    WcsphTimeIntegration timeIntegration = WcsphTimeIntegration::SymplecticEuler;
    WcsphWallConsistency wallConsistency =
        WcsphWallConsistency::ConstantPressureCompletion;

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

class WcsphSolver : public IFluidSolver {
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
    void step(std::vector<FluidParticle>& particles, float deltaTime) override;
    FluidDiagnostics getDiagnostics() const override { return diagnostics; }
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
    void accumulateDensityRates(
        std::vector<FluidParticle>& particles,
        std::vector<float>* wallDensityRates
    ) const;
    // Continuity-rate terms at the prepared neighbors: compression from fluid
    // pairs (when includePairs), delta-SPH diffusion (when includeDiffusion)
    // and the wall normal-velocity rate, accumulated into rates.
    void densityRateTerms(
        const std::vector<FluidParticle>& particles,
        bool includePairs,
        bool includeDiffusion,
        std::vector<float>& rates,
        std::vector<float>* wallDensityRates
    ) const;

    void computeShiftVelocities(const std::vector<FluidParticle>& particles);

    struct BoundaryPair {
        std::size_t fluid;
        std::size_t boundary;
    };
    void stepInternal(
        std::vector<FluidParticle>& particles,
        float deltaTime,
        const IFluidContainer* boundary,
        const std::vector<FluidBoundaryParticle>* boundaryParticles,
        const SubstepCallback* afterSubstep
    );

    WcsphConfig config;
    FluidParticleSpatialGrid grid;
    // Neighbor lists of the most recent prepareState, reused by integrate().
    std::vector<FluidParticleSpatialGrid::ParticlePair> pairs;
    std::vector<BoundaryPair> boundaryPairs;
    const std::vector<FluidBoundaryParticle>* preparedBoundary = nullptr;
    std::vector<Vector2> shiftVelocities;
    WcsphStatistics lastStatistics;
    FluidDiagnostics diagnostics;
};

}

#endif // WCSPH_SOLVER_H
