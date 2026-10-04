#pragma once
#include "fluid_solver.h"

namespace PhysicsEngine {
struct DfsphConfig {
    Vector2 externalAcceleration = {0, -9.81f};
    float maximumTimeStep = 1.0f/60;
    float cflFactor = 0.25f;
    int maximumSubsteps = 1024;
    int maximumIterations = 200;
    float densityTolerance = 0.001f;
    float divergenceTolerance = 0.01f; // s^-1
    float relaxation = 0.5f;
    void Validate() const;
};

// Free-surface DFSPH with two relaxed Jacobi pressure projections. Negative
// pressure is clamped; expansion and free-surface density deficits are allowed.
class DfsphSolver final : public IFluidSolver {
public:
    explicit DfsphSolver(float referenceSmoothingLength, const DfsphConfig& config = {});
    void step(std::vector<FluidParticle>& particles, float deltaTime) override;
    FluidDiagnostics getDiagnostics() const override { return diagnostics; }
    const DfsphConfig& getConfig() const { return config; }
private:
    struct Pair { std::size_t a, b; Vector2 gradient; double viscosityCoupling = 0.0; };
    void prepare(std::vector<FluidParticle>& particles);
    void project(std::vector<FluidParticle>& particles, float dt, bool density);
    float stableTimeStep(const std::vector<FluidParticle>& particles) const;
    std::vector<Vector2> pressureAcceleration(const std::vector<FluidParticle>& particles,
        const std::vector<float>& pressure) const;
    std::vector<float> densityRates(const std::vector<FluidParticle>& particles,
        const std::vector<Vector2>& velocities) const;
    DfsphConfig config;
    FluidParticleSpatialGrid grid;
    std::vector<FluidParticleSpatialGrid::ParticlePair> neighbors;
    std::vector<Pair> pairs;
    std::vector<float> diagonal;
    double viscosityTimeLimit = 0.0;
    FluidDiagnostics diagnostics;
};
}
