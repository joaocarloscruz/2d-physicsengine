#ifndef WCSPH_SOLVER_H
#define WCSPH_SOLVER_H

#include "fluid_particle.h"
#include "fluid_particle_spatial_grid.h"

#include <cstdint>
#include <vector>

namespace PhysicsEngine {

struct WcsphConfig {
    Vector2 externalAcceleration = Vector2(0.0f, -9.81f);
    float speedOfSound = 20.0f;
    float equationOfStateExponent = 7.0f;
    float cflFactor = 0.25f;
    float maximumTimeStep = 1.0f / 60.0f;
    int maximumSubsteps = 1024;
    bool clampNegativePressure = true;

    void Validate() const;
};

struct WcsphStatistics {
    std::uint32_t substepCount = 0;
    FluidNeighborStatistics neighbors;
    float minimumDensity = 0.0f;
    float maximumDensity = 0.0f;
    float maximumSpeed = 0.0f;
    float stableTimeStep = 0.0f;
};

class WcsphSolver {
public:
    explicit WcsphSolver(
        float referenceSmoothingLength,
        const WcsphConfig& config = WcsphConfig{}
    );

    void prepare(std::vector<FluidParticle>& particles);
    void step(std::vector<FluidParticle>& particles, float deltaTime);
    float getStableTimeStep(
        const std::vector<FluidParticle>& particles
    ) const;

    const WcsphConfig& getConfig() const;
    const WcsphStatistics& getLastStatistics() const;

private:
    void prepareState(std::vector<FluidParticle>& particles);
    void integrate(std::vector<FluidParticle>& particles, float deltaTime);

    WcsphConfig config;
    FluidParticleSpatialGrid grid;
    WcsphStatistics lastStatistics;
};

}

#endif // WCSPH_SOLVER_H
