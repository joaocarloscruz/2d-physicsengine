#ifndef COUPLED_FLUID_SIMULATION_H
#define COUPLED_FLUID_SIMULATION_H

#include "fluid_rigid_coupler.h"
#include "wcsph_solver.h"

#include <cstdint>
#include <vector>

namespace PhysicsEngine {

struct CoupledFluidStatistics {
    WcsphStatistics fluid;
    FluidRigidCouplingStatistics coupling;
    std::uint32_t rigidBodySubstepCount = 0;
    float integratedTime = 0.0f;
};

class CoupledFluidSimulation {
public:
    CoupledFluidSimulation(
        float referenceSmoothingLength,
        const WcsphConfig& fluidConfig = WcsphConfig{},
        const FluidRigidCouplingSettings& couplingSettings =
            FluidRigidCouplingSettings{}
    );

    void step(
        std::vector<FluidParticle>& particles,
        const std::vector<RigidBody*>& bodies,
        float deltaTime
    );
    void step(
        std::vector<FluidParticle>& particles,
        const std::vector<RigidBody*>& bodies,
        float deltaTime,
        const IFluidContainer& boundary
    );

    const CoupledFluidStatistics& getLastStatistics() const;

private:
    void resetStatistics();
    void advanceRigidSubstep(
        std::vector<FluidParticle>& particles,
        const std::vector<RigidBody*>& bodies,
        float deltaTime
    );

    WcsphSolver fluidSolver;
    FluidRigidCoupler coupler;
    FluidBoundarySamplingSettings boundarySampling;
    CoupledFluidStatistics lastStatistics;
};

} // namespace PhysicsEngine

#endif // COUPLED_FLUID_SIMULATION_H
