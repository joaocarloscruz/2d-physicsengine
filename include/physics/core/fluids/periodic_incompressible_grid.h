#pragma once
#include "physics/core/fluids/periodic_mac_grid.h"

namespace PhysicsEngine {
struct PeriodicIncompressibleGridConfig {
    PeriodicMacGridConfig geometry;
    double density=1,kinematicViscosity=0;
    void Validate() const;
};
struct IncompressibleStepConfig {
    double cflSafety=.8;
    double absoluteDivergenceTolerance=1e-10,relativeDivergenceTolerance=1e-10;
    double absoluteVelocityTolerance=1e-10,relativeVelocityTolerance=1e-10;
    std::size_t maximumSubsteps=1000,maximumIterations=10000,maximumCellVisits=100000000;
    static constexpr std::size_t MaximumSubsteps=10000;
    static constexpr std::size_t MaximumIterations=MacProjectionConfig::MaximumIterations;
    static constexpr std::size_t MaximumCellVisits=MacProjectionConfig::MaximumCellVisits;
    void Validate() const;
};
struct IncompressibleSubstepDiagnostics {
    double timeStep=0,outgoingCfl=0,maximumDualDivergence=0,maximumRowSum=0;
    double initialKineticEnergy=0,advectedKineticEnergy=0;
    // Signed dual-divergence work, nonnegative donor loss, nonnegative FE increment.
    double dualDivergenceWork=0,donorDissipation=0,forwardEulerIncrementEnergy=0;
    double advectionEnergyChange=0,advectionStorageError=0,advectionEnergyBound=0;
    double roundoffEnergyAllowance=0,meanRoundoffAllowanceX=0,meanRoundoffAllowanceY=0;
    MacDiffusionDiagnostics diffusion;
    MacProjectionDiagnostics projection;
};
struct IncompressibleStepDiagnostics {
    double timeStep=0,initialTime=0,finalTime=0;
    std::size_t substeps=0,iterations=0,cellVisits=0;
    bool zeroStepNoOp=false;
    double initialMeanX=0,initialMeanY=0,finalMeanX=0,finalMeanY=0;
    double meanRoundoffAllowanceX=0,meanRoundoffAllowanceY=0;
    double initialKineticEnergy=0,finalKineticEnergy=0,storedEnergyChange=0;
    double dualDivergenceWork=0,donorDissipation=0,forwardEulerIncrementEnergy=0;
    double viscousDissipation=0,viscousIncrementEnergy=0,viscousResidualWork=0;
    double projectionCorrectionEnergy=0,projectionResidualWork=0;
    double residualEnergyBound=0,roundoffEnergyAllowance=0,storageEnergyError=0;
    MacProjectionDiagnostics initialProjection;
    // Bounded by maximumSubsteps <= MaximumSubsteps, one record per accepted substep.
    std::vector<IncompressibleSubstepDiagnostics> history;
};
// Native, constant-density periodic Navier-Stokes flow. Face-center samples,
// SI units (m,s,kg); geometry/materials immutable. No World/SPH integration.
// Positive steps project input, then donor FE advection / BE viscosity / projection.
// Every getter returns an owning copy. Setters retain clock and operator history.
class PeriodicIncompressibleGrid {
public:
    explicit PeriodicIncompressibleGrid(const PeriodicIncompressibleGridConfig& config={});
    PeriodicIncompressibleGridConfig config() const { return config_; }
    MacVelocityState velocities() const { return mac_.velocities(); }
    MacProjectionSnapshot lastProjection() const { return mac_.lastProjection(); }
    MacDiffusionDiagnostics lastDiffusion() const { return mac_.lastDiffusion(); }
    IncompressibleStepDiagnostics lastStep() const { return step_; }
    double time() const { return time_; }
    void setVelocities(const MacVelocityState& velocities) { mac_.setVelocities(velocities); }
    std::vector<double> divergence() const { return mac_.divergence(); }
    IncompressibleStepDiagnostics step(double timeStep,const IncompressibleStepConfig& config={});
private:
    PeriodicIncompressibleGridConfig config_;
    PeriodicMacGrid mac_;
    double time_=0;
    IncompressibleStepDiagnostics step_;
};
} // namespace PhysicsEngine
