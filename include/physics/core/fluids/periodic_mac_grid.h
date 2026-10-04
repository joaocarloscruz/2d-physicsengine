#pragma once
#include <cstddef>
#include <vector>

namespace PhysicsEngine {
struct PeriodicMacGridConfig {
    std::size_t columns=16,rows=16;
    double spacingX=1,spacingY=1;
    static constexpr std::size_t MaximumCells=262144;
    void Validate() const;
};
struct MacVelocityState {
    // Index i + columns*j. u is at (i*dx,(j+.5)*dy),
    // v at ((i+.5)*dx,j*dy). Each periodic face is stored exactly once.
    std::vector<double> xFaces,yFaces;
};
struct MacProjectionConfig {
    double density=1,timeStep=1;
    double absoluteDivergenceTolerance=1e-10,relativeDivergenceTolerance=1e-10;
    std::size_t maximumIterations=1000,maximumCellVisits=100000000;
    static constexpr std::size_t MaximumIterations=1000000,MaximumCellVisits=1000000000;
    void Validate() const;
};
struct MacProjectionDiagnostics {
    std::size_t iterations=0,cellVisits=0;
    double density=0,timeStep=0; // Accepted pressure conversion; no simulation clock.
    double initialDivergenceRms=0,finalDivergenceRms=0,targetDivergenceRms=0;
    double removedDivergenceMean=0,potentialMean=0,pressureMean=0;
    double initialMeanX=0,initialMeanY=0,finalMeanX=0,finalMeanY=0;
    double initialKineticEnergy=0,finalKineticEnergy=0,correctionKineticEnergy=0;
    double velocityCorrectionInnerProduct=0,divergencePotentialInnerProduct=0;
    double residualEnergyBound=0,storageEnergyError=0,roundoffEnergyAllowance=0;
    bool zeroDivergenceNoOp=false;
};
struct MacProjectionSnapshot {
    // Cell centers ((i+.5)*dx,(j+.5)*dy), zero-mean gauge.
    // Potential has units length²/time; pressure = density*potential/timeStep.
    std::vector<double> potential,pressure;
    MacProjectionDiagnostics diagnostics;
};
struct MacDiffusionConfig {
    double kinematicViscosity=0,timeStep=0,density=1;
    // RMS of the two-component equation residual, in velocity units.
    double absoluteVelocityTolerance=1e-10,relativeVelocityTolerance=1e-10;
    // Shared across both component solves, not a per-component allowance.
    std::size_t maximumIterations=1000,maximumCellVisits=100000000;
    static constexpr std::size_t MaximumIterations=1000000,MaximumCellVisits=1000000000;
    void Validate() const;
};
struct MacDiffusionDiagnostics {
    std::size_t iterations=0,iterationsX=0,iterationsY=0,cellVisits=0;
    double kinematicViscosity=0,timeStep=0,density=0;
    double initialVelocityRms=0,finalResidualRms=0,targetResidualRms=0;
    double initialMeanX=0,initialMeanY=0,finalMeanX=0,finalMeanY=0;
    double meanRoundoffAllowanceX=0,meanRoundoffAllowanceY=0;
    double initialKineticEnergy=0,finalKineticEnergy=0;
    double gradientDissipation=0,incrementKineticEnergy=0,residualWork=0;
    double residualEnergyBound=0,storageEnergyError=0,roundoffEnergyAllowance=0;
    bool zeroTransportNoOp=false;
};
// Periodic velocity projection only: no advection, forces, walls, free surfaces,
// World/SPH coupling. Constant-viscosity diffusion is a separate operation.
// Every public snapshot is an owning copy.
class PeriodicMacGrid {
public:
    explicit PeriodicMacGrid(const PeriodicMacGridConfig& config={});
    PeriodicMacGridConfig config() const { return config_; }
    MacVelocityState velocities() const { return velocities_; }
    MacProjectionSnapshot lastProjection() const { return projection_; }
    MacDiffusionDiagnostics lastDiffusion() const { return diffusion_; }
    void setVelocities(const MacVelocityState& velocities);
    std::vector<double> divergence() const;
    MacProjectionDiagnostics project(const MacProjectionConfig& config={});
    MacDiffusionDiagnostics diffuse(const MacDiffusionConfig& config={});
private:
    PeriodicMacGridConfig config_;
    MacVelocityState velocities_;
    MacProjectionSnapshot projection_;
    MacDiffusionDiagnostics diffusion_;
};
} // namespace PhysicsEngine
