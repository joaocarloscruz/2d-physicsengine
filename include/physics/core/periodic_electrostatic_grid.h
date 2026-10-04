#pragma once
#include <cstddef>
#include <vector>

namespace PhysicsEngine {
struct ElectrostaticGridConfig {
    std::size_t columns = 16, rows = 16;
    double spacingX = 1, spacingY = 1, permittivity = 1;
    static constexpr std::size_t MaximumCells = 262144;
    void Validate() const;
};
struct ElectrostaticSolveConfig {
    // Charge-density RMS units (C/m^3 in SI), plus relative * effective source RMS.
    double absoluteGaussTolerance = 1e-10, relativeGaussTolerance = 1e-10;
    std::size_t maximumIterations = 1000, maximumCellVisits = 100000000;
    static constexpr std::size_t MaximumIterations = 1000000, MaximumCellVisits = 1000000000;
    void Validate() const;
};
struct ElectrostaticField {
    std::vector<double> xFaces, yFaces;
};
struct ElectrostaticDiagnostics {
    std::size_t iterations = 0, cellVisits = 0, residualRestarts = 0;
    double permittivity = 1;
    double originalChargeMean = 0, effectiveChargeMean = 0;
    double originalIntegratedCharge = 0, effectiveIntegratedCharge = 0;
    double neutralityMeanAllowance = 0, removedChargeMean = 0;
    double maximumSourceCorrection = 0, sourceCorrectionAllowance = 0;
    double effectiveChargeRms = 0, targetGaussRms = 0, finalGaussRms = 0, maximumAbsGauss = 0;
    double originalGaussRms = 0, maximumAbsOriginalGauss = 0;
    double potentialMean = 0, meanFieldX = 0, meanFieldY = 0, curlRms = 0, maximumAbsCurl = 0;
    double fieldEnergy = 0, sourceEnergy = 0, residualEnergyCorrection = 0;
    double residualEnergyBound = 0, energyIdentityError = 0, roundoffEnergyAllowance = 0;
    bool hasSolution = false, zeroSource = false;
};
struct ElectrostaticSnapshot {
    std::vector<double> originalCharge, effectiveCharge, potential;
    ElectrostaticField field;
    // epsilon*div(stored E)-effectiveCharge, and corner curl(stored E).
    std::vector<double> gaussResidual, curl;
    ElectrostaticDiagnostics diagnostics;
};
// Static homogeneous periodic -epsilon Lap(phi)=rho, E=-grad(phi).
// Cell centers ((i+.5)dx,(j+.5)dy), x faces (i dx,(j+.5)dy), y faces
// ((i+.5)dx,j dy); row-major i+columns*j, periodic faces stored once.
// Zero-mean potential and zero harmonic/DC electric field. Caller-prescribed
// charge only, no clock, point-charge deposition, particle feedback or coupling.
// SI rho C/m^3, epsilon F/m, phi V, E V/m; charge C/m and energy J/m per depth.
class PeriodicElectrostaticGrid {
  public:
    explicit PeriodicElectrostaticGrid(const ElectrostaticGridConfig &config = {});
    ElectrostaticGridConfig getConfig() const { return config_; }
    ElectrostaticSnapshot getSnapshot() const { return snapshot_; }
    // Material nonneutrality is rejected. Roundoff-sized uniform mean removal
    // is explicitly bounded/reported, with original and effective sources kept.
    // All successful publications are atomic; any failure preserves the snapshot.
    ElectrostaticDiagnostics solve(const std::vector<double> &charge,
                                   const ElectrostaticSolveConfig &options = {});

  private:
    ElectrostaticGridConfig config_;
    ElectrostaticSnapshot snapshot_;
};
} // namespace PhysicsEngine
