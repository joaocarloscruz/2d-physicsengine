#pragma once
#include <cstddef>
#include <vector>

namespace PhysicsEngine {
struct MaxwellGridConfig {
    std::size_t columns = 16, rows = 16;
    double spacingX = 1, spacingY = 1;
    double permittivity = 1, permeability = 1; // F/m, H/m in SI; defaults use reduced units.
    double cflSafety = 0.9, maxSubstep = 0.1;
    std::size_t maximumSubsteps = 10000, maximumCellVisits = 100000000;
    static constexpr std::size_t MaximumCells = 262144;
    static constexpr std::size_t MaximumSubsteps = 1000000;
    static constexpr std::size_t MaximumCellVisits = 1000000000;
    void Validate() const;
};
struct MaxwellFieldState {
    // Index i + columns*j; periodic samples stored once. All components share
    // the same time: Ez at (i*dx,j*dy), Hx at (i*dx,(j+.5)*dy),
    // Hy at ((i+.5)*dx,j*dy).
    std::vector<double> ez, hx, hy;
};
struct MaxwellGridDiagnostics {
    double electricEnergy = 0, magneticEnergy = 0, totalEnergy = 0;
    double modifiedEnergy = 0, modifiedEnergyStep = 0;
    double meanEz = 0, meanHx = 0, meanHy = 0;
    double maxAbsEz = 0, maxAbsHx = 0, maxAbsHy = 0;
    // Divergence of H, in A/m^2 in SI. Divergence of B is mu times this value.
    double magneticDivergenceRms = 0, maxAbsMagneticDivergence = 0;
    double time = 0, stableTimeStep = 0, lastSubstep = 0;
    std::size_t lastSubsteps = 0, lastCellVisits = 0;
};
// Return-only ledger for one accepted explicitly Ohmic operation; J/m in SI.
// Exact subflow work uses the measured pre-decay physical electric energy.
// Represented loss is the difference of measured pre/post electric energies;
// storage discrepancy includes field rounding and energy-measurement roundoff.
struct MaxwellOhmicStepDiagnostics {
    double conductivity = 0, duration = 0, startTime = 0, endTime = 0;
    double initialPhysicalEnergy = 0, finalPhysicalEnergy = 0;
    double exactJouleEnergy = 0, representedElectricEnergyLoss = 0;
    double wavePhysicalEnergyChange = 0, modifiedEnergyDissipation = 0;
    double decayStorageEnergyChange = 0, physicalBalanceResidual = 0;
    double substep = 0;
    std::size_t substeps = 0, cellVisits = 0;
};
// Independent homogeneous periodic TMz fields; step is lossless and explicitly
// requested stepOhmic adds homogeneous scalar conductivity. No other sources, material
// interfaces, particle coupling or World integration. Geometry/medium/budgets
// are immutable. All public snapshots own their values.
class MaxwellGrid {
  public:
    explicit MaxwellGrid(const MaxwellGridConfig &config = {});
    MaxwellGridConfig getConfig() const { return config_; }
    MaxwellFieldState getState() const { return state_; }
    MaxwellGridDiagnostics getDiagnostics() const { return diagnostics_; }
    double getWaveSpeed() const { return speed_; }
    double getStableTimeStep() const { return limit_; }
    // Does not reset the clock. Resets last-step work/reference h to zero and
    // recomputes field diagnostics. Failure preserves all fields and diagnostics.
    void setState(const MaxwellFieldState &state);
    // Dx_forward Hx + Dy_forward Hy at ((i+.5)*dx,(j+.5)*dy), in A/m^2 in SI.
    // The physical constraint div B = 0 is equivalent for uniform mu > 0.
    // Initial divergence is observed and preserved, never projected away.
    std::vector<double> getMagneticDivergence() const;
    // Fixed-h invariant at a caller-supplied reference step. h=0 gives raw
    // physical energy; positive h must satisfy the strict physical CFL bound.
    double getModifiedEnergy(double h) const;
    // Symmetric H half-kick / E full-drift / H half-kick. Zero is a complete
    // no-op. Failure preserves fields, clock and diagnostic counters.
    void step(double dt);
    // S/m conductivity; symmetric exact E decay / wave / exact E decay.
    // No persistent heat ledger or thermal feedback. Failed calls publish nothing.
    // sigma=0 delegates exactly to step, including its original work budget.
    MaxwellOhmicStepDiagnostics stepOhmic(double dt, double conductivity);

  private:
    MaxwellGridConfig config_;
    MaxwellFieldState state_;
    MaxwellGridDiagnostics diagnostics_;
    double ix_, iy_, electricX_, electricY_, magneticX_, magneticY_;
    double electricScale_, magneticScale_, speed_, rate_, limit_;
    MaxwellGridDiagnostics measure(const MaxwellFieldState &state, double h,
                                   double *electricModifiedPart = nullptr) const;
    struct StepPlan {
        double h, time;
        std::size_t count;
    };
    StepPlan plan(double dt, std::size_t passesPerSubstep) const;
    void waveStep(MaxwellFieldState &state, double h) const;
    double divergenceAt(const MaxwellFieldState &state, std::size_t i, std::size_t j) const;
};
} // namespace PhysicsEngine
