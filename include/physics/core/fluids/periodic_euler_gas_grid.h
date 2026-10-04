#pragma once
#include <cstddef>
#include <vector>

namespace PhysicsEngine {
struct EulerGasGridConfig {
    std::size_t columns = 16, rows = 16;
    double spacingX = 1, spacingY = 1, gamma = 1.4;
    static constexpr std::size_t MaximumCells = 262144;
    void Validate() const;
};
struct EulerGasStepConfig {
    double cflSafety = .9, maxSubstep = .1;
    std::size_t maximumSubsteps = 10000, maximumCellVisits = 100000000;
    static constexpr std::size_t MaximumSubsteps = 1000000, MaximumCellVisits = 1000000000;
    void Validate() const;
};
struct EulerGasState {
    std::vector<double> density, momentumX, momentumY, totalEnergy;
};
struct EulerGasPrimitives {
    std::vector<double> velocityX, velocityY, pressure, soundSpeed, internalEnergy;
};
struct EulerGasSummary {
    // Area integrals per unit depth: kg/m, kg/s and J/m in SI.
    double mass = 0, momentumX = 0, momentumY = 0, totalEnergy = 0;
    double absoluteMomentumX = 0, absoluteMomentumY = 0, internalEnergy = 0, kineticEnergy = 0;
    double minimumDensity = 0, maximumDensity = 0, minimumPressure = 0, maximumPressure = 0;
};
struct EulerGasDiagnostics {
    EulerGasSummary initial, final;
    double massDefect = 0, momentumXDefect = 0, momentumYDefect = 0, totalEnergyDefect = 0;
    double massRoundoffAllowance = 0, momentumXRoundoffAllowance = 0;
    double momentumYRoundoffAllowance = 0, totalEnergyRoundoffAllowance = 0;
    double duration = 0, timeBefore = 0, timeAfter = 0, lastSubstep = 0;
    double maximumSignalSpeedX = 0, maximumSignalSpeedY = 0, maximumCfl = 0;
    std::size_t substeps = 0, cellVisits = 0;
    bool zeroDurationNoOp = false;
};
// Independent first-order unsplit ideal-gas Euler finite-volume solver.
// Immutable homogeneous geometry/medium, periodic boundaries, no sources.
// Row-major cell averages i+columns*j, centers ((i+.5)dx,(j+.5)dy).
// rho: kg/m^3; momenta: kg/(m^2 s); total/internal energy and pressure: J/m^3.
// Snapshots own their arrays. Failed steps preserve fields, clock and diagnostics.
class PeriodicEulerGasGrid {
  public:
    explicit PeriodicEulerGasGrid(const EulerGasGridConfig &config = {});
    EulerGasGridConfig config() const { return config_; }
    EulerGasState state() const { return state_; }
    EulerGasPrimitives primitives() const;
    EulerGasDiagnostics lastStep() const { return diagnostics_; }
    double time() const noexcept { return time_; }
    void setState(const EulerGasState &state);
    EulerGasDiagnostics step(double duration, const EulerGasStepConfig &options = {});

  private:
    EulerGasGridConfig config_;
    EulerGasState state_;
    EulerGasDiagnostics diagnostics_;
    double time_ = 0;
};
} // namespace PhysicsEngine
