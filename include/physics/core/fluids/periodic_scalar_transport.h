#pragma once
#include "physics/core/fluids/periodic_mac_grid.h"
#include <cstddef>
#include <vector>

namespace PhysicsEngine {
struct PeriodicScalarGridConfig {
    std::size_t columns=16, rows=16;
    double spacingX=1, spacingY=1;
    static constexpr std::size_t MaximumCells=262144;
    void Validate() const;
};
struct ScalarTransportConfig {
    double cflSafety=.9, maxSubstep=.1;
    std::size_t maximumSubsteps=10000, maximumCellVisits=100000000;
    static constexpr std::size_t MaximumSubsteps=1000000, MaximumCellVisits=1000000000;
    void Validate() const;
};
struct ScalarTransportDiagnostics {
    std::size_t substeps=0, cellVisits=0;
    double duration=0, timeBefore=0, timeAfter=0, lastSubstep=0;
    double maximumOutflowRate=0, outflowRateBound=0, maximumAbsDivergence=0, maximumCfl=0;
    double initialIntegratedScalar=0, finalIntegratedScalar=0;
    double initialAbsoluteIntegral=0, finalAbsoluteIntegral=0;
    double integratedScalarDrift=0, conservationRoundoffAllowance=0;
    double initialMinimum=0, initialMaximum=0, finalMinimum=0, finalMaximum=0;
    double rangeRoundoffAllowance=0;
    bool nonnegativeInput=false, discreteDivergenceFree=false, zeroDurationNoOp=false;
};
// Cell averages q obey q_t + div(u q)=0. First-order unsplit donor-cell
// transport on a periodic grid; prescribed face velocities remain frozen.
// No velocity advection, forces, automatic projection/coupling or shared clock.
// Row-major scalar index i+columns*j, centers ((i+.5)dx,(j+.5)dy).
// Faces use MacVelocityState's layout; every public snapshot owns its values.
class PeriodicScalarTransport {
public:
    explicit PeriodicScalarTransport(const PeriodicScalarGridConfig& config={});
    PeriodicScalarGridConfig config() const { return config_; }
    std::vector<double> state() const { return scalar_; }
    MacVelocityState velocities() const { return velocities_; }
    double time() const noexcept { return time_; }
    ScalarTransportDiagnostics lastStep() const { return diagnostics_; }
    void setState(const std::vector<double>& scalar);
    void setVelocities(const MacVelocityState& velocities);
    ScalarTransportDiagnostics step(double duration, const ScalarTransportConfig& config={});
private:
    PeriodicScalarGridConfig config_;
    std::vector<double> scalar_;
    MacVelocityState velocities_;
    double time_=0;
    ScalarTransportDiagnostics diagnostics_;
};
} // namespace PhysicsEngine
