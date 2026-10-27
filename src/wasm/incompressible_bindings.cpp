#include "mac_shared.h"
#include "physics/core/fluids/periodic_incompressible_grid.h"
#include <cmath>
#include <emscripten/bind.h>
#include <stdexcept>
#include <string>

namespace PhysicsEngine::Wasm {
namespace {
struct IncompressibleConfig {
    MacGridConfig geometry;
    double density, kinematicViscosity;
};
// Integer limits stay JavaScript numbers until checked against hard bounds.
struct IncompressibleOptions {
    double cflSafety;
    double absoluteDivergenceTolerance, relativeDivergenceTolerance;
    double absoluteVelocityTolerance, relativeVelocityTolerance;
    double maximumSubsteps, maximumIterations, maximumCellVisits;
};
std::size_t BoundedCount(double value, std::size_t maximum) {
    if (!std::isfinite(value) || value < 0 || value > double(maximum))
        throw std::invalid_argument("Incompressible work limit exceeds its hard bound.");
    return Count(value);
}
PeriodicIncompressibleGridConfig Native(IncompressibleConfig c) {
    PeriodicIncompressibleGridConfig result;
    result.geometry = Native(c.geometry);
    result.density = c.density;
    result.kinematicViscosity = c.kinematicViscosity;
    return result;
}
IncompressibleStepConfig Native(IncompressibleOptions c) {
    IncompressibleStepConfig result;
    result.cflSafety = c.cflSafety;
    result.absoluteDivergenceTolerance = c.absoluteDivergenceTolerance;
    result.relativeDivergenceTolerance = c.relativeDivergenceTolerance;
    result.absoluteVelocityTolerance = c.absoluteVelocityTolerance;
    result.relativeVelocityTolerance = c.relativeVelocityTolerance;
    result.maximumSubsteps = BoundedCount(c.maximumSubsteps, IncompressibleStepConfig::MaximumSubsteps);
    result.maximumIterations = BoundedCount(c.maximumIterations, IncompressibleStepConfig::MaximumIterations);
    result.maximumCellVisits = BoundedCount(c.maximumCellVisits, IncompressibleStepConfig::MaximumCellVisits);
    return result;
}
PeriodicIncompressibleGrid *CreateDefault() { return new PeriodicIncompressibleGrid(); }
PeriodicIncompressibleGrid *CreateConfigured(IncompressibleConfig c) {
    return new PeriodicIncompressibleGrid(Native(c));
}
IncompressibleConfig Config(const PeriodicIncompressibleGrid &grid) {
    const auto c = grid.config();
    return {Public(c.geometry), c.density, c.kinematicViscosity};
}
// The JavaScript boundary has already copied both arrays into dense, own,
// finite number entries of the captured native length; recheck defensively.
void SetVelocities(PeriodicIncompressibleGrid &grid, const emscripten::val &x, const emscripten::val &y) {
    using emscripten::val;
    const auto array = val::global("Array");
    if (!array.call<bool>("isArray", x) || !array.call<bool>("isArray", y))
        throw std::invalid_argument("Incompressible velocities require plain JavaScript arrays.");
    const auto g = grid.config().geometry;
    const auto count = g.columns * g.rows;
    if (Count(x["length"].as<double>()) != count || Count(y["length"].as<double>()) != count)
        throw std::invalid_argument("Incompressible face array length does not match the grid.");
    MacVelocityState staged;
    staged.xFaces.resize(count);
    staged.yFaces.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        const auto xi = x[i], yi = y[i];
        if (xi.typeOf().as<std::string>() != "number" || yi.typeOf().as<std::string>() != "number")
            throw std::invalid_argument("Incompressible velocity entries must be numbers.");
        staged.xFaces[i] = xi.as<double>();
        staged.yFaces[i] = yi.as<double>();
    }
    grid.setVelocities(staged);
}
emscripten::val History(const IncompressibleStepDiagnostics &d) {
    auto result = emscripten::val::array();
    for (std::size_t i = 0; i < d.history.size(); ++i)
        result.set(i, emscripten::val(d.history[i]));
    return result;
}
void RejectHistory(IncompressibleStepDiagnostics &, emscripten::val) {
    throw std::invalid_argument("Incompressible step reports are output-only.");
}
} // namespace
} // namespace PhysicsEngine::Wasm

EMSCRIPTEN_BINDINGS(periodic_incompressible_grid) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    value_object<IncompressibleConfig>("PeriodicIncompressibleGridConfig")
        .field("geometry", &IncompressibleConfig::geometry)
        .field("density", &IncompressibleConfig::density)
        .field("kinematicViscosity", &IncompressibleConfig::kinematicViscosity);
    value_object<IncompressibleOptions>("IncompressibleStepConfig")
        .field("cflSafety", &IncompressibleOptions::cflSafety)
        .field("absoluteDivergenceTolerance", &IncompressibleOptions::absoluteDivergenceTolerance)
        .field("relativeDivergenceTolerance", &IncompressibleOptions::relativeDivergenceTolerance)
        .field("absoluteVelocityTolerance", &IncompressibleOptions::absoluteVelocityTolerance)
        .field("relativeVelocityTolerance", &IncompressibleOptions::relativeVelocityTolerance)
        .field("maximumSubsteps", &IncompressibleOptions::maximumSubsteps)
        .field("maximumIterations", &IncompressibleOptions::maximumIterations)
        .field("maximumCellVisits", &IncompressibleOptions::maximumCellVisits);
    value_object<IncompressibleSubstepDiagnostics>("IncompressibleSubstepDiagnostics")
        .field("timeStep", &IncompressibleSubstepDiagnostics::timeStep)
        .field("outgoingCfl", &IncompressibleSubstepDiagnostics::outgoingCfl)
        .field("maximumDualDivergence", &IncompressibleSubstepDiagnostics::maximumDualDivergence)
        .field("maximumRowSum", &IncompressibleSubstepDiagnostics::maximumRowSum)
        .field("initialKineticEnergy", &IncompressibleSubstepDiagnostics::initialKineticEnergy)
        .field("advectedKineticEnergy", &IncompressibleSubstepDiagnostics::advectedKineticEnergy)
        .field("dualDivergenceWork", &IncompressibleSubstepDiagnostics::dualDivergenceWork)
        .field("donorDissipation", &IncompressibleSubstepDiagnostics::donorDissipation)
        .field("forwardEulerIncrementEnergy", &IncompressibleSubstepDiagnostics::forwardEulerIncrementEnergy)
        .field("advectionEnergyChange", &IncompressibleSubstepDiagnostics::advectionEnergyChange)
        .field("advectionStorageError", &IncompressibleSubstepDiagnostics::advectionStorageError)
        .field("advectionEnergyBound", &IncompressibleSubstepDiagnostics::advectionEnergyBound)
        .field("roundoffEnergyAllowance", &IncompressibleSubstepDiagnostics::roundoffEnergyAllowance)
        .field("meanRoundoffAllowanceX", &IncompressibleSubstepDiagnostics::meanRoundoffAllowanceX)
        .field("meanRoundoffAllowanceY", &IncompressibleSubstepDiagnostics::meanRoundoffAllowanceY)
        .field("diffusion", &IncompressibleSubstepDiagnostics::diffusion)
        .field("projection", &IncompressibleSubstepDiagnostics::projection);
    value_object<IncompressibleStepDiagnostics>("IncompressibleStepDiagnostics")
        .field("timeStep", &IncompressibleStepDiagnostics::timeStep)
        .field("initialTime", &IncompressibleStepDiagnostics::initialTime)
        .field("finalTime", &IncompressibleStepDiagnostics::finalTime)
        .field("substeps", &IncompressibleStepDiagnostics::substeps)
        .field("iterations", &IncompressibleStepDiagnostics::iterations)
        .field("cellVisits", &IncompressibleStepDiagnostics::cellVisits)
        .field("zeroStepNoOp", &IncompressibleStepDiagnostics::zeroStepNoOp)
        .field("initialMeanX", &IncompressibleStepDiagnostics::initialMeanX)
        .field("initialMeanY", &IncompressibleStepDiagnostics::initialMeanY)
        .field("finalMeanX", &IncompressibleStepDiagnostics::finalMeanX)
        .field("finalMeanY", &IncompressibleStepDiagnostics::finalMeanY)
        .field("meanRoundoffAllowanceX", &IncompressibleStepDiagnostics::meanRoundoffAllowanceX)
        .field("meanRoundoffAllowanceY", &IncompressibleStepDiagnostics::meanRoundoffAllowanceY)
        .field("initialKineticEnergy", &IncompressibleStepDiagnostics::initialKineticEnergy)
        .field("finalKineticEnergy", &IncompressibleStepDiagnostics::finalKineticEnergy)
        .field("storedEnergyChange", &IncompressibleStepDiagnostics::storedEnergyChange)
        .field("dualDivergenceWork", &IncompressibleStepDiagnostics::dualDivergenceWork)
        .field("donorDissipation", &IncompressibleStepDiagnostics::donorDissipation)
        .field("forwardEulerIncrementEnergy", &IncompressibleStepDiagnostics::forwardEulerIncrementEnergy)
        .field("viscousDissipation", &IncompressibleStepDiagnostics::viscousDissipation)
        .field("viscousIncrementEnergy", &IncompressibleStepDiagnostics::viscousIncrementEnergy)
        .field("viscousResidualWork", &IncompressibleStepDiagnostics::viscousResidualWork)
        .field("projectionCorrectionEnergy", &IncompressibleStepDiagnostics::projectionCorrectionEnergy)
        .field("projectionResidualWork", &IncompressibleStepDiagnostics::projectionResidualWork)
        .field("residualEnergyBound", &IncompressibleStepDiagnostics::residualEnergyBound)
        .field("roundoffEnergyAllowance", &IncompressibleStepDiagnostics::roundoffEnergyAllowance)
        .field("storageEnergyError", &IncompressibleStepDiagnostics::storageEnergyError)
        .field("initialProjection", &IncompressibleStepDiagnostics::initialProjection)
        .field("history", &History, &RejectHistory);
    class_<PeriodicIncompressibleGrid>("PeriodicIncompressibleGrid")
        .constructor(&CreateDefault, allow_raw_pointers())
        .constructor(&CreateConfigured, allow_raw_pointers())
        .function("config", &Config)
        .function("velocities", optional_override([](const PeriodicIncompressibleGrid &grid) {
                      return Public(grid.velocities());
                  }))
        .function("lastProjection", optional_override([](const PeriodicIncompressibleGrid &grid) {
                      return Public(grid.lastProjection());
                  }))
        .function("lastDiffusion", &PeriodicIncompressibleGrid::lastDiffusion)
        .function("lastStep", &PeriodicIncompressibleGrid::lastStep)
        .function("time", &PeriodicIncompressibleGrid::time)
        .function("divergence", optional_override([](const PeriodicIncompressibleGrid &grid) {
                      return Copy(grid.divergence());
                  }))
        .function("setVelocities", &SetVelocities)
        .function("step", optional_override([](PeriodicIncompressibleGrid &grid, double duration) {
                      return grid.step(duration);
                  }))
        .function("step", optional_override([](PeriodicIncompressibleGrid &grid, double duration,
                                               IncompressibleOptions options) {
                      return grid.step(duration, Native(options));
                  }));
}
