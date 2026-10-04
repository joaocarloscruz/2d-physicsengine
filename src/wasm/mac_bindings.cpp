#include "checked_indices.h"
#include "physics/core/fluids/periodic_mac_grid.h"
#include <cmath>
#include <emscripten/bind.h>
#include <stdexcept>
#include <string>

namespace PhysicsEngine::Wasm {
namespace {
struct MacGridConfig {
    double columns, rows, spacingX, spacingY;
};
struct MacOptions {
    double density, timeStep, absoluteDivergenceTolerance, relativeDivergenceTolerance;
    double maximumIterations, maximumCellVisits;
};
struct MacVelocities {
    emscripten::val xFaces, yFaces;
};
struct MacSnapshot {
    emscripten::val potential, pressure;
    MacProjectionDiagnostics diagnostics;
};
PeriodicMacGridConfig Native(MacGridConfig c) {
    return {Count(c.columns), Count(c.rows), c.spacingX, c.spacingY};
}
MacProjectionConfig Native(MacOptions c) {
    return {c.density,
            c.timeStep,
            c.absoluteDivergenceTolerance,
            c.relativeDivergenceTolerance,
            Count(c.maximumIterations),
            Count(c.maximumCellVisits)};
}
PeriodicMacGrid *CreateDefault() {
    return new PeriodicMacGrid();
}
PeriodicMacGrid *CreateConfigured(MacGridConfig c) {
    return new PeriodicMacGrid(Native(c));
}
MacGridConfig Config(const PeriodicMacGrid &grid) {
    const auto c = grid.config();
    return {double(c.columns), double(c.rows), c.spacingX, c.spacingY};
}
emscripten::val Copy(const std::vector<double> &values) {
    auto result = emscripten::val::array();
    for (std::size_t i = 0; i < values.size(); ++i)
        result.set(i, values[i]);
    return result;
}
MacVelocities Velocities(const PeriodicMacGrid &grid) {
    const auto v = grid.velocities();
    return {Copy(v.xFaces), Copy(v.yFaces)};
}
MacSnapshot Projection(const PeriodicMacGrid &grid) {
    const auto p = grid.lastProjection();
    return {Copy(p.potential), Copy(p.pressure), p.diagnostics};
}
void SetVelocities(PeriodicMacGrid &grid, const emscripten::val &x, const emscripten::val &y) {
    using emscripten::val;
    const auto array = val::global("Array");
    if (!array.call<bool>("isArray", x) || !array.call<bool>("isArray", y))
        throw std::invalid_argument("MAC velocities require plain JavaScript arrays.");
    const auto c = grid.config();
    const auto count = c.columns * c.rows;
    // Reject either shape before allocating or reading either native copy.
    if (Count(x["length"].as<double>()) != count || Count(y["length"].as<double>()) != count)
        throw std::invalid_argument("MAC face array length does not match the grid.");
    MacVelocityState staged;
    staged.xFaces.resize(count);
    staged.yFaces.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        const auto xi = x[i], yi = y[i];
        if (xi.typeOf().as<std::string>() != "number" || yi.typeOf().as<std::string>() != "number")
            throw std::invalid_argument("MAC velocity entries must be numbers.");
        staged.xFaces[i] = xi.as<double>();
        staged.yFaces[i] = yi.as<double>();
        if (!std::isfinite(staged.xFaces[i]) || !std::isfinite(staged.yFaces[i]))
            throw std::invalid_argument("MAC velocity entries must be finite.");
    }
    grid.setVelocities(staged);
}
} // namespace
} // namespace PhysicsEngine::Wasm

EMSCRIPTEN_BINDINGS(periodic_mac_grid) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    value_object<MacGridConfig>("PeriodicMacGridConfig")
        .field("columns", &MacGridConfig::columns)
        .field("rows", &MacGridConfig::rows)
        .field("spacingX", &MacGridConfig::spacingX)
        .field("spacingY", &MacGridConfig::spacingY);
    value_object<MacOptions>("MacProjectionConfig")
        .field("density", &MacOptions::density)
        .field("timeStep", &MacOptions::timeStep)
        .field("absoluteDivergenceTolerance", &MacOptions::absoluteDivergenceTolerance)
        .field("relativeDivergenceTolerance", &MacOptions::relativeDivergenceTolerance)
        .field("maximumIterations", &MacOptions::maximumIterations)
        .field("maximumCellVisits", &MacOptions::maximumCellVisits);
    value_object<MacVelocities>("MacVelocityState")
        .field("xFaces", &MacVelocities::xFaces)
        .field("yFaces", &MacVelocities::yFaces);
    value_object<MacProjectionDiagnostics>("MacProjectionDiagnostics")
        .field("iterations", &MacProjectionDiagnostics::iterations)
        .field("cellVisits", &MacProjectionDiagnostics::cellVisits)
        .field("density", &MacProjectionDiagnostics::density)
        .field("timeStep", &MacProjectionDiagnostics::timeStep)
        .field("initialDivergenceRms", &MacProjectionDiagnostics::initialDivergenceRms)
        .field("finalDivergenceRms", &MacProjectionDiagnostics::finalDivergenceRms)
        .field("targetDivergenceRms", &MacProjectionDiagnostics::targetDivergenceRms)
        .field("removedDivergenceMean", &MacProjectionDiagnostics::removedDivergenceMean)
        .field("potentialMean", &MacProjectionDiagnostics::potentialMean)
        .field("pressureMean", &MacProjectionDiagnostics::pressureMean)
        .field("initialMeanX", &MacProjectionDiagnostics::initialMeanX)
        .field("initialMeanY", &MacProjectionDiagnostics::initialMeanY)
        .field("finalMeanX", &MacProjectionDiagnostics::finalMeanX)
        .field("finalMeanY", &MacProjectionDiagnostics::finalMeanY)
        .field("initialKineticEnergy", &MacProjectionDiagnostics::initialKineticEnergy)
        .field("finalKineticEnergy", &MacProjectionDiagnostics::finalKineticEnergy)
        .field("correctionKineticEnergy", &MacProjectionDiagnostics::correctionKineticEnergy)
        .field("velocityCorrectionInnerProduct",
               &MacProjectionDiagnostics::velocityCorrectionInnerProduct)
        .field("divergencePotentialInnerProduct",
               &MacProjectionDiagnostics::divergencePotentialInnerProduct)
        .field("residualEnergyBound", &MacProjectionDiagnostics::residualEnergyBound)
        .field("storageEnergyError", &MacProjectionDiagnostics::storageEnergyError)
        .field("roundoffEnergyAllowance", &MacProjectionDiagnostics::roundoffEnergyAllowance)
        .field("zeroDivergenceNoOp", &MacProjectionDiagnostics::zeroDivergenceNoOp);
    value_object<MacSnapshot>("MacProjectionSnapshot")
        .field("potential", &MacSnapshot::potential)
        .field("pressure", &MacSnapshot::pressure)
        .field("diagnostics", &MacSnapshot::diagnostics);
    class_<PeriodicMacGrid>("PeriodicMacGrid")
        .constructor(&CreateDefault, allow_raw_pointers())
        .constructor(&CreateConfigured, allow_raw_pointers())
        .function("getConfig", &Config)
        .function("getVelocities", &Velocities)
        .function("getLastProjection", &Projection)
        .function("getDivergence", optional_override([](const PeriodicMacGrid &grid) {
                      return Copy(grid.divergence());
                  }))
        .function("setVelocities", &SetVelocities)
        .function("project",
                  optional_override([](PeriodicMacGrid &grid) { return grid.project(); }))
        .function("project", optional_override([](PeriodicMacGrid &grid, MacOptions c) {
                      return grid.project(Native(c));
                  }));
}
