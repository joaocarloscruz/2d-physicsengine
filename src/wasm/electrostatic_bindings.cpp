#include "checked_indices.h"
#include "physics/core/periodic_electrostatic_grid.h"
#include <cmath>
#include <emscripten/bind.h>
#include <stdexcept>
#include <string>

namespace PhysicsEngine::Wasm {
namespace {
struct ElectrostaticConfig {
    double columns, rows, spacingX, spacingY, permittivity;
};
struct ElectrostaticOptions {
    double absoluteGaussTolerance, relativeGaussTolerance, maximumIterations, maximumCellVisits;
};
struct ElectrostaticFaces {
    emscripten::val xFaces, yFaces;
};
struct ElectrostaticCopy {
    emscripten::val originalCharge, effectiveCharge, potential;
    ElectrostaticFaces field;
    emscripten::val gaussResidual, curl;
    ElectrostaticDiagnostics diagnostics;
};
std::size_t BoundedCount(double value, std::size_t minimum, std::size_t maximum) {
    if (!std::isfinite(value) || value < double(minimum) || value > double(maximum))
        throw std::invalid_argument("Electrostatic count exceeds its hard bounds.");
    return Count(value);
}
ElectrostaticGridConfig Native(ElectrostaticConfig c) {
    return {BoundedCount(c.columns, 2, ElectrostaticGridConfig::MaximumCells),
            BoundedCount(c.rows, 2, ElectrostaticGridConfig::MaximumCells),
            c.spacingX, c.spacingY, c.permittivity};
}
ElectrostaticSolveConfig Native(ElectrostaticOptions c) {
    return {c.absoluteGaussTolerance, c.relativeGaussTolerance,
            BoundedCount(c.maximumIterations, 0, ElectrostaticSolveConfig::MaximumIterations),
            BoundedCount(c.maximumCellVisits, 0, ElectrostaticSolveConfig::MaximumCellVisits)};
}
PeriodicElectrostaticGrid* CreateDefault() { return new PeriodicElectrostaticGrid(); }
PeriodicElectrostaticGrid* CreateConfigured(ElectrostaticConfig c) {
    return new PeriodicElectrostaticGrid(Native(c));
}
ElectrostaticConfig Config(const PeriodicElectrostaticGrid& grid) {
    const auto c=grid.getConfig();
    return {double(c.columns), double(c.rows), c.spacingX, c.spacingY, c.permittivity};
}
emscripten::val Copy(const std::vector<double>& values) {
    auto result=emscripten::val::array();
    for(std::size_t i=0;i<values.size();++i)result.set(i,values[i]);
    return result;
}
ElectrostaticCopy Snapshot(const PeriodicElectrostaticGrid& grid) {
    const auto s=grid.getSnapshot();
    return {Copy(s.originalCharge), Copy(s.effectiveCharge), Copy(s.potential),
            {Copy(s.field.xFaces), Copy(s.field.yFaces)}, Copy(s.gaussResidual), Copy(s.curl), s.diagnostics};
}
ElectrostaticDiagnostics Solve(PeriodicElectrostaticGrid& grid, const emscripten::val& values,
                              const ElectrostaticSolveConfig& options) {
    using emscripten::val;
    if(!val::global("Array").call<bool>("isArray",values))
        throw std::invalid_argument("Electrostatic charge requires a JavaScript array.");
    const auto c=grid.getConfig();const auto count=c.columns*c.rows;
    if(Count(values["length"].as<double>())!=count)
        throw std::invalid_argument("Electrostatic charge length does not match the grid.");
    std::vector<double> charge(count);
    const auto owns=val::global("Object")["prototype"]["hasOwnProperty"];
    for(std::size_t i=0;i<count;++i) {
        if(!owns.call<bool>("call",values,val(i)))
            throw std::invalid_argument("Electrostatic charge arrays must be dense.");
        const auto value=values[i];
        if(value.typeOf().as<std::string>()!="number")
            throw std::invalid_argument("Electrostatic charge entries must be numbers.");
        charge[i]=value.as<double>();
        if(!std::isfinite(charge[i]))throw std::invalid_argument("Electrostatic charge must be finite.");
    }
    return grid.solve(charge,options);
}
} // namespace
} // namespace PhysicsEngine::Wasm

EMSCRIPTEN_BINDINGS(periodic_electrostatic_grid) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    value_object<ElectrostaticConfig>("ElectrostaticGridConfig")
        .field("columns",&ElectrostaticConfig::columns).field("rows",&ElectrostaticConfig::rows)
        .field("spacingX",&ElectrostaticConfig::spacingX).field("spacingY",&ElectrostaticConfig::spacingY)
        .field("permittivity",&ElectrostaticConfig::permittivity);
    value_object<ElectrostaticOptions>("ElectrostaticSolveConfig")
        .field("absoluteGaussTolerance",&ElectrostaticOptions::absoluteGaussTolerance)
        .field("relativeGaussTolerance",&ElectrostaticOptions::relativeGaussTolerance)
        .field("maximumIterations",&ElectrostaticOptions::maximumIterations)
        .field("maximumCellVisits",&ElectrostaticOptions::maximumCellVisits);
    value_object<ElectrostaticFaces>("ElectrostaticField")
        .field("xFaces",&ElectrostaticFaces::xFaces).field("yFaces",&ElectrostaticFaces::yFaces);
    value_object<ElectrostaticCopy>("ElectrostaticSnapshot")
        .field("originalCharge",&ElectrostaticCopy::originalCharge)
        .field("effectiveCharge",&ElectrostaticCopy::effectiveCharge)
        .field("potential",&ElectrostaticCopy::potential).field("field",&ElectrostaticCopy::field)
        .field("gaussResidual",&ElectrostaticCopy::gaussResidual).field("curl",&ElectrostaticCopy::curl)
        .field("diagnostics",&ElectrostaticCopy::diagnostics);
    value_object<ElectrostaticDiagnostics>("ElectrostaticDiagnostics")
        .field("iterations",&ElectrostaticDiagnostics::iterations).field("cellVisits",&ElectrostaticDiagnostics::cellVisits)
        .field("residualRestarts",&ElectrostaticDiagnostics::residualRestarts)
        .field("permittivity",&ElectrostaticDiagnostics::permittivity)
        .field("originalChargeMean",&ElectrostaticDiagnostics::originalChargeMean)
        .field("effectiveChargeMean",&ElectrostaticDiagnostics::effectiveChargeMean)
        .field("originalIntegratedCharge",&ElectrostaticDiagnostics::originalIntegratedCharge)
        .field("effectiveIntegratedCharge",&ElectrostaticDiagnostics::effectiveIntegratedCharge)
        .field("neutralityMeanAllowance",&ElectrostaticDiagnostics::neutralityMeanAllowance)
        .field("removedChargeMean",&ElectrostaticDiagnostics::removedChargeMean)
        .field("maximumSourceCorrection",&ElectrostaticDiagnostics::maximumSourceCorrection)
        .field("sourceCorrectionAllowance",&ElectrostaticDiagnostics::sourceCorrectionAllowance)
        .field("effectiveChargeRms",&ElectrostaticDiagnostics::effectiveChargeRms)
        .field("targetGaussRms",&ElectrostaticDiagnostics::targetGaussRms)
        .field("finalGaussRms",&ElectrostaticDiagnostics::finalGaussRms)
        .field("maximumAbsGauss",&ElectrostaticDiagnostics::maximumAbsGauss)
        .field("originalGaussRms",&ElectrostaticDiagnostics::originalGaussRms)
        .field("maximumAbsOriginalGauss",&ElectrostaticDiagnostics::maximumAbsOriginalGauss)
        .field("potentialMean",&ElectrostaticDiagnostics::potentialMean)
        .field("meanFieldX",&ElectrostaticDiagnostics::meanFieldX).field("meanFieldY",&ElectrostaticDiagnostics::meanFieldY)
        .field("curlRms",&ElectrostaticDiagnostics::curlRms).field("maximumAbsCurl",&ElectrostaticDiagnostics::maximumAbsCurl)
        .field("fieldEnergy",&ElectrostaticDiagnostics::fieldEnergy).field("sourceEnergy",&ElectrostaticDiagnostics::sourceEnergy)
        .field("residualEnergyCorrection",&ElectrostaticDiagnostics::residualEnergyCorrection)
        .field("residualEnergyBound",&ElectrostaticDiagnostics::residualEnergyBound)
        .field("energyIdentityError",&ElectrostaticDiagnostics::energyIdentityError)
        .field("roundoffEnergyAllowance",&ElectrostaticDiagnostics::roundoffEnergyAllowance)
        .field("hasSolution",&ElectrostaticDiagnostics::hasSolution).field("zeroSource",&ElectrostaticDiagnostics::zeroSource);
    class_<PeriodicElectrostaticGrid>("PeriodicElectrostaticGrid")
        .constructor(&CreateDefault,allow_raw_pointers()).constructor(&CreateConfigured,allow_raw_pointers())
        .function("getConfig",&Config).function("getSnapshot",&Snapshot)
        .function("solve",optional_override([](PeriodicElectrostaticGrid& grid,const val& source) {
            return Solve(grid,source,{});
        }))
        .function("solve",optional_override([](PeriodicElectrostaticGrid& grid,const val& source,ElectrostaticOptions options) {
            return Solve(grid,source,Native(options));
        }));
}
