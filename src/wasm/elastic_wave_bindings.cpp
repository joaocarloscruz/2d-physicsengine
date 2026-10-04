#include "checked_indices.h"
#include "physics/core/elastic_wave_grid.h"
#include <cmath>
#include <emscripten/bind.h>
#include <stdexcept>
#include <string>

namespace PhysicsEngine::Wasm {
namespace {
struct ElasticConfig {
    double columns, rows, spacingX, spacingY, density, lambda, shearModulus;
    double cflSafety, maxSubstep, maximumSubsteps, maximumCellVisits;
};
struct ElasticRates {
    emscripten::val accelerationX, accelerationY, strainRateXX, strainRateYY, engineeringShearRate;
};
struct ElasticState {
    emscripten::val vx, vy, sigmaXX, sigmaYY, sigmaXY;
};
std::size_t BoundedCount(double value, std::size_t maximum) {
    if (!std::isfinite(value) || value <= 0 || value > double(maximum))
        throw std::invalid_argument("Elastic wave count exceeds its positive hard bounds.");
    return Count(value);
}
ElasticWaveGridConfig Native(ElasticConfig c) {
    return {BoundedCount(c.columns, ElasticWaveGridConfig::MaximumCells),
            BoundedCount(c.rows, ElasticWaveGridConfig::MaximumCells),
            c.spacingX,
            c.spacingY,
            c.density,
            c.lambda,
            c.shearModulus,
            c.cflSafety,
            c.maxSubstep,
            BoundedCount(c.maximumSubsteps, ElasticWaveGridConfig::MaximumSubsteps),
            BoundedCount(c.maximumCellVisits, ElasticWaveGridConfig::MaximumCellVisits)};
}
ElasticWaveGrid *CreateDefault() { return new ElasticWaveGrid(); }
ElasticWaveGrid *CreateConfigured(ElasticConfig c) { return new ElasticWaveGrid(Native(c)); }
ElasticConfig Config(const ElasticWaveGrid &grid) {
    const auto c = grid.getConfig();
    return {double(c.columns),
            double(c.rows),
            c.spacingX,
            c.spacingY,
            c.density,
            c.lambda,
            c.shearModulus,
            c.cflSafety,
            c.maxSubstep,
            double(c.maximumSubsteps),
            double(c.maximumCellVisits)};
}
emscripten::val Copy(const std::vector<double> &values) {
    auto result = emscripten::val::array();
    for (std::size_t i = 0; i < values.size(); ++i)
        result.set(i, values[i]);
    return result;
}
ElasticState State(const ElasticWaveGrid &grid) {
    const auto s = grid.getState();
    return {Copy(s.vx), Copy(s.vy), Copy(s.sigmaXX), Copy(s.sigmaYY), Copy(s.sigmaXY)};
}
ElasticRates Rates(const ElasticWaveGrid &grid) {
    const auto r = grid.getSpatialRates();
    return {Copy(r.accelerationX), Copy(r.accelerationY), Copy(r.strainRateXX),
            Copy(r.strainRateYY), Copy(r.engineeringShearRate)};
}
void SetState(ElasticWaveGrid &grid, const ElasticState &state) {
    using emscripten::val;
    const auto array = val::global("Array");
    for (const auto *values :
         {&state.vx, &state.vy, &state.sigmaXX, &state.sigmaYY, &state.sigmaXY})
        if (!array.call<bool>("isArray", *values))
            throw std::invalid_argument("Elastic wave fields require plain JavaScript arrays.");
    const auto c = grid.getConfig();
    const auto count = c.columns * c.rows;
    // Check all complete shapes before allocating or copying any field.
    for (const auto *values :
         {&state.vx, &state.vy, &state.sigmaXX, &state.sigmaYY, &state.sigmaXY})
        if (Count((*values)["length"].as<double>()) != count)
            throw std::invalid_argument("Elastic wave field array length does not match the grid.");
    ElasticWaveState staged;
    for (auto *values : {&staged.vx, &staged.vy, &staged.sigmaXX, &staged.sigmaYY, &staged.sigmaXY})
        values->resize(count);
    const auto owns = val::global("Object")["prototype"]["hasOwnProperty"];
    const val *inputs[] = {&state.vx, &state.vy, &state.sigmaXX, &state.sigmaYY, &state.sigmaXY};
    std::vector<double> *outputs[] = {&staged.vx, &staged.vy, &staged.sigmaXX, &staged.sigmaYY,
                                      &staged.sigmaXY};
    for (std::size_t field = 0; field < 5; ++field)
        for (std::size_t i = 0; i < count; ++i) {
            const auto &values = *inputs[field];
            if (!owns.call<bool>("call", values, val(i)))
                throw std::invalid_argument("Elastic wave field arrays must be dense.");
            const auto value = values[i];
            if (value.typeOf().as<std::string>() != "number")
                throw std::invalid_argument("Elastic wave field entries must be numbers.");
            const double number = value.as<double>();
            if (!std::isfinite(number))
                throw std::invalid_argument("Elastic wave field entries must be finite.");
            (*outputs[field])[i] = number;
        }
    grid.setState(staged);
}
} // namespace
} // namespace PhysicsEngine::Wasm

EMSCRIPTEN_BINDINGS(elastic_wave_grid) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    value_object<ElasticConfig>("ElasticWaveGridConfig")
        .field("columns", &ElasticConfig::columns)
        .field("rows", &ElasticConfig::rows)
        .field("spacingX", &ElasticConfig::spacingX)
        .field("spacingY", &ElasticConfig::spacingY)
        .field("density", &ElasticConfig::density)
        .field("lambda", &ElasticConfig::lambda)
        .field("shearModulus", &ElasticConfig::shearModulus)
        .field("cflSafety", &ElasticConfig::cflSafety)
        .field("maxSubstep", &ElasticConfig::maxSubstep)
        .field("maximumSubsteps", &ElasticConfig::maximumSubsteps)
        .field("maximumCellVisits", &ElasticConfig::maximumCellVisits);
    value_object<ElasticState>("ElasticWaveState")
        .field("vx", &ElasticState::vx)
        .field("vy", &ElasticState::vy)
        .field("sigmaXX", &ElasticState::sigmaXX)
        .field("sigmaYY", &ElasticState::sigmaYY)
        .field("sigmaXY", &ElasticState::sigmaXY);
    value_object<ElasticWaveDiagnostics>("ElasticWaveDiagnostics")
        .field("kineticEnergy", &ElasticWaveDiagnostics::kineticEnergy)
        .field("strainEnergy", &ElasticWaveDiagnostics::strainEnergy)
        .field("totalEnergy", &ElasticWaveDiagnostics::totalEnergy)
        .field("modifiedEnergy", &ElasticWaveDiagnostics::modifiedEnergy)
        .field("modifiedEnergyStep", &ElasticWaveDiagnostics::modifiedEnergyStep)
        .field("physicalEnergyUpperBound", &ElasticWaveDiagnostics::physicalEnergyUpperBound)
        .field("meanVx", &ElasticWaveDiagnostics::meanVx)
        .field("meanVy", &ElasticWaveDiagnostics::meanVy)
        .field("meanSigmaXX", &ElasticWaveDiagnostics::meanSigmaXX)
        .field("meanSigmaYY", &ElasticWaveDiagnostics::meanSigmaYY)
        .field("meanSigmaXY", &ElasticWaveDiagnostics::meanSigmaXY)
        .field("maxAbsVelocity", &ElasticWaveDiagnostics::maxAbsVelocity)
        .field("maxAbsStress", &ElasticWaveDiagnostics::maxAbsStress)
        .field("meanSigmaZZ", &ElasticWaveDiagnostics::meanSigmaZZ)
        .field("maxAbsSigmaZZ", &ElasticWaveDiagnostics::maxAbsSigmaZZ)
        .field("compatibilityRms", &ElasticWaveDiagnostics::compatibilityRms)
        .field("maxAbsCompatibility", &ElasticWaveDiagnostics::maxAbsCompatibility)
        .field("time", &ElasticWaveDiagnostics::time)
        .field("stableTimeStep", &ElasticWaveDiagnostics::stableTimeStep)
        .field("lastSubstep", &ElasticWaveDiagnostics::lastSubstep)
        .field("lastSubsteps", &ElasticWaveDiagnostics::lastSubsteps)
        .field("lastCellVisits", &ElasticWaveDiagnostics::lastCellVisits);
    value_object<ElasticRates>("ElasticWaveSpatialRates")
        .field("accelerationX", &ElasticRates::accelerationX)
        .field("accelerationY", &ElasticRates::accelerationY)
        .field("strainRateXX", &ElasticRates::strainRateXX)
        .field("strainRateYY", &ElasticRates::strainRateYY)
        .field("engineeringShearRate", &ElasticRates::engineeringShearRate);
    class_<ElasticWaveGrid>("ElasticWaveGrid")
        .constructor(&CreateDefault, allow_raw_pointers())
        .constructor(&CreateConfigured, allow_raw_pointers())
        .function("getConfig", &Config)
        .function("getState", &State)
        .function("getDiagnostics", &ElasticWaveGrid::getDiagnostics)
        .function("getCompatibility", optional_override([](const ElasticWaveGrid &grid) {
                      return Copy(grid.getCompatibility());
                  }))
        .function("getOutOfPlaneStress", optional_override([](const ElasticWaveGrid &grid) {
                      return Copy(grid.getOutOfPlaneStress());
                  }))
        .function("getSpatialRates", &Rates)
        .function("getCompressionalSpeed", &ElasticWaveGrid::getCompressionalSpeed)
        .function("getShearSpeed", &ElasticWaveGrid::getShearSpeed)
        .function("getStableTimeStep", &ElasticWaveGrid::getStableTimeStep)
        .function("getModifiedEnergy", &ElasticWaveGrid::getModifiedEnergy)
        .function("setState", &SetState)
        .function("step", &ElasticWaveGrid::step);
}
