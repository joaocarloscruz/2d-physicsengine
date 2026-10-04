#include "checked_indices.h"
#include "physics/core/maxwell_grid.h"
#include <cmath>
#include <emscripten/bind.h>
#include <stdexcept>
#include <string>

namespace PhysicsEngine::Wasm {
namespace {
struct MaxwellConfig {
    double columns, rows, spacingX, spacingY, permittivity, permeability;
    double cflSafety, maxSubstep, maximumSubsteps, maximumCellVisits;
};
struct MaxwellState {
    emscripten::val ez, hx, hy;
};
std::size_t BoundedCount(double value, std::size_t maximum) {
    if (!std::isfinite(value) || value <= 0 || value > double(maximum))
        throw std::invalid_argument("Maxwell count exceeds its positive hard bounds.");
    return Count(value);
}
MaxwellGridConfig Native(MaxwellConfig c) {
    return {BoundedCount(c.columns, MaxwellGridConfig::MaximumCells),
            BoundedCount(c.rows, MaxwellGridConfig::MaximumCells),
            c.spacingX,
            c.spacingY,
            c.permittivity,
            c.permeability,
            c.cflSafety,
            c.maxSubstep,
            BoundedCount(c.maximumSubsteps, MaxwellGridConfig::MaximumSubsteps),
            BoundedCount(c.maximumCellVisits, MaxwellGridConfig::MaximumCellVisits)};
}
MaxwellGrid *CreateDefault() {
    return new MaxwellGrid();
}
MaxwellGrid *CreateConfigured(MaxwellConfig c) {
    return new MaxwellGrid(Native(c));
}
MaxwellConfig Config(const MaxwellGrid &grid) {
    const auto c = grid.getConfig();
    return {double(c.columns),
            double(c.rows),
            c.spacingX,
            c.spacingY,
            c.permittivity,
            c.permeability,
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
MaxwellState State(const MaxwellGrid &grid) {
    const auto s = grid.getState();
    return {Copy(s.ez), Copy(s.hx), Copy(s.hy)};
}
void SetState(MaxwellGrid &grid, const MaxwellState &state) {
    using emscripten::val;
    const auto array = val::global("Array");
    for (const auto *values : {&state.ez, &state.hx, &state.hy})
        if (!array.call<bool>("isArray", *values))
            throw std::invalid_argument("Maxwell fields require plain JavaScript arrays.");
    const auto c = grid.getConfig();
    const auto count = c.columns * c.rows;
    // Check all complete shapes before allocating or copying any field.
    for (const auto *values : {&state.ez, &state.hx, &state.hy})
        if (Count((*values)["length"].as<double>()) != count)
            throw std::invalid_argument("Maxwell field array length does not match the grid.");
    MaxwellFieldState staged;
    staged.ez.resize(count);
    staged.hx.resize(count);
    staged.hy.resize(count);
    const auto owns = val::global("Object")["prototype"]["hasOwnProperty"];
    const val *inputs[] = {&state.ez, &state.hx, &state.hy};
    std::vector<double> *outputs[] = {&staged.ez, &staged.hx, &staged.hy};
    for (std::size_t field = 0; field < 3; ++field)
        for (std::size_t i = 0; i < count; ++i) {
            const auto &values = *inputs[field];
            if (!owns.call<bool>("call", values, val(i)))
                throw std::invalid_argument("Maxwell field arrays must be dense.");
            const auto value = values[i];
            if (value.typeOf().as<std::string>() != "number")
                throw std::invalid_argument("Maxwell field entries must be numbers.");
            const double number = value.as<double>();
            if (!std::isfinite(number))
                throw std::invalid_argument("Maxwell field entries must be finite.");
            (*outputs[field])[i] = number;
        }
    grid.setState(staged);
}
} // namespace
} // namespace PhysicsEngine::Wasm

EMSCRIPTEN_BINDINGS(maxwell_grid) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    value_object<MaxwellConfig>("MaxwellGridConfig")
        .field("columns", &MaxwellConfig::columns)
        .field("rows", &MaxwellConfig::rows)
        .field("spacingX", &MaxwellConfig::spacingX)
        .field("spacingY", &MaxwellConfig::spacingY)
        .field("permittivity", &MaxwellConfig::permittivity)
        .field("permeability", &MaxwellConfig::permeability)
        .field("cflSafety", &MaxwellConfig::cflSafety)
        .field("maxSubstep", &MaxwellConfig::maxSubstep)
        .field("maximumSubsteps", &MaxwellConfig::maximumSubsteps)
        .field("maximumCellVisits", &MaxwellConfig::maximumCellVisits);
    value_object<MaxwellState>("MaxwellFieldState")
        .field("ez", &MaxwellState::ez)
        .field("hx", &MaxwellState::hx)
        .field("hy", &MaxwellState::hy);
    value_object<MaxwellGridDiagnostics>("MaxwellGridDiagnostics")
        .field("electricEnergy", &MaxwellGridDiagnostics::electricEnergy)
        .field("magneticEnergy", &MaxwellGridDiagnostics::magneticEnergy)
        .field("totalEnergy", &MaxwellGridDiagnostics::totalEnergy)
        .field("modifiedEnergy", &MaxwellGridDiagnostics::modifiedEnergy)
        .field("modifiedEnergyStep", &MaxwellGridDiagnostics::modifiedEnergyStep)
        .field("meanEz", &MaxwellGridDiagnostics::meanEz)
        .field("meanHx", &MaxwellGridDiagnostics::meanHx)
        .field("meanHy", &MaxwellGridDiagnostics::meanHy)
        .field("maxAbsEz", &MaxwellGridDiagnostics::maxAbsEz)
        .field("maxAbsHx", &MaxwellGridDiagnostics::maxAbsHx)
        .field("maxAbsHy", &MaxwellGridDiagnostics::maxAbsHy)
        .field("magneticDivergenceRms", &MaxwellGridDiagnostics::magneticDivergenceRms)
        .field("maxAbsMagneticDivergence", &MaxwellGridDiagnostics::maxAbsMagneticDivergence)
        .field("time", &MaxwellGridDiagnostics::time)
        .field("stableTimeStep", &MaxwellGridDiagnostics::stableTimeStep)
        .field("lastSubstep", &MaxwellGridDiagnostics::lastSubstep)
        .field("lastSubsteps", &MaxwellGridDiagnostics::lastSubsteps)
        .field("lastCellVisits", &MaxwellGridDiagnostics::lastCellVisits);
    class_<MaxwellGrid>("MaxwellGrid")
        .constructor(&CreateDefault, allow_raw_pointers())
        .constructor(&CreateConfigured, allow_raw_pointers())
        .function("getConfig", &Config)
        .function("getState", &State)
        .function("getDiagnostics", &MaxwellGrid::getDiagnostics)
        .function("getMagneticDivergence", optional_override([](const MaxwellGrid &grid) {
                      return Copy(grid.getMagneticDivergence());
                  }))
        .function("getWaveSpeed", &MaxwellGrid::getWaveSpeed)
        .function("getStableTimeStep", &MaxwellGrid::getStableTimeStep)
        .function("getModifiedEnergy", &MaxwellGrid::getModifiedEnergy)
        .function("setState", &SetState)
        .function("step", &MaxwellGrid::step);
}
