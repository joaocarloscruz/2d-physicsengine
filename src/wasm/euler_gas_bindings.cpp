#include "checked_indices.h"
#include "physics/core/fluids/periodic_euler_gas_grid.h"
#include <cmath>
#include <emscripten/bind.h>
#include <stdexcept>
#include <string>

namespace PhysicsEngine::Wasm {
namespace {
struct EulerConfig {
    double columns, rows, spacingX, spacingY, gamma;
};
struct EulerOptions {
    double cflSafety, maxSubstep, maximumSubsteps, maximumCellVisits;
};
struct EulerSecondOptions {
    double cflSafety, maxSubstep, maximumSubsteps, maximumCellVisits;
    double maximumAttempts, maximumRetriesPerSubstep;
};
struct EulerState {
    emscripten::val density, momentumX, momentumY, totalEnergy;
};
struct EulerPrimitives {
    emscripten::val velocityX, velocityY, pressure, soundSpeed, internalEnergy;
};
std::size_t BoundedCount(double value, std::size_t minimum, std::size_t maximum) {
    if (!std::isfinite(value) || value < double(minimum) || value > double(maximum))
        throw std::invalid_argument("Euler gas count exceeds its hard bounds.");
    return Count(value);
}
EulerGasGridConfig Native(EulerConfig c) {
    return {BoundedCount(c.columns, 2, EulerGasGridConfig::MaximumCells),
            BoundedCount(c.rows, 2, EulerGasGridConfig::MaximumCells), c.spacingX, c.spacingY,
            c.gamma};
}
EulerGasStepConfig Native(EulerOptions c) {
    return {c.cflSafety, c.maxSubstep,
            BoundedCount(c.maximumSubsteps, 0, EulerGasStepConfig::MaximumSubsteps),
            BoundedCount(c.maximumCellVisits, 0, EulerGasStepConfig::MaximumCellVisits)};
}
EulerGasSecondOrderConfig Native(EulerSecondOptions c) {
    EulerGasSecondOrderConfig result;
    static_cast<EulerGasStepConfig &>(result) =
        Native(EulerOptions{c.cflSafety, c.maxSubstep, c.maximumSubsteps, c.maximumCellVisits});
    result.maximumAttempts =
        BoundedCount(c.maximumAttempts, 0, EulerGasSecondOrderConfig::MaximumAttempts);
    result.maximumRetriesPerSubstep = BoundedCount(
        c.maximumRetriesPerSubstep, 0, EulerGasSecondOrderConfig::MaximumRetriesPerSubstep);
    return result;
}
PeriodicEulerGasGrid *CreateDefault() { return new PeriodicEulerGasGrid(); }
PeriodicEulerGasGrid *CreateConfigured(EulerConfig c) {
    return new PeriodicEulerGasGrid(Native(c));
}
EulerConfig Config(const PeriodicEulerGasGrid &grid) {
    const auto c = grid.config();
    return {double(c.columns), double(c.rows), c.spacingX, c.spacingY, c.gamma};
}
emscripten::val Copy(const std::vector<double> &values) {
    auto result = emscripten::val::array();
    for (std::size_t i = 0; i < values.size(); ++i)
        result.set(i, values[i]);
    return result;
}
EulerState State(const PeriodicEulerGasGrid &grid) {
    const auto s = grid.state();
    return {Copy(s.density), Copy(s.momentumX), Copy(s.momentumY), Copy(s.totalEnergy)};
}
EulerPrimitives Primitives(const PeriodicEulerGasGrid &grid) {
    const auto p = grid.primitives();
    return {Copy(p.velocityX), Copy(p.velocityY), Copy(p.pressure), Copy(p.soundSpeed),
            Copy(p.internalEnergy)};
}
void SetState(PeriodicEulerGasGrid &grid, const EulerState &state) {
    using emscripten::val;
    const val *inputs[] = {&state.density, &state.momentumX, &state.momentumY, &state.totalEnergy};
    const auto c = grid.config();
    const auto count = c.columns * c.rows;
    for (const auto *values : inputs) {
        if (!val::global("Array").call<bool>("isArray", *values))
            throw std::invalid_argument("Euler gas fields require ordinary JavaScript arrays.");
        if (Count((*values)["length"].as<double>()) != count)
            throw std::invalid_argument("Euler gas field length does not match the grid.");
    }
    EulerGasState staged;
    std::vector<double> *outputs[] = {&staged.density, &staged.momentumX, &staged.momentumY,
                                      &staged.totalEnergy};
    for (auto *values : outputs)
        values->resize(count);
    const auto owns = val::global("Object")["prototype"]["hasOwnProperty"];
    for (std::size_t field = 0; field < 4; ++field)
        for (std::size_t i = 0; i < count; ++i) {
            const auto &values = *inputs[field];
            if (!owns.call<bool>("call", values, val(i)))
                throw std::invalid_argument("Euler gas arrays must be dense.");
            const auto value = values[i];
            if (value.typeOf().as<std::string>() != "number")
                throw std::invalid_argument("Euler gas entries must be numbers.");
            const double number = value.as<double>();
            if (!std::isfinite(number))
                throw std::invalid_argument("Euler gas entries must be finite.");
            (*outputs[field])[i] = number;
        }
    grid.setState(staged);
}
template <class Diagnostic>
emscripten::value_object<Diagnostic> &BaseReportFields(emscripten::value_object<Diagnostic> &report) {
    return report.field("initial", &EulerGasDiagnostics::initial)
        .field("final", &EulerGasDiagnostics::final)
        .field("massDefect", &EulerGasDiagnostics::massDefect)
        .field("momentumXDefect", &EulerGasDiagnostics::momentumXDefect)
        .field("momentumYDefect", &EulerGasDiagnostics::momentumYDefect)
        .field("totalEnergyDefect", &EulerGasDiagnostics::totalEnergyDefect)
        .field("massRoundoffAllowance", &EulerGasDiagnostics::massRoundoffAllowance)
        .field("momentumXRoundoffAllowance", &EulerGasDiagnostics::momentumXRoundoffAllowance)
        .field("momentumYRoundoffAllowance", &EulerGasDiagnostics::momentumYRoundoffAllowance)
        .field("totalEnergyRoundoffAllowance", &EulerGasDiagnostics::totalEnergyRoundoffAllowance)
        .field("duration", &EulerGasDiagnostics::duration)
        .field("timeBefore", &EulerGasDiagnostics::timeBefore)
        .field("timeAfter", &EulerGasDiagnostics::timeAfter)
        .field("lastSubstep", &EulerGasDiagnostics::lastSubstep)
        .field("maximumSignalSpeedX", &EulerGasDiagnostics::maximumSignalSpeedX)
        .field("maximumSignalSpeedY", &EulerGasDiagnostics::maximumSignalSpeedY)
        .field("maximumCfl", &EulerGasDiagnostics::maximumCfl)
        .field("substeps", &EulerGasDiagnostics::substeps)
        .field("cellVisits", &EulerGasDiagnostics::cellVisits)
        .field("zeroDurationNoOp", &EulerGasDiagnostics::zeroDurationNoOp);
}
} // namespace
} // namespace PhysicsEngine::Wasm

EMSCRIPTEN_BINDINGS(periodic_euler_gas_grid) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    value_object<EulerConfig>("EulerGasGridConfig")
        .field("columns", &EulerConfig::columns)
        .field("rows", &EulerConfig::rows)
        .field("spacingX", &EulerConfig::spacingX)
        .field("spacingY", &EulerConfig::spacingY)
        .field("gamma", &EulerConfig::gamma);
    value_object<EulerOptions>("EulerGasStepConfig")
        .field("cflSafety", &EulerOptions::cflSafety)
        .field("maxSubstep", &EulerOptions::maxSubstep)
        .field("maximumSubsteps", &EulerOptions::maximumSubsteps)
        .field("maximumCellVisits", &EulerOptions::maximumCellVisits);
    value_object<EulerSecondOptions>("EulerGasSecondOrderConfig")
        .field("cflSafety", &EulerSecondOptions::cflSafety)
        .field("maxSubstep", &EulerSecondOptions::maxSubstep)
        .field("maximumSubsteps", &EulerSecondOptions::maximumSubsteps)
        .field("maximumCellVisits", &EulerSecondOptions::maximumCellVisits)
        .field("maximumAttempts", &EulerSecondOptions::maximumAttempts)
        .field("maximumRetriesPerSubstep", &EulerSecondOptions::maximumRetriesPerSubstep);
    value_object<EulerState>("EulerGasState")
        .field("density", &EulerState::density)
        .field("momentumX", &EulerState::momentumX)
        .field("momentumY", &EulerState::momentumY)
        .field("totalEnergy", &EulerState::totalEnergy);
    value_object<EulerPrimitives>("EulerGasPrimitives")
        .field("velocityX", &EulerPrimitives::velocityX)
        .field("velocityY", &EulerPrimitives::velocityY)
        .field("pressure", &EulerPrimitives::pressure)
        .field("soundSpeed", &EulerPrimitives::soundSpeed)
        .field("internalEnergy", &EulerPrimitives::internalEnergy);
    value_object<EulerGasSummary>("EulerGasSummary")
        .field("mass", &EulerGasSummary::mass)
        .field("momentumX", &EulerGasSummary::momentumX)
        .field("momentumY", &EulerGasSummary::momentumY)
        .field("totalEnergy", &EulerGasSummary::totalEnergy)
        .field("absoluteMomentumX", &EulerGasSummary::absoluteMomentumX)
        .field("absoluteMomentumY", &EulerGasSummary::absoluteMomentumY)
        .field("internalEnergy", &EulerGasSummary::internalEnergy)
        .field("kineticEnergy", &EulerGasSummary::kineticEnergy)
        .field("minimumDensity", &EulerGasSummary::minimumDensity)
        .field("maximumDensity", &EulerGasSummary::maximumDensity)
        .field("minimumPressure", &EulerGasSummary::minimumPressure)
        .field("maximumPressure", &EulerGasSummary::maximumPressure);
    value_object<EulerGasDiagnostics> baseReport("EulerGasDiagnostics");
    BaseReportFields(baseReport);
    value_object<EulerGasSecondOrderDiagnostics> highReport("EulerGasSecondOrderDiagnostics");
    BaseReportFields(highReport)
        .field("attempts", &EulerGasSecondOrderDiagnostics::attempts)
        .field("rejectedAttempts", &EulerGasSecondOrderDiagnostics::rejectedAttempts)
        .field("reconstructionPreparations",
               &EulerGasSecondOrderDiagnostics::reconstructionPreparations)
        .field("reconstructionTrials", &EulerGasSecondOrderDiagnostics::reconstructionTrials)
        .field("limitedSlopeCells", &EulerGasSecondOrderDiagnostics::limitedSlopeCells)
        .field("positivityLimitedCells", &EulerGasSecondOrderDiagnostics::positivityLimitedCells)
        .field("rangeLimitedCells", &EulerGasSecondOrderDiagnostics::rangeLimitedCells)
        .field("zeroSlopeFallbackCells", &EulerGasSecondOrderDiagnostics::zeroSlopeFallbackCells)
        .field("forwardEulerStages", &EulerGasSecondOrderDiagnostics::forwardEulerStages)
        .field("blendPasses", &EulerGasSecondOrderDiagnostics::blendPasses)
        .field("minimumSlopeScale", &EulerGasSecondOrderDiagnostics::minimumSlopeScale)
        .field("maximumRejectedCfl", &EulerGasSecondOrderDiagnostics::maximumRejectedCfl);
    class_<PeriodicEulerGasGrid>("PeriodicEulerGasGrid")
        .constructor(&CreateDefault, allow_raw_pointers())
        .constructor(&CreateConfigured, allow_raw_pointers())
        .function("config", &Config)
        .function("state", &State)
        .function("primitives", &Primitives)
        .function("lastStep", &PeriodicEulerGasGrid::lastStep)
        .function("lastSecondOrderStep", &PeriodicEulerGasGrid::lastSecondOrderStep)
        .function("time", &PeriodicEulerGasGrid::time)
        .function("setState", &SetState)
        .function("step", optional_override([](PeriodicEulerGasGrid &grid, double duration) {
                      return grid.step(duration);
                  }))
        .function("step", optional_override([](PeriodicEulerGasGrid &grid, double duration,
                                               EulerOptions options) {
                      return grid.step(duration, Native(options));
                  }))
        .function("stepSecondOrder", optional_override([](PeriodicEulerGasGrid &grid, double duration) {
                      return grid.stepSecondOrder(duration);
                  }))
        .function("stepSecondOrder", optional_override([](PeriodicEulerGasGrid &grid, double duration,
                                                          EulerSecondOptions options) {
                      return grid.stepSecondOrder(duration, Native(options));
                  }));
}
