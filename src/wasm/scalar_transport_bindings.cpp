#include "checked_indices.h"
#include "physics/core/fluids/periodic_scalar_transport.h"
#include <cmath>
#include <emscripten/bind.h>
#include <stdexcept>
#include <string>

namespace PhysicsEngine::Wasm {
namespace {
struct ScalarGridConfig {
    double columns, rows, spacingX, spacingY;
};
struct ScalarOptions {
    double cflSafety, maxSubstep, maximumSubsteps, maximumCellVisits;
};
std::size_t BoundedCount(double value, std::size_t maximum) {
    if (!std::isfinite(value) || value > double(maximum))
        throw std::invalid_argument("Scalar transport count exceeds its hard bound.");
    return Count(value);
}
PeriodicScalarGridConfig Native(ScalarGridConfig c) {
    return {BoundedCount(c.columns, PeriodicScalarGridConfig::MaximumCells),
            BoundedCount(c.rows, PeriodicScalarGridConfig::MaximumCells), c.spacingX, c.spacingY};
}
ScalarTransportConfig Native(ScalarOptions c) {
    return {c.cflSafety, c.maxSubstep,
            BoundedCount(c.maximumSubsteps, ScalarTransportConfig::MaximumSubsteps),
            BoundedCount(c.maximumCellVisits, ScalarTransportConfig::MaximumCellVisits)};
}
PeriodicScalarTransport *CreateDefault() {
    return new PeriodicScalarTransport();
}
PeriodicScalarTransport *CreateConfigured(ScalarGridConfig c) {
    return new PeriodicScalarTransport(Native(c));
}
ScalarGridConfig Config(const PeriodicScalarTransport &grid) {
    const auto c = grid.config();
    return {double(c.columns), double(c.rows), c.spacingX, c.spacingY};
}
emscripten::val Copy(const std::vector<double> &values) {
    auto result = emscripten::val::array();
    for (std::size_t i = 0; i < values.size(); ++i)
        result.set(i, values[i]);
    return result;
}
emscripten::val Velocities(const PeriodicScalarTransport &grid) {
    const auto v = grid.velocities();
    auto result = emscripten::val::object();
    result.set("xFaces", Copy(v.xFaces));
    result.set("yFaces", Copy(v.yFaces));
    return result;
}
void CheckShape(const emscripten::val &values, std::size_t count) {
    if (!emscripten::val::global("Array").call<bool>("isArray", values) ||
        Count(values["length"].as<double>()) != count)
        throw std::invalid_argument("Scalar transport requires grid-sized JavaScript arrays.");
}
std::vector<double> Read(const emscripten::val &values, std::size_t count) {
    std::vector<double> result(count);
    const auto owns = emscripten::val::global("Object")["prototype"]["hasOwnProperty"];
    for (std::size_t i = 0; i < count; ++i) {
        if (!owns.call<bool>("call", values, emscripten::val(i)))
            throw std::invalid_argument("Scalar transport arrays must be dense.");
        const auto value = values[i];
        if (value.typeOf().as<std::string>() != "number")
            throw std::invalid_argument("Scalar transport entries must be numbers.");
        result[i] = value.as<double>();
        if (!std::isfinite(result[i]))
            throw std::invalid_argument("Scalar transport entries must be finite.");
    }
    return result;
}
void SetState(PeriodicScalarTransport &grid, const emscripten::val &values) {
    const auto c = grid.config();
    const auto count = c.columns * c.rows;
    CheckShape(values, count);
    grid.setState(Read(values, count));
}
void SetVelocities(PeriodicScalarTransport &grid, const emscripten::val &x,
                   const emscripten::val &y) {
    const auto c = grid.config();
    const auto count = c.columns * c.rows;
    // Both shapes precede any allocation/copy of a native field.
    CheckShape(x, count);
    CheckShape(y, count);
    grid.setVelocities({Read(x, count), Read(y, count)});
}
} // namespace
} // namespace PhysicsEngine::Wasm

EMSCRIPTEN_BINDINGS(periodic_scalar_transport) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    value_object<ScalarGridConfig>("PeriodicScalarGridConfig")
        .field("columns", &ScalarGridConfig::columns)
        .field("rows", &ScalarGridConfig::rows)
        .field("spacingX", &ScalarGridConfig::spacingX)
        .field("spacingY", &ScalarGridConfig::spacingY);
    value_object<ScalarOptions>("ScalarTransportConfig")
        .field("cflSafety", &ScalarOptions::cflSafety)
        .field("maxSubstep", &ScalarOptions::maxSubstep)
        .field("maximumSubsteps", &ScalarOptions::maximumSubsteps)
        .field("maximumCellVisits", &ScalarOptions::maximumCellVisits);
    value_object<ScalarTransportDiagnostics>("ScalarTransportDiagnostics")
        .field("substeps", &ScalarTransportDiagnostics::substeps)
        .field("cellVisits", &ScalarTransportDiagnostics::cellVisits)
        .field("duration", &ScalarTransportDiagnostics::duration)
        .field("timeBefore", &ScalarTransportDiagnostics::timeBefore)
        .field("timeAfter", &ScalarTransportDiagnostics::timeAfter)
        .field("lastSubstep", &ScalarTransportDiagnostics::lastSubstep)
        .field("maximumOutflowRate", &ScalarTransportDiagnostics::maximumOutflowRate)
        .field("outflowRateBound", &ScalarTransportDiagnostics::outflowRateBound)
        .field("maximumAbsDivergence", &ScalarTransportDiagnostics::maximumAbsDivergence)
        .field("maximumCfl", &ScalarTransportDiagnostics::maximumCfl)
        .field("initialIntegratedScalar", &ScalarTransportDiagnostics::initialIntegratedScalar)
        .field("finalIntegratedScalar", &ScalarTransportDiagnostics::finalIntegratedScalar)
        .field("initialAbsoluteIntegral", &ScalarTransportDiagnostics::initialAbsoluteIntegral)
        .field("finalAbsoluteIntegral", &ScalarTransportDiagnostics::finalAbsoluteIntegral)
        .field("integratedScalarDrift", &ScalarTransportDiagnostics::integratedScalarDrift)
        .field("conservationRoundoffAllowance",
               &ScalarTransportDiagnostics::conservationRoundoffAllowance)
        .field("initialMinimum", &ScalarTransportDiagnostics::initialMinimum)
        .field("initialMaximum", &ScalarTransportDiagnostics::initialMaximum)
        .field("finalMinimum", &ScalarTransportDiagnostics::finalMinimum)
        .field("finalMaximum", &ScalarTransportDiagnostics::finalMaximum)
        .field("rangeRoundoffAllowance", &ScalarTransportDiagnostics::rangeRoundoffAllowance)
        .field("nonnegativeInput", &ScalarTransportDiagnostics::nonnegativeInput)
        .field("discreteDivergenceFree", &ScalarTransportDiagnostics::discreteDivergenceFree)
        .field("zeroDurationNoOp", &ScalarTransportDiagnostics::zeroDurationNoOp);
    class_<PeriodicScalarTransport>("PeriodicScalarTransport")
        .constructor(&CreateDefault, allow_raw_pointers())
        .constructor(&CreateConfigured, allow_raw_pointers())
        .function("getConfig", &Config)
        .function("getState", optional_override([](const PeriodicScalarTransport &grid) {
                      return Copy(grid.state());
                  }))
        .function("getVelocities", &Velocities)
        .function("getTime", &PeriodicScalarTransport::time)
        .function("getLastStep", &PeriodicScalarTransport::lastStep)
        .function("setState", &SetState)
        .function("setVelocities", &SetVelocities)
        .function("step", optional_override([](PeriodicScalarTransport &grid, double duration) {
                      return grid.step(duration);
                  }))
        .function("step", optional_override(
                              [](PeriodicScalarTransport &grid, double duration, ScalarOptions c) {
                                  return grid.step(duration, Native(c));
                              }));
}
