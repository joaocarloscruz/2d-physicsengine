#pragma once
#include "checked_indices.h"
#include "physics/core/fluids/periodic_mac_grid.h"
#include <emscripten/val.h>
#include <vector>

// JavaScript value types shared by the MAC projection and incompressible-flow
// bindings. mac_bindings.cpp registers them once under their MAC names.
namespace PhysicsEngine::Wasm {
struct MacGridConfig {
    double columns, rows, spacingX, spacingY;
};
struct MacVelocities {
    emscripten::val xFaces, yFaces;
};
struct MacSnapshot {
    emscripten::val potential, pressure;
    MacProjectionDiagnostics diagnostics;
};
inline PeriodicMacGridConfig Native(MacGridConfig c) {
    return {Count(c.columns), Count(c.rows), c.spacingX, c.spacingY};
}
inline MacGridConfig Public(const PeriodicMacGridConfig &c) {
    return {double(c.columns), double(c.rows), c.spacingX, c.spacingY};
}
inline emscripten::val Copy(const std::vector<double> &values) {
    auto result = emscripten::val::array();
    for (std::size_t i = 0; i < values.size(); ++i)
        result.set(i, values[i]);
    return result;
}
inline MacVelocities Public(const MacVelocityState &v) {
    return {Copy(v.xFaces), Copy(v.yFaces)};
}
inline MacSnapshot Public(const MacProjectionSnapshot &p) {
    return {Copy(p.potential), Copy(p.pressure), p.diagnostics};
}
} // namespace PhysicsEngine::Wasm
