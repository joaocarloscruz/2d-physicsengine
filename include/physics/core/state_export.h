#pragma once
#include "world.h"
#include "fluids/fluid_solver.h"
#include <string>

namespace PhysicsEngine {
// Schema version 1, locale-independent, round-trip float precision. Throws
// invalid_argument for non-finite state/time rather than producing invalid JSON.
std::string ExportWorldJson(const World& world, double time = 0);
std::string ExportWorldCsv(const World& world, double time = 0);
std::string ExportFluidJson(const std::vector<FluidParticle>& particles,
    const FluidDiagnostics& diagnostics, double time = 0);
std::string ExportFluidCsv(const std::vector<FluidParticle>& particles, double time = 0);
}
