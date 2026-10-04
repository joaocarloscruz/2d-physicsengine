#pragma once
// Supported consumer entry point. Solver caches, broad/narrow phase details,
// and headers not listed in docs/api.md are implementation interfaces.
#include "engine.h"
#include "physics/core/joints.h"
#include "physics/core/softbody.h"
#include "physics/core/forces/gravity.h"
#include "physics/core/forces/drag.h"
#include "physics/core/collisions/continuous_collision.h"
#include "physics/core/fluids/dfsph_solver.h"
#include "physics/core/fluids/wcsph_solver.h"
#include "physics/core/fluids/coupled_fluid_simulation.h"
#include "physics/core/state_export.h"
