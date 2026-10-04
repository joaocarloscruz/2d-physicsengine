# 2D Physics Engine

A C++17 library for 2D rigid bodies, particles, fluids and mass-spring soft bodies.
Standalone modules add thermal conduction, prescribed-field charged particles,
planar Newtonian gravity, scalar membrane waves, periodic MAC projection and
viscosity, and periodic TMz electromagnetic fields.
WebAssembly exposes rigid bodies, particles, soft bodies, thermal networks,
charged particles, gravity, waves, periodic projection/viscosity, Maxwell fields
and owned spatial query results;
see the [JavaScript API](docs/webassembly.md) for supported interfaces.
Version 0.2 adds owned shapes, collision lifecycle events, swept circle collisions,
distance, revolute and prismatic joints with drives and stops, simulation islands,
optional sleeping, a DFSPH solver, deformable networks, and CSV/JSON exports.
The native library has no third-party runtime dependency
beyond the C++ runtime. Catch2 is vendored for tests; the separate visualizer uses SFML.

## Build and test

Use CMake 3.16+ and a C++17 compiler (GCC, Clang, or MSVC):

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

On Windows, run these commands in a compiler environment or use a portable
LLVM-MinGW distribution with its `bin` directory on the current process's PATH.
`build.ps1` runs the same configure/build/test sequence. Neither the library nor
its tests require the SFML visualizer. See [WebAssembly](docs/webassembly.md) for
the JavaScript build and [visualization](visualization/README.md) for SFML.

## Use the library

```cpp
#include <physics/physics.h>
using namespace PhysicsEngine;

World world;
auto body = std::make_shared<RigidBody>(Circle(0.5f), Material{}, Vector2(0, 2));
body->SetCcdEnabled(true);
world.addBody(body);
world.addUniversalForce(std::make_unique<Gravity>(Vector2(0, -9.81f)));
world.step();
std::string snapshot = ExportWorldJson(world, 1.0 / 60);
```

Bodies clone their shapes during construction. Stack shapes and temporary shapes
are safe; JavaScript shape handles can be deleted immediately after body creation.
Worlds own shared body/joint handles. Listeners remain caller-owned and must be
unregistered before they are destroyed.

## Install or embed

```sh
cmake --install build --config Release --prefix /path/to/physics-install
```

Consumers set `CMAKE_PREFIX_PATH` to that prefix, then use:

```cmake
find_package(PhysicsEngine 0.2 CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE PhysicsEngine::Engine)
```

The same target is available through `add_subdirectory`. Use `BUILD_SHARED_LIBS=OFF`
for static builds, `BUILD_TESTING=OFF` for library-only builds, and
`PHYSICS_BUILD_EXAMPLES=OFF` to omit the examples. Shared-library consumers must
make the installed library and their compiler's runtime available to the loader.

## Reproduce experiments

`replay_experiment [output-prefix]` runs a fixed 240-step bouncing-ball experiment
and writes a complete time series to `<prefix>.csv` and `<prefix>.json`. Re-running
the executable with the same build produces identical files. It defaults to
`experiment` in the current directory. Multi-configuration generators put the
executable in `build/Release`; single-configuration generators use `build`.

`fluid_solver_comparison [output.csv]` compares WCSPH and DFSPH at two resolutions
and two DFSPH tolerances, reporting density errors, compression rates, iterations,
and elapsed time. This benchmark is a free-surface compressing patch, not a general
claim of superiority across all fluid scenes.

`softbody_rope` runs a pinned ten-link elastic rope with gravity and axial damping,
then prints energy, strain and substep diagnostics. It requires no renderer.

`thermal_network_demo` reports conduction and reservoir energy accounting.
`gravity_binary` compares a two-body orbit with its analytical period, and
`wave_membrane_demo` measures a fixed-edge membrane mode's energy. These native
examples run without graphics and are included in the CTest smoke checks.

`mac_projection_demo` reports divergence, mean flow and energy before and after
a periodic grid projection. `mac_diffusion_demo` measures implicit viscosity,
its achieved velocity residual and dissipation. `maxwell_grid_demo` compares
oscillating physical field energy with the fixed-step modified invariant and
reports magnetic divergence. These are separate bounded operations/models.
`rigid_contact_benchmark --quick` measures resting
contacts, stacks, friction and isolated impacts against physical oracles; its
finite output includes substantial stack drift in difficult configurations.

## Documentation

- [Architecture, units and numerical limits](docs/architecture.md)
- [Physics capabilities, evidence and remaining gaps](docs/physics-capabilities.md)
- [Polygon centroids and rigid-body setup](docs/polygon-centering.md)
- [Supported API, ownership, errors and compatibility](docs/api.md)
- [Collision events and continuous detection](docs/collision-lifecycle-and-ccd.md)
- [Native and JavaScript point, ray, circle overlap and circle sweep queries](docs/spatial-queries.md)
- [Joints, islands and sleeping](docs/joints-and-sleeping.md)
- [Mass-spring deformable bodies](docs/soft-bodies.md)
- [Charged particles in prescribed electromagnetic fields](docs/electromagnetic-particles.md)
- [Thermal conduction networks](docs/thermal-networks.md)
- [Planar Newtonian N-body gravity](docs/nbody-gravity.md)
- [Scalar waves and membranes](docs/wave-membranes.md)
- [Periodic MAC projection and pressure diagnostics](docs/periodic-mac-projection.md)
- [Periodic MAC viscosity and dissipation](docs/periodic-mac-diffusion.md)
- [Periodic homogeneous TMz electromagnetic fields](docs/maxwell-grids.md)
- [Rigid-contact physical benchmark](docs/rigid-contact-benchmark.md)
- [Polygon manifold geometry and scale limits](docs/polygon-manifold-numerics.md)
- [DFSPH method and benchmark tradeoffs](docs/dfsph-solver.md)
- [Export schema and replay](docs/state-export.md)
- [Existing numerical validation](docs/numerical-validation.md)
- [WCSPH consistency diagnostic and formulation plan](docs/fluid-consistency-diagnostic.md)
- [Opt-in matched cubic kernels: measurements and limits](docs/cubic-kernel-experiments.md)
- [Sampled-wall force and pressure-work audit](docs/fluid-wall-audit.md)
- [Opt-in signed wall-pressure extrapolation](docs/wall-pressure-modes.md)
- [Experimental planar reflection and its measured limits](docs/planar-reflected-experiment.md)
- [WCSPH and boundaries](docs/wcsph-solver.md)
- [Fluid–rigid coupling](docs/fluid-rigid-coupling.md)

The library is experimental. Tests cover its documented operating cases; see each
feature's limitations before using it as a reference for a new physical regime.
The [coverage and validation roadmap](https://github.com/joaocarloscruz/2d-physicsengine/issues/52)
tracks the next capabilities and unresolved numerical work.
