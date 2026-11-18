# Changelog

## Unreleased

- `SimulationConfig::enableJointSubsteps` (off by default) lets groups whose
  joints are all distance or revolute joints take the soft substep path:
  soft joint rows toward the position error at the current poses
  (`jointHertz`, default 60; `jointDampingRatio`, default 2), warm-started
  every substep, plus a rigid relax. A 10-link chain holding a 100x heavier
  box stretches 0.14 m instead of 0.53 m and jitters at 0.12 m/s instead of
  2.3 m/s. Motors, limits, springs and torque caps work per substep. With the
  option off every result is unchanged bit for bit. The fields are bound in
  the WebAssembly `SimulationConfig` value object.
- CCD now sweeps position corrections (joint and contact position solves)
  against moving bodies as well as static ones. A joint pulling a thin CCD
  body toward a thin dynamic plank carried it through the plank in one step,
  since only static targets were swept; now a CCD body's correction is also
  cut at contact with any body it collides with, each target held at the pose
  it ends the step in (a target without CCD keeps its whole correction).
  Scenes without CCD bodies are unchanged bit for bit.
- CCD sweeps only the pairs whose bounds over the step overlap, instead of
  every pair with a CCD body on every impact: a 150-body pile hit by 60 CCD
  bullets of mixed shapes takes 2.2 ms per step instead of 30 ms. Results are
  unchanged bit for bit.
- The legacy step order (`enableSpeculativeContacts = false`) solves joints
  before contacts in each velocity iteration, like the default order. Solving
  joints last left light jointed bodies closing on their contacts, and
  restitution bounced that speed back: scenes with 1:40 to 1:350 jointed mass
  ratios gained up to 32% energy. The default order is unchanged bit for bit.
- GCC and Clang builds disable floating-point contraction
  (`-ffp-contract=off`). GCC fused multiply-adds whenever the target CPU had
  FMA, so builds for newer CPUs (`-march=x86-64-v3`, and the current
  ubuntu-latest CI compiler) gave different results and failed exact-cancellation
  tests. Baseline x86-64 builds are unchanged bit for bit.
- Documented the joint mass-ratio limit and redundant joints (see
  [joints](docs/joints-and-sleeping.md)).

## 1.0.0

The first release considered complete for its documented scope: a 2D rigid-body
engine for games and small numerical models, with fluids and standalone
simulation modules beside it. Measured limits are listed in
[getting started](docs/getting-started.md#known-limits) and
[physics capabilities](docs/physics-capabilities.md).

### Rigid bodies

- Contacts are predicted at the start of each step (speculative contacts), so
  fast bodies stop at surfaces instead of sinking in; resting overlap is zero.
- Contact-only groups are solved in soft substeps (`solverSubsteps = 4`), so
  towers of 20 slightly misaligned boxes stand at 60 Hz. `solverSubsteps = 1`
  keeps the single-pass solve.
- Warm starting at full strength, stable polygon feature ids, two-point
  manifolds corrected as a block, and joints warm-started and solved with the
  contacts.
- Sensor bodies report overlaps through the collision events without pushing.
  Bodies carry opaque user data.
- New joints: `MouseJoint`, `WeldJoint` and `WheelJoint`. `DistanceJoint` gains
  a spring and a length range (rope). Jointed bodies no longer collide with each
  other unless `setCollideConnected(true)`.
- CCD covers every shape pair (circles, polygons and capsules, static or
  moving) and sweeps rotation: thin boxes, capsule bullets and spinning
  polygons no longer pass through thin walls at up to the speed and spin
  limits, and capsule pairs no longer make the step throw. A body pressed into
  another past its allowed depth is stopped for the rest of the step instead of
  being let through.
- CCD also sweeps position correction against static walls, and CCD bodies may
  share a world with sensor capsules. Fixes found by seeded World fuzzing: a
  valid polygon no longer makes the step throw after float rounding, resting or
  wedged CCD bodies no longer exhaust the impact budget and freeze the world,
  and a CCD impact on a sleeping body no longer breaks the step.
- The linear and angular velocity limits are applied again after the solve, so
  joint and contact impulses can no longer push a body past them.
- Narrow-phase fixes found by seeded property tests: capsule manifolds report
  the exact minimum translation, capsule contacts no longer jump on near ties,
  and polygon speculative gaps are no longer overstated.
- An optimization pass made the step about twice as fast without changing any
  result; with the soft substeps, 1000 settled bodies take about 4.4 ms per
  60 Hz step on a Ryzen 7 5700X. Separate Worlds may step on separate threads.

### Fluids

- WCSPH defaults meet all nine physical targets of issue #44 (continuity
  density, Wendland C2 kernel, acoustic damping, particle shifting, hydrostatic
  initialization, constant-pressure wall completion).
- Capsules couple with fluids like circles and polygons.

### WebAssembly

- JavaScript exposes capsules, sensors, user data, every joint type, every
  `SimulationConfig` field, uniform gravity and a bounded collision event
  buffer.
- A browser playground (`examples/web-playground`) runs ten scenes, including a
  drivable car with a sensor goal and a joints gallery.

### Tools and documentation

- A getting-started guide, a cookbook of self-checking examples (platformer,
  ragdoll, vehicle, destructible wall, trigger zones) and a README with
  screenshots.
- CTest gates for visible collision quality, 99 rigid-body scenarios, seeded
  narrow-phase properties and the step benchmark smoke run.
- The SFML visualizer has ten scenes, drags with a mouse joint and draws
  sensors and every joint type.

### Compatibility

- The CMake package is now 1.0.0 with same-major compatibility
  (`find_package(PhysicsEngine 1.0)`).
- Default numerical results differ from 0.2 because of the solver changes
  above.
- Private members were added to `World` and `RigidBody`; rebuild everything
  that links the library.
- JavaScript `setSimulationConfig` needs every field; spread the object from
  `getSimulationConfig()`.
