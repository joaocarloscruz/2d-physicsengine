# Changelog

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
