# Public API and compatibility

Include `physics/physics.h` and link `PhysicsEngine::Engine`. Supported consumer
types are `Engine`, `World`, `RigidBody`, shapes/materials, force generators,
`SimulationConfig`, statistics, fixed-step runner, listeners/events, joints,
particle systems, standalone `SoftBody` mass-spring and `ThermalNetwork`
conduction networks, standalone `WaveMembrane` scalar waves, fluid solvers/containers/coupling, point/ray/sweep queries, and export
functions. Solver caches, constraint preparation, and broad/narrow-phase
implementation headers are internal interfaces without compatibility guarantees.

The native `ChargedParticle`, `UniformElectromagneticField` and `Vector2d` types
support [prescribed-field electromagnetic particle motion](electromagnetic-particles.md).

`NBodyGravity` supports independent [planar Newtonian point-mass gravity](nbody-gravity.md)
with bounded pair work, optional Plummer softening and transactional stepping.

`WaveMembrane` and its configuration/boundary/diagnostic types support
[uniform scalar waves and membranes](wave-membranes.md). Its steps are staged:
numeric or budget failures preserve state, queued acceleration and diagnostics.

See [native spatial queries](spatial-queries.md) for boundary/inside semantics,
finite segment hits, deterministic ordering, collision filters and retained handles.

## Ownership

`RigidBody` owns an immutable cloned shape (`unique_ptr<const Shape>`). The pointer
constructor is retained as a copying adapter; it never adopts the caller's pointer.
Custom shape subclasses must implement `Clone`. The read-only shape observer is
`body.shape.get()`. Bodies cannot be copied, preventing duplicate stable IDs.
World and joint containers hold shared body handles. Removing a body also removes
its attached joints, force registrations, and contacts. IDs are never reused
within the process; they are not portable identifiers across separate simulations.

Collision events contain values and IDs, so they remain valid after removal. The
legacy manifold callback contains borrowed body pointers, valid for that callback.
Listeners are caller-owned: unregister a listener before destruction. A World
destructor does not invoke callbacks. Simulation objects are not thread-safe.

## Error behavior

Constructors and setters reject invalid numeric arguments with
`std::invalid_argument`, including non-finite vectors, negative timesteps, invalid
shapes/materials and invalid solver settings. Registration APIs reject null inputs;
duplicate bodies, joints and listeners are idempotent. Removing an absent/null
object is a no-op. Joints require two distinct bodies already registered in their
world, with at least one dynamic body. `Engine::getMaterial` throws `out_of_range`
for missing keys. Calling `step` from a collision callback throws `logic_error`.

Step failures caused by numerical overflow or exhausting a fluid substep budget
throw a runtime error and may leave a partially advanced simulation; they are not
transactions. DFSPH pressure iteration exhaustion instead sets `converged=false`
and reports residuals. CCD impact exhaustion reports a flag and leaves bodies at
the last safe sweep time. Listener exceptions propagate after clearing queued
notifications. Export rejects non-finite state instead of emitting invalid JSON.

Rigid-body force/torque accumulation, impulse application and individual
integration updates check derived float results before publication. An
`std::overflow_error` preserves that operation's body state, pending loads and
wake state. Integration uses double intermediates and a stable speed norm for
optional velocity caps. This does not roll back other bodies already advanced by
a failing `World::step`; contact and joint solvers are also separate operations.

The standalone `SoftBody` module stages each step and leaves particle state and
substep diagnostics unchanged when numerical or work-budget checks fail. Its
validated state, finite anchors and integration limits are described in
[soft-bodies.md](soft-bodies.md).

`ThermalNetwork` also stages steps and preserves queued powers and energy
accounting on failure. Its units, temperature bounds and reservoir bookkeeping
are described in [thermal-networks.md](thermal-networks.md).

Legacy public body fields are retained for source compatibility. Direct mutation
bypasses validation and wake tracking; use setters and `Wake()` for application
code. Do not change mass/inertia fields independently. Force-generator parameter
changes require explicitly waking affected bodies.

## Versioning

The CMake package version is 0.2.0. While version zero is under development, a minor
release may change source APIs or numerical behavior; patch releases preserve the
documented API. Rebuild all consumers on upgrades; binary ABI compatibility is not
promised. CMake accepts only the requested minor version. Export schemas are
versioned independently (`schemaVersion: 1`); readers should ignore new fields.
Bitwise reproducibility is expected only for identical inputs, builds and platforms.

Migration from the previous unversioned API: replace shape casts such as
`static_cast<Circle*>(body.shape)` with `static_cast<const Circle*>(body.shape.get())`.
Source shape lifetime management is no longer necessary. Default `Material{}` is
now valid. Null registrations and non-finite state setters now throw consistently.

Prismatic slider joints: see [prismatic-joints.md](prismatic-joints.md); public header `physics/core/prismatic_joint.h`.
