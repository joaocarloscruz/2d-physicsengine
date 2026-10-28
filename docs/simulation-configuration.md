# Simulation configuration

`SimulationConfig` makes the engine's numerical policy explicit. A `World`
validates and copies the configuration, so callers cannot partially apply an
invalid update.

```cpp
PhysicsEngine::World world;
PhysicsEngine::SimulationConfig config = world.getSimulationConfig();

config.solverIterations = 16;
config.fixedTimeStep = 1.0f / 120.0f;
config.maxSubstepsPerAdvance = 12;
config.positionCorrectionFactor = 0.6f;
config.penetrationSlop = 0.002f;
config.warmStartFactor = 0.9f;
config.restitutionVelocityThreshold = 1.0f;
config.velocityTolerance = 0.0001f;
config.maxPositionCorrection = 0.2f;
config.enableLinearVelocityLimit = false;
config.enableAngularVelocityLimit = false;

world.setSimulationConfig(config);
```

The defaults preserve the behavior that predates this configuration API:

| Setting | Default |
| --- | ---: |
| Fixed timestep | `1 / 60` seconds |
| Maximum substeps per advance | `8` |
| Solver iterations | `10` |
| Position correction factor | `0.8` |
| Penetration slop | `0.005` |
| Warm-start factor | `1.0` |
| Restitution velocity threshold | `1.0` |
| Velocity tolerance | `0.0001` |
| Maximum position correction | `0.2` |
| Linear velocity limit | enabled at `200` |
| Angular velocity limit | enabled at `30` |
| Solver substeps | `4` |
| Contact hertz | `60` |
| Contact damping ratio | `2` |
| Contact push speed | `3` m/s |

The last four were added with the substepped contact solve and changed the
default step; `solverSubsteps = 1` restores the single-pass solve exactly.

The fixed timestep must be positive and finite, and the maximum substep count
must be positive. Correction and warm-start factors must be finite values between zero and one.
Penetration slop must be finite and non-negative. Enabled velocity limits must
be positive and finite. Set either enable flag to `false` when the simulation
must not silently clamp that velocity.

`enableSpeculativeContacts` (default `true`) selects the step order: detect
contacts at the start-of-step poses with speculative margins and integrate
positions after the velocity solve, or (`false`) the legacy order that
integrates first and detects at the new poses. See
[contact solver](contact-solver.md#step-order).

`solverSubsteps` (1 to 64, default 4) applies to the speculative order. Each
group of touching bodies without joints is solved in that many substeps, with
soft contacts (Box2D v3's soft step); groups with joints, groups whose soft
contacts would sag past their tolerance, and every group when the value is 1
take the single rigid solve. `solverIterations / solverSubsteps` iterations
(at least one) run per substep, plus one relax pass. `contactHertz` (positive,
default 60) is the soft contact's stiffness; it is capped at a quarter of the
substep rate, so 60 Hz needs a 60 Hz step with 4 substeps. `contactDampingRatio`
(non-negative, default 2) damps it, and `contactPushSpeed` (positive, default
3 m/s) caps the speed at which overlap is pushed out. See
[contact solver](contact-solver.md#substepped-soft-contacts).

Contacts slower than the restitution velocity threshold are solved without a
bounce, which prevents resting contacts from injecting energy. Velocity
components at or below the velocity tolerance are treated as settled. Maximum
position correction caps the distance repaired by one position iteration.

The same value object is available through WebAssembly using
`engine.getSimulationConfig()` and `engine.setSimulationConfig(config)`. It
carries every native field, including `solverSubsteps`, `contactHertz`,
`contactDampingRatio`, `contactPushSpeed`, `enableSpeculativeContacts` and
`enableFrictionDisplacementCorrection`. `setSimulationConfig` needs all fields,
so start from the object `getSimulationConfig()` returns. The integer fields
(`maxSubstepsPerAdvance`, `solverIterations`, `maximumCcdImpacts`,
`solverSubsteps`) reject fractional, non-finite and out-of-range numbers instead
of truncating them, and an invalid update leaves the configuration unchanged.
See [WebAssembly](webassembly.md).
