# WebAssembly build

The WebAssembly target exposes the engine's basic simulation API to JavaScript
through Emscripten's Embind library.

Scene queries are available through `queryPoint`, `queryCircle`, `rayCastAll`,
`rayCastNearest`, `sweepCircleAll` and `sweepCircleNearest`. They accept an Engine
and return owned collections with exact BigInt body IDs, retained body handles
and copied hit geometry. See [query arguments, filtering and object cleanup](spatial-queries.md#javascript-queries-and-result-ownership).

## Owned periodic MAC projection grids

`PeriodicMacGrid` owns a standalone periodic velocity projection grid.
`new physics.PeriodicMacGrid()` uses 16 columns, 16 rows and unit spacings.
The configured constructor accepts a complete plain object with `columns`,
`rows`, `spacingX` and `spacingY`. Geometry is fixed after construction.
This module projects velocities; it does not implement advection, viscosity,
walls, free surfaces, time integration or Engine/World/SPH coupling. See the
[native operators, physical units and accuracy limits](periodic-mac-projection.md).

```javascript
const grid = new physics.PeriodicMacGrid({
    columns: 8, rows: 6, spacingX: 0.1, spacingY: 0.17
});
let projection;
try {
    const c = grid.getConfig();
    const n = c.columns * c.rows;
    const xFaces = Array.from({length: n}, (_, k) =>
        0.3 + Math.sin(2 * Math.PI * (k % c.columns) / c.columns));
    grid.setVelocities(xFaces, Array(n).fill(-0.2));
    const diagnostics = grid.project({
        density: 1000, timeStep: 0.01,
        absoluteDivergenceTolerance: 1e-10,
        relativeDivergenceTolerance: 1e-10,
        maximumIterations: 1000, maximumCellVisits: 100000000
    });
    projection = grid.getLastProjection();
    console.log(grid.getVelocities(), grid.getDivergence(), diagnostics);
} finally {
    grid.delete();
}
console.log(projection.pressure); // Plain copied arrays survive grid deletion.
```

Each array uses index `i + columns*j` and has `columns*rows` entries.
`xFaces` stores u at `(i*dx, (j+0.5)*dy)`; `yFaces` stores v at
`((i+0.5)*dx, j*dy)`. Potential, pressure and divergence are cell-centered at
`((i+0.5)*dx, (j+0.5)*dy)`. Periods are `columns*dx` and `rows*dy`;
periodic faces are stored once, without a duplicate last row or column.

`getConfig()`, `getVelocities()`, `getDivergence()` and
`getLastProjection()` return plain copied objects/arrays, including the nested
`diagnostics` object. Changing inputs after `setVelocities`, mutating a snapshot,
or deleting the grid cannot change other snapshots. No borrowed WASM memory,
typed views or vector wrappers are returned; snapshots require no deletion.
Delete the owned grid handle once when finished.

`setVelocities(xFaces, yFaces)` accepts two plain JS arrays with finite numeric
entries. Both lengths are checked before either native copy is allocated or
read; typed arrays, sparse arrays, numeric strings and nonfinite entries are
rejected. Both components are copied and committed together. The last successful
projection snapshot remains available after a velocity setter; it describes
that earlier projection until another projection succeeds.

`project()` uses density 1, pressure-conversion interval 1, absolute and relative
divergence tolerances `1e-10`, 1000 iterations and 100000000 cell visits.
`project(options)` requires all six fields shown above. The accepted target is
`max(absoluteDivergenceTolerance, relativeDivergenceTolerance*initialDivergenceRms)`.
The stored face velocities must meet the recomputed target. Exactly zero
divergence preserves velocities and publishes zero potential/pressure with zero
iterations; diagnostic passes still consume work. `timeStep` converts potential
to pressure and does not advance a clock. All failed calls retain velocities and
the complete last successful projection snapshot.

Dimensions and work/iteration counts are checked as JS doubles before native
integer conversion: negative, fractional, nonfinite and wrapping values are
rejected. Each dimension is at least 2 and the product is at most 262144 cells,
checked before allocation. Iterations are bounded by 1000000 and cell visits by
1000000000. Zero budgets are valid inputs and fail if the operation needs more
work. Native geometry, tolerance, pressure-scale and arithmetic validation also
applies; there is no hidden tolerance floor.

Diagnostics expose every native field with these units (m and s when using SI):

| Fields | Units / meaning |
| --- | --- |
| `iterations`, `cellVisits`, `zeroDivergenceNoOp` | Accepted iteration/pass work counts; exact-zero initial-divergence flag |
| `density`, `timeStep` | Accepted density and pressure-conversion interval in s |
| `initialDivergenceRms`, `finalDivergenceRms`, `targetDivergenceRms`, `removedDivergenceMean` | 1/s |
| `potentialMean` and snapshot `potential` | m²/s, zero-mean gauge |
| `pressureMean` and snapshot `pressure` | `density*potential/timeStep`: Pa for kg/m³ density, N/m for kg/m² density |
| `initialMeanX`, `initialMeanY`, `finalMeanX`, `finalMeanY` | Mean face velocity, m/s |
| `initialKineticEnergy`, `finalKineticEnergy`, `correctionKineticEnergy` | J per meter of depth for kg/m³ density; J for kg/m² density |
| `velocityCorrectionInnerProduct`, `divergencePotentialInnerProduct`, `residualEnergyBound`, `storageEnergyError`, `roundoffEnergyAllowance` | Same energy units, including the cell-area and density weights |

Finite tolerance allows the measured residual energy term; success alone does
not imply exact orthogonality or strict energy decrease. The roundoff allowance
is a scale-aware heuristic guard, with no absolute energy floor. Consult the
native derivation before interpreting these diagnostic pairings as physical work.

## Owned scalar-wave grids

`WaveMembrane` exposes the standalone native uniform membrane solver. Construct
an owned grid with `(width, height, spacingX, spacingY)` for native defaults, or
pass a complete configuration as a fifth argument. The default boundary is
`physics.WaveBoundary.FixedZero`; the other supported value is
`physics.WaveBoundary.Periodic`.

```javascript
const defaults = new physics.WaveMembrane(3, 3, 1, 1);
const config = defaults.getConfig();
defaults.delete();
config.boundary = physics.WaveBoundary.Periodic;
config.tension = 12;          // N/m
config.surfaceDensity = 3;    // kg/m²; wave speed is sqrt(tension/density).
config.damping = 0.1;         // gamma in 1/s; PDE damping is -2*gamma*velocity.
const wave = new physics.WaveMembrane(8, 6, 0.1, 0.2, config);
let snapshot;
try {
    const displacement = Array(48).fill(0);
    const velocity = Array(48).fill(0);
    displacement[2 * 8 + 3] = 0.001; // Row-major: y*width+x, metres.
    wave.setState(displacement, velocity);
    wave.queueAcceleration(3, 2, -0.2); // m/s², held during the next accepted step.
    wave.step(0.01);
    snapshot = wave.getCell(3, 2); // {displacement, velocity, queuedAcceleration}
    console.log(snapshot, wave.getDiagnostics());
} finally {
    wave.delete();
}
console.log(snapshot); // Plain copied values remain safe after deletion.
```

`getWidth`, `getHeight`, `getSpacingX`, `getSpacingY` and `getCellCount` describe
the immutable geometry. `getCell(x,y)` returns a copied scalar snapshot.
`getDisplacements`, `getVelocities` and `getQueuedAccelerations` return fresh
plain JS arrays in row-major order, with double-precision values. These are
copies rather than WASM-memory views; mutation does not modify native state,
and arrays/snapshots require no `delete()`.

`setState(displacements, velocities)` accepts two plain JS arrays, each exactly
width*height long with numeric finite entries. Both lengths are checked before
native array allocation, and input values are copied. Typed arrays and array-like
objects are currently rejected. `setCellState(x,y,displacement[,velocity])`
updates one cell; omitted velocity defaults to zero. `queueAcceleration` adds
to the pending cell load, `clearAcceleration(x,y)` clears one and
`clearAccelerations()` clears all. Displacement and velocity use metres and m/s.

Use the complete plain object returned by `getConfig` for `setConfig` or the
configured constructor. It contains `tension`, `surfaceDensity`, `damping`,
`boundary`, `cflSafety`, `maxSubstep`, `maxCells`, `maxSubsteps` and `maxCellWork`.
Dimensions, coordinates and budget counts arrive as doubles and must be exact
finite nonnegative integers within the WASM integer range before conversion.
Coordinates must refer to existing cells; grid/boundary minima and native budget
limits still apply. Fractional, negative, nonfinite and wrapping counts are
rejected. Invalid state/configuration changes preserve existing state.

Fixed-zero grids include their boundary nodes and require zero displacement,
velocity and loads on every edge. Periodic grids omit duplicated endpoints;
their periods are width*spacingX and height*spacingY. `step(0)` retains loads,
resets last-work counters and does not advance time. Failed positive steps retain
state, queued acceleration, time and prior diagnostics; accepted positive steps
consume acceleration. `getStableTimeStep` reports the CFL/configuration bound.
`getDiagnostics` copies physical kinetic/strain/total energy, maximum absolute
state values, time, stable timestep, last substep size/count and grid-cell work.
The Verlet integrator does not conserve physical energy exactly.

The object owns its grid independently of `Engine`/`World`; no automatic rigid,
fluid or multiphysics coupling is provided. Delete the owned grid once when
finished. See [the native membrane model, CFL, resources and representability
limits](wave-membranes.md) for the supported numerical regime.

## Periodic TMz Maxwell fields

`MaxwellGrid` owns a standalone homogeneous, lossless, source-free periodic TMz
grid. `new physics.MaxwellGrid()` uses the complete default configuration below;
the configured constructor requires every field. Configuration stays immutable.
Counts arrive as JavaScript numbers and must be positive finite integers within
the native hard ceilings before conversion: 262144 cells, 1000000 substeps and
1000000000 cell visits. Both dimensions must be at least two.

```js
const grid = new physics.MaxwellGrid({
    columns: 16, rows: 16, spacingX: 1, spacingY: 1,
    permittivity: 1, permeability: 1, cflSafety: 0.9, maxSubstep: 0.1,
    maximumSubsteps: 10000, maximumCellVisits: 100000000
});
const fields = grid.getState(); // plain copied {ez: [], hx: [], hy: []}
fields.ez[0] = 1;
grid.setState(fields);
const h = grid.getStableTimeStep();
const invariantBefore = grid.getModifiedEnergy(h);
grid.step(h);
const diagnostics = grid.getDiagnostics();
const divergence = grid.getMagneticDivergence(); // plain copied array
const retained = grid.getState();
grid.delete(); // snapshots remain usable; they need no delete()
```

All fields are synchronous in time. At index `i + columns*j`, Ez is at
`(i*dx,j*dy)`, Hx at `(i*dx,(j+.5)*dy)` and Hy at `((i+.5)*dx,j*dy)`.
Periodic samples are stored once without duplicate end rows/columns. SI fields
are Ez in V/m and Hx/Hy in A/m, spacings in meters, permittivity in F/m and
permeability in H/m. Defaults eps=mu=1 use reduced units. Energy diagnostics are
J per meter of out-of-plane depth. `getMagneticDivergence()` and diagnostic
`magneticDivergenceRms`/`maxAbsMagneticDivergence` measure **div H**, in A/m².
The physical constraint is `div B = mu div H = 0` for uniform permeability;
initial divergence is observed and preserved rather than projected away.

`getConfig()`, `getState()`, `getDiagnostics()` and `getMagneticDivergence()`
return owned plain values. Input state requires three plain dense JavaScript
arrays of exactly `columns*rows` finite numeric entries; typed arrays and sparse
arrays are rejected. All three lengths are checked before allocating/copying
any field. Mutating input or output arrays never aliases the grid. A successful
`setState()` preserves the clock and resets previous step work/reference h.
Invalid state or positive-step failure preserves all fields, time and
diagnostics. `step(0)` is a complete no-op.

`getWaveSpeed()` returns `1/sqrt(eps*mu)`. `getStableTimeStep()` combines the
strict 2D CFL bound and configured maximum duration. `step(dt)` partitions dt
within bounded substep/cell work; accepted arithmetic visits are exactly
`columns*rows*(3*lastSubsteps+1)`, excluding bounded copies/input/getter scans.
Diagnostics copy electric/magnetic/total physical energy, modified energy and
its reference step, all three means/maxima, div H RMS/max, clock, stable duration
and previous actual substep/count/cell visits. Physical energy oscillates.
`getModifiedEnergy(h)` observes the fixed-h invariant; positive h must meet the
strict physical CFL and coefficient representability requirements. Compare
the same h before/after a run; changing h changes this quadratic form.

Delete the grid exactly once when finished. It is independent of Engine/World;
there are no charge/current sources, particle coupling, interfaces, absorbing
boundaries, conductors or 3D components. See [the native equations, invariant,
resource accounting and representability limits](maxwell-grids.md).

## Prerequisites

Install and activate the Emscripten SDK, then make sure `emcmake` and `cmake`
are available in the current PowerShell session.

## Build and verify

```powershell
.\build-wasm.ps1
node .\build-wasm\wasm\smoke-test.cjs
```

The build produces `physics_engine.js` and `physics_engine.wasm` in
`build-wasm/wasm`. It also copies a browser smoke test into that directory.
Serve the directory over HTTP and open `index.html` to run it:

```powershell
emrun .\build-wasm\wasm\index.html
```

## JavaScript API

The module exposes `Engine`, `Circle`, `Polygon`, `RigidBody`, `ParticleSystem`,
`Material`, and `Vector2`. Use `createRigidBody` and `createParticleSystem` to
create the shared handles expected by the corresponding `Engine` methods.

```javascript
const physics = await createPhysicsEngineModule();
const engine = new physics.Engine();
const simulationConfig = engine.getSimulationConfig();
simulationConfig.solverIterations = 16;
simulationConfig.fixedTimeStep = 1 / 120;
simulationConfig.maxSubstepsPerAdvance = 8;
simulationConfig.enableLinearVelocityLimit = false;
engine.setSimulationConfig(simulationConfig);
const shape = new physics.Circle(1);
const body = physics.createRigidBody(
    shape,
    {
        density: 1,
        restitution: 0.5,
        staticFriction: 0.6,
        dynamicFriction: 0.4,
    },
    { x: 0, y: 0 },
    false,
);

body.setVelocity({ x: 3, y: 0 });
body.setCollisionCategoryBits(0x00000001);
body.setCollisionMaskBits(0x00000006);
engine.addBody(body);
engine.step(0.5);
console.log(body.getPosition());

// For variable frame time, prefer the backlog-preserving fixed-step runner.
const progress = engine.advance(frameTimeSeconds);
console.log(progress.stepsPerformed, progress.remainingTime);

engine.delete();
body.delete();
shape.delete();
```

Simulation iteration/substep/CCD counts must be positive integers within the
native signed 32-bit range. Particle-system indices and reserve capacities must
be nonnegative integers within the native unsigned 32-bit range; indices must
also identify an existing particle. Fractional, non-finite and overflowing
values throw before mutation instead of truncating or wrapping. `reserve(0)`
is valid. Configuration getters retain the same plain object fields.

Collision filtering uses 32-bit category and mask fields. Two bodies collide
only when each body's category is included in the other body's mask. New bodies
default to category `0x00000001` and mask `0xFFFFFFFF`, preserving the original
collide-with-everything behavior.

`engine.stepFixed()` performs exactly one configured fixed step.
`engine.advance(elapsedTime)` caps work at `maxSubstepsPerAdvance` and returns a
`FixedStepResult`; any excess time remains available through
`engine.getAccumulatedTime()` and is processed by later calls. The runner never
silently discards elapsed time.
Because the cumulative counter is 64-bit, `engine.getTotalStepCount()` returns
a JavaScript `BigInt` (for example, `120n`). Per-call `stepsPerformed` remains a
regular number.

After `step`, `stepFixed`, or `advance`,
`engine.getLastStepStatistics()` returns per-step integration, broad-phase,
contact, solver, and fluid counters. See `docs/simulation-statistics.md` for the
counting and reset semantics.

`RigidBody` owns a cloned shape. The JavaScript shape handle may be deleted
immediately after `createRigidBody`. World and joint shared handles keep bodies
alive independently of JavaScript handles. Delete each JavaScript handle once.
`engine.removeBody(body)` also removes attached joints; `engine.clearBodies()`
removes all bodies and joints.

`createDistanceJoint(a, b, length, localAnchorA, localAnchorB)` and
`createRevoluteJoint(a, b, localAnchorA, localAnchorB)` return shared joint handles
for `engine.addJoint`/`removeJoint`. CCD and waking use `body.setCcdEnabled`,
`isCcdEnabled`, `wake` and `isAwake`. Sleeping is configured through the object
returned by `getSimulationConfig`. Use that complete object when changing fields.
`engine.exportJson(time)` and `engine.exportCsv(time)` return state/statistics text.
Fluid solvers and collision listener subclasses currently have native C++ APIs only.

The revolute factory returns a `RevoluteJoint` handle extending `Joint` with
`setMotor(enabled, speed, maxTorque)`, `setLimits(enabled, lower, upper)`,
`getAngle()` and `getMotorTorque()`. Motor speed uses radians/second and limits
use radians relative to the construction pose. Query settings with
`isMotorEnabled`, `getMotorSpeed`, `getMaxMotorTorque`, `areLimitsEnabled`,
`getLowerLimit` and `getUpperLimit`. The handle remains accepted by
`engine.addJoint` and `engine.removeJoint`; delete it once when finished.
See [joint behavior and limitations](joints-and-sleeping.md).

`createPrismaticJoint(a, b, localAxisA, localAnchorA, localAnchorB)` returns a
`PrismaticJoint` shared handle accepted by the same engine joint methods. It
supports `setMotor(enabled, speed, maxForce)`, `setLimits(enabled, lower, upper)`,
their setting getters, `getMotorForce`, `getTranslation`, `getTranslationSpeed`,
`getTransverseError`, `getAngle`, `getReferenceAngle`, `getAxis` and `getLocalAxis`.
Translation uses signed world distance along A's axis; limits do not subtract
the construction translation. See [slider constraints and controls](prismatic-joints.md).

`ChargedParticle` independently integrates a test charge in prescribed uniform
fields. Use either its default neutral constructor or all four explicit arguments:

```javascript
const charge = new physics.ChargedParticle({x: 0, y: 0}, {x: 2, y: 0}, 3, 6);
charge.step(0.1, {electric: {x: 0, y: 0}, magnetic: 2});
console.log(charge.getPosition(), charge.getVelocity(), charge.getKineticEnergy());
charge.delete();
```

`getPosition` and `getVelocity` return independent plain JavaScript values with
double-precision coordinates. `setState(position, velocity)` validates both values.
Supply the field argument on every `step` call; zero fields give free motion.
The object is independent of `Engine` and owns no borrowed handles. See
[the physical scope, units and numerical limits](electromagnetic-particles.md).

## Owned soft-body simulations

`SoftBody` is an independent mass-spring simulation, with default construction
or a complete configuration object. It does not join `Engine` storage or
automatically collide/couple with rigid bodies, particles or fluids.

```javascript
const defaults = new physics.SoftBody();
const config = defaults.getConfig();
defaults.delete();
config.maxSubstep = 0.001;
const cloth = new physics.SoftBody(config);
try {
    const a = cloth.addParticle({x: 0, y: 0}, {x: 0, y: 0}, 1, true);
    const b = cloth.addParticle({x: 1.2, y: 0}, {x: 0, y: 0}, 1, false);
    cloth.addSpring(a, b, 1, 4, 0);
    cloth.applyForce(b, 0, -2);
    cloth.step(0.1);
    console.log(cloth.getParticle(b), cloth.getSpring(0), cloth.getDiagnostics());
} finally {
    cloth.delete();
}
```

Use `getParticleCount`/`getSpringCount` and `getParticle(index)`/`getSpring(index)`
to inspect topology. They return plain copied JS objects, including nested
position, velocity and force values; snapshots require no `delete()` and remain
safe after the simulation is deleted. `getConfig`, `getUniformAcceleration`,
`getAccumulatedForce(index)` and `getDiagnostics` also return copies.

`addParticle(position)` supplies native defaults; its full form takes position,
velocity, mass and fixed status. `addSpring(first, second, restLength, stiffness)`
defaults damping to zero, or accepts damping as a fifth argument.
`setParticleState(index, position[, velocity])`, `setFixed`, `applyImpulse`,
`setUniformAcceleration` and `setConfig` retain native validation. `applyForce`
accepts either `(index, {x, y})` or `(index, doubleX, doubleY)`; `clearForces()`
clears all pending forces and `clearForces(index)` clears one node.

Indices and configuration counts must be finite exact nonnegative integers
within the WASM index range; indices must also refer to existing elements.
Fractional, negative, nonfinite and wrapped values are rejected before any cast.
Native configuration, topology and step budgets remain in force. Modify a
complete object returned by `getConfig` when changing settings. A failed step
preserves state and queued loads; `step(0)` retains loads without integration,
and a successful positive step consumes them. See [native units and numerical
scope](soft-bodies.md). Delete each owned simulation once when finished.

## Owned thermal networks

`ThermalNetwork` is a standalone heat-capacity graph, independent of `Engine`
storage. It does not automatically heat rigid bodies, deform a soft body or
couple to fluids. Use default construction or a complete configuration object:

```javascript
const heat = new physics.ThermalNetwork();
try {
    const hot = heat.addNode(400, 2); // Kelvin, J/K; fixed defaults to false.
    const cold = heat.addNode(300, 3, false);
    heat.addLink(hot, cold, 1); // W/K
    heat.applyPower(cold, 4); // W, queued for the next positive step.
    heat.step(0.1);
    console.log(heat.getNode(cold), heat.getLink(0), heat.getDiagnostics());
    const config = heat.getConfig();
    config.maxSubstep = 0.005;
    heat.setConfig(config);
    // new physics.ThermalNetwork(config) also constructs a configured graph.
} finally {
    heat.delete();
}
```

Topology is append-only: `getNodeCount`/`getLinkCount` and `getNode(index)`/
`getLink(index)` expose copied snapshots. `setTemperature(index, kelvin)`,
`setFixed(index, fixed)`, `applyPower(index, watts)`, `clearPowers()` and
`clearPowers(index)` preserve native state/load validation. `getDiagnostics`
reports energies, thermostat heat accounting, temperature bounds and accepted
substeps. `getConfig` returns a complete copied object for `setConfig` or the
configured constructor. All getters produce plain JS values with no borrowed
references, vector wrappers or snapshot cleanup; they remain safe after deletion.

The same checked integer input rules and native budgets as `SoftBody` apply.
A failed step preserves temperatures, pending powers and heat accounting.
Zero time retains pending powers; a successful positive step consumes them,
including loads on fixed-temperature nodes. Thermostats track the compensating
reservoir heat separately. See [thermal units, conservation and numerical
scope](thermal-networks.md). Delete each owned network once when finished.

## N-body gravity

`NBodyGravity` owns a standalone double-precision planar gravity simulation.
It has default and full-configuration constructors. The default `G=1` uses
reduced units; consult [native gravity](nbody-gravity.md) for physical scope,
Plummer softening, integration accuracy and encounter/work limits.

```javascript
const orbit = new physics.NBodyGravity();
const gravityConfig = orbit.getConfig();
gravityConfig.maxSubstep = 0.002;
orbit.setConfig(gravityConfig);
orbit.addParticle({x: -0.5, y: 0}, {x: 0, y: -Math.SQRT1_2}, 1);
orbit.addParticle({x:  0.5, y: 0}, {x: 0, y:  Math.SQRT1_2}, 1);
orbit.step(0.5);
const first = orbit.getParticle(0);
const metrics = orbit.getDiagnostics();
orbit.delete();
console.log(first.position, metrics.totalEnergy); // Copied values remain valid.
```

`addParticle(position)` defaults to zero velocity and unit mass. The other
form is `addParticle(position, velocity, mass)`. Use `getParticleCount()`,
`getParticle(index)`, `setState(index, position, velocity)`,
`applyImpulse(index, impulse)`, `getConfig()`, `setConfig(config)`,
`getDiagnostics()` and `step(dt)` for interaction. Index and count-budget
arguments must be finite nonnegative integers in the native index range;
zero budgets are rejected. Snapshots are copied plain values, including nested
position/velocity/diagnostic vectors. Failed steps preserve particles and the
previous successful work counters. No Engine registration or implicit World
coupling is involved. Delete the owned simulation handle when finished.
