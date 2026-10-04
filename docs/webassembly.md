# WebAssembly build

The WebAssembly target exposes the engine's basic simulation API to JavaScript
through Emscripten's Embind library.

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
