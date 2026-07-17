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

`RigidBody` currently refers to its `Shape` through a non-owning pointer. Keep
the JavaScript shape object alive for as long as its body exists, and delete the
engine and body before deleting the shape.
