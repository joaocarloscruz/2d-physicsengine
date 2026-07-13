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

The module exposes `Engine`, `Circle`, `Polygon`, `RigidBody`, `Material`, and
`Vector2`. Use `createRigidBody` to create the shared body handle expected by
`Engine.addBody`.

```javascript
const physics = await createPhysicsEngineModule();
const engine = new physics.Engine();
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
engine.addBody(body);
engine.step(0.5);
console.log(body.getPosition());

engine.delete();
body.delete();
shape.delete();
```

`RigidBody` currently refers to its `Shape` through a non-owning pointer. Keep
the JavaScript shape object alive for as long as its body exists, and delete the
engine and body before deleting the shape.
