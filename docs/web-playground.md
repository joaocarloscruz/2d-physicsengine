# Web playground

`examples/web-playground/` is a static page that runs the WebAssembly build in a
browser: ten scenes, mouse dragging, spawning, pause and single-step. It has no
bundler, framework or npm dependency. See [WebAssembly](webassembly.md) for the
module itself.

## Build and serve

Build the WebAssembly target as described in [WebAssembly](webassembly.md), then
serve the output folder over HTTP. Browsers will not load a `.wasm` file from
`file://`.

```sh
emcmake cmake -S . -B build-wasm -DCMAKE_BUILD_TYPE=Release
cmake --build build-wasm --target physics_engine_wasm --parallel 2
cd build-wasm/wasm
python -m http.server 8000
```

Open <http://localhost:8000/playground.html>. The build copies `playground.html`,
`playground.js`, `scenes.js` and `scenes-test.cjs` next to `physics_engine.js`, and
refreshes them whenever the sources change, without relinking the module. If the
module cannot be loaded the page shows a message instead of a blank canvas.

## Test

`scenes-test.cjs` runs in Node and needs a build with
`-DPHYSICS_WASM_BOUNDARY_TEST_PROBES=ON`, like the other WebAssembly tests:

```sh
node build-wasm/wasm/scenes-test.cjs
```

For each of the ten scenes it builds the scene, runs 600 fixed steps and asserts
that nothing throws, every pose and joint anchor is finite, no dynamic body leaves
the arena, and the Tower and Pyramid top bodies end within 5 cm of where they
started. It also drives spawning, picking, dragging with a `MouseJoint` (the joint
and its ground body are deleted when the drag ends), the `advance()` frame loop and
culling, and checks that every Embind handle it created is deleted and that the
native heap, the live test-probe objects and the retained JavaScript values
(`count_emval_handles`) are unchanged after repeating all scenes. Body and joint
counts must be the same on every repeat.

Two scene tests go further:

- **Car and sensors:** with no throttle the car does not roll (the idle brake holds
  it), with full throttle the rear motor drives it forward over the bumps into the
  goal, the engine's sensor Begin event counts one entry, staying inside does not
  count again, reversing out raises the End event, driving back in counts a second
  entry, a `queryCircle` cover of the car agrees with the events on both sides, the
  suspension stays inside its stroke, and dragging the chassis leaves the body and
  joint counts alone.
- **Joints gallery:** the weld segments stay together (separation under 10 cm
  during the impact, under 0.05 rad bend once settled), the soft weld swings, the
  rope never stretches past 4 m and goes slack, the spring changes length, and the
  suspension rig travels inside its stops.

## Controls

| Input | Action |
| --- | --- |
| Left drag on a body | Drag it with a `MouseJoint` (5 Hz spring, force cap 1000 times the body mass) |
| Left drag on empty space, middle or right drag | Pan the camera |
| Mouse wheel | Zoom at the cursor (10 to 200 pixels per metre) |
| Arrow keys | Pan (in the car scene Left/Right drive instead; `Shift` + arrows pan) |
| `Left`/`Right` or `A`/`D` | Car scene: drive left or right with the rear-wheel motor |
| `1` to `9`, `0` | Load scene 1 to 9, or scene 10 with `0`, and reset the camera |
| `R` | Rebuild the current scene |
| `Space`, `N` | Pause or resume; single-step (pauses first) |
| `C`, `B`, `K` | Drop a circle, box or capsule at the cursor |
| `Tab`, `D`, `H` | Toggle the HUD (`D` does not in the car scene, where it drives); toggle the help panel |

The toolbar and scene buttons do the same. The scenes are the ones in the native
visualizer (`visualization/scenes.cpp`): Playground, Tower, Pyramid, Dominoes,
Chain and pendulum, Capsules and ramps, Seesaw and mass ratio, Machines, Car and
sensors, Joints gallery.

The HUD shows the scene, body and joint counts, a smoothed wall-clock cost of one
fixed step in milliseconds, and the frame rate. The car scene adds a line with the
goal entry count (and `[CAR IN GOAL]` while overlapping) and the car speed. The body count includes static
bodies and the invisible static anchors that scenes use as joint pivots.

## How it steps

`scenes.js` contains the scene builders and the `Scene` class. It has no DOM
dependency, so the page and the Node test share it. Each frame calls
`Scene.advanceFrame(seconds)`:

- The frame time is clamped to 0.1 s, as in the native visualizer.
- The scene calls `engine.advance()` once per fixed step, passing exactly the time
  that completes the next step, and the engine keeps the remainder. This lets the
  drag spring and the Machines slider controller run before every step, which
  matters because applied forces are cleared after each step. The `alpha` it
  returns is the `interpolationAlpha` of the last `advance` call.
- Gravity is a universal force added with `addGravity(engine, {x, y})`.

The JavaScript API exposes no previous body poses. The scene therefore copies each
dynamic body's pose just before every step, and the renderer blends that copy with
the current pose using `alpha`. Joint lines are drawn from the same blended poses
and the joints' local anchors.

Dragging creates a `MouseJoint` (`createMouseJoint`) from a ground body to the
grabbed body at the local point that was grabbed: 5 Hz, damping ratio 0.9 (the
engine's implicit solve adds damping, so this settles like a critically damped
spring), with the force capped at 1000 times the body mass, as in
`visualization/renderer.cpp`. The joint needs a static body that is already in the
engine, so `beginDrag` creates a tiny static circle that collides with nothing and
`endDrag` removes both. Neither appears in the scene's body or joint lists, so the
counts and drawing do not change while dragging. The pointer only calls
`setTarget`; the engine applies the pull on every step. Dragging a body that is
asleep wakes it.

## Car and sensors, and the joints gallery

Scene 9 and scene 10 use the joints and sensors added after the original eight
scenes. Their layout mirrors the native scenes.

The goal is a static body with `setSensor(true)` (and a user data tag). The scene
turns on the engine's [collision events](webassembly.md#collision-events) and, after
every fixed step, takes the buffer with `takeCollisionEvents()` and passes each event to
its handlers. The goal handler keeps the set of car parts (the chassis and the two
wheels) that are inside: a sensor Begin or Persist event between the goal and a car part
adds it, a sensor End removes it. The first part in is one entry, and the last part out
is one exit. These are the same events a native `ICollisionListener` sees. The list is
an owned collection and is deleted in a `finally` block after every step. A buffer
overflow throws instead of silently losing an entry.

The car's chassis and wheels are joined by `createWheelJoint` with a spring
(3 Hz, damping ratio 0.7) and a 0.3 m stroke limit. The rear wheel's motor runs at
up to 16 rad/s with 16 N m, and holds with a small torque when no key is pressed.
The page sets `scene.input.throttle` from the keys, and the scene's pre-step
controller applies it, so it works the same under `advance()`, single-step and the
Node test.

The gallery uses `createWeldJoint` (rigid, and one with `setAngularSpring`),
`createDistanceJoint` with `setLengthRange(0, 4)` (rope) and `setSpring` (spring)
and `createWheelJoint` with a spring and limits. Joints are drawn like in the native
visualizer: welds pink with hexagon markers, wheel joints cyan with a bar for the
stroke, springs as a green zig-zag, ropes tan. Sensors are translucent green and
turn amber while the car is in the goal.

## Handle lifetimes

A `Scene` owns every handle it creates: the engine, each body and joint, and the
temporary shapes (deleted right after `createRigidBody`). `dispose()` deletes them
all, and the page calls it on every scene change and reset. `queryPoint` results
are owned collections; `pick` reads only the `BigInt` body ids and deletes the
collection in a `finally` block, so it creates no body handle. Spawned bodies that
get farther than 60 m from the origin are removed with `engine.removeBody` and
their handle is deleted.

## Limits

- Rendering, mouse and keyboard handling have only been run against a stubbed DOM
  in Node, not checked visually in a browser.
- There is no pinch zoom; on touch screens one finger drags a body or pans.
- At most 500 bodies exist at once; further spawns are ignored.
- Dragging works only while the simulation runs; a paused scene does not move.
- Below about 10 fps the simulation runs slower than real time instead of
  catching up.
- `addGravity` stacks and cannot be undone on an engine, so each scene starts from
  a fresh `Engine`.
