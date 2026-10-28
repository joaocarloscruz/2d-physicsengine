# Web playground

`examples/web-playground/` is a static page that runs the WebAssembly build in a
browser: eight scenes, mouse dragging, spawning, pause and single-step. It has no
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

For each of the eight scenes it builds the scene, runs 600 fixed steps and asserts
that nothing throws, every pose and joint anchor is finite, no dynamic body leaves
the arena, and the Tower and Pyramid top bodies end within 5 cm of where they
started. It also drives spawning, picking, the drag spring, the `advance()` frame
loop and culling, and checks that every Embind handle it created is deleted and
that the native heap, the live test-probe objects and the retained JavaScript
values (`count_emval_handles`) are unchanged after repeating all scenes.

## Controls

| Input | Action |
| --- | --- |
| Left drag on a body | Drag it with a critically damped spring |
| Left drag on empty space, middle or right drag | Pan the camera |
| Mouse wheel | Zoom at the cursor (10 to 200 pixels per metre) |
| Arrow keys | Pan |
| `1` to `8` | Load a scene and reset the camera |
| `R` | Rebuild the current scene |
| `Space`, `N` | Pause or resume; single-step (pauses first) |
| `C`, `B`, `K` | Drop a circle, box or capsule at the cursor |
| `D`, `H` | Toggle the HUD; toggle the help panel |

The toolbar and scene buttons do the same. The scenes are the ones in the native
visualizer (`visualization/scenes.cpp`): Playground, Tower, Pyramid, Dominoes,
Chain and pendulum, Capsules and ramps, Seesaw and mass ratio, Machines.

The HUD shows the scene, body and joint counts, a smoothed wall-clock cost of one
fixed step in milliseconds, and the frame rate. The body count includes static
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

The drag spring follows `applyDragSpring` in `visualization/renderer.cpp`: 5 Hz,
critically damped, acting at the local point that was grabbed, with the force
capped at 1000 times the body mass.

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
