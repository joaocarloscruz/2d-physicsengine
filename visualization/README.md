# Physics Engine Visualization

This separate SFML application displays an interactive 2D physics simulation.
See [QUICKSTART](QUICKSTART.md) for the engine installation, visualizer build,
PowerShell helper and hidden `--smoke-test capture.png` render check.

## Dependencies

- SFML 3 with static Graphics, Window and System libraries
- Installed PhysicsEngine 1.0 package
- CMake 3.16+ and a compatible C++ compiler/runtime for both packages

The visualizer consumes the installed engine through `PhysicsEngine::Engine`.
Building the engine alone does not install its CMake package. Pass both install
prefixes in `CMAKE_PREFIX_PATH`; `SFML_DIR`, if used, must point to the directory
containing `SFMLConfig.cmake`, typically `<prefix>/lib/cmake/SFML`.

## Build SFML locally

Download the official [SFML 3.1.0 source](https://github.com/SFML/SFML/releases/tag/3.1.0)
and build it with the same compiler/generator as the engine and visualizer.
SFML 3.1 requires CMake 3.28+. This example installs only into a local
prefix and selects the static libraries expected by the visualizer:

```sh
cmake -S /path/to/SFML -B /path/to/sfml-build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DSFML_BUILD_AUDIO=OFF -DSFML_BUILD_NETWORK=OFF -DSFML_BUILD_EXAMPLES=OFF -DSFML_BUILD_TEST_SUITE=OFF
cmake --build /path/to/sfml-build --config Release --parallel 2
cmake --install /path/to/sfml-build --config Release --prefix /path/to/sfml-install
```

On Linux, the Graphics/Window modules need development packages for X11,
Xrandr, Xcursor, Xi, udev, OpenGL, FreeType and HarfBuzz. Distribution `libsfml-dev`
packages that provide SFML 2 do not satisfy this project. Consult SFML's
[source build instructions](https://www.sfml-dev.org/tutorials/3.0/getting-started/build-from-source/)
for your platform. CI builds pinned SFML 3.1.0 on Windows and Linux and checks
the screenshot under Xvfb on Linux.

## Controls

| Key/Button | Action |
|------------|--------|
| 1-9, 0 (number row or keypad) | Switch to that scene; 0 is scene 10 (see below) |
| R | Reset the current scene |
| Left Mouse + Drag | Pick up and move objects with a `MouseJoint` (5 Hz spring, force capped at 1000 times the body mass) |
| Right Mouse Click / B | Spawn a box at the cursor |
| Middle Mouse / C | Spawn a circle at the cursor |
| K | Spawn a capsule at the cursor |
| Space | Pause/unpause simulation |
| H | Show/hide the controls list in the HUD |
| Tab (or D, except in the car scene) | Show/hide the whole HUD |
| Arrow keys | Move the camera (in the car scene only Up/Down; Shift+arrows pan in any direction) |
| Left/Right or A/D | Car scene only: drive left or right with the rear-wheel motor |
| Mouse Wheel | Zoom in/out |
| ESC | Exit |

The simulation always advances with a fixed 60 Hz step. D drives in the car scene,
so Tab is the key that hides the HUD everywhere.

Joints are drawn between their world anchors. Revolute, prismatic and plain distance
joints are thin yellow lines, with a round marker for revolute joints and a square
marker for the other two. The newer joints have their own look: weld joints are pink with hexagon markers, wheel joints are cyan with round
markers and a translucent bar showing the suspension stroke, a distance joint with a
spring is a green zig-zag, and a rope (a distance joint with a length range and no
spring) is a tan line. While you drag, the `MouseJoint` is a yellow line from the
grab point to the cursor.

Sensor bodies are drawn as translucent green zones with a green outline. A scene can
mark a sensor as active, and then it turns amber (the car scene does this while the
car overlaps its goal).

Dragging uses a `MouseJoint`. It needs a static body that is already in the world,
so the visualizer adds a tiny static circle that collides with nothing when you grab
a body and removes it, with the joint, when you let go. The HUD body and joint counts
leave them out. The joint asks for 5 Hz and a damping ratio of 0.9. The engine solves
the spring implicitly and that adds damping at 60 Hz, so 0.9 settles the way the old
hand-written critically damped spring did: a box grabbed at its centre and pulled
5 m reaches 4.88 m after 0.2 s, against 4.86 m for the old spring. Grabbed by a corner
and pulled 2 m, a box overshoots the target by about 0.1 m with the joint and about
0.5 m with the old spring. The force cap stays at
1000 times the body mass (about 100 m g).

## HUD

The top-left panel shows the scene name, body and joint counts, the smoothed
duration of `World::step` in milliseconds, the frame rate and whether the
simulation is paused, plus extra lines a scene supplies (the car scene's goal
counter and speed); a two-row bar at the bottom lists the scenes with the current
one in brackets. Text needs a font file because SFML 3 has none built in. The
visualizer looks for Segoe UI or Arial on Windows, DejaVu Sans or Liberation
Sans on Linux and Arial or Helvetica on macOS. If none is found it draws no
text and shows the scene name in the window title.

## Scenes

Every scene is built in `scenes.cpp` from the public engine API only.

| Key | Scene | What it shows |
|-----|-------|---------------|
| 1 | Playground | Two boxes and a ball to spawn onto and drag around |
| 2 | Tower | Twelve boxes stacked with small offsets |
| 3 | Pyramid | A 36-box pyramid (eight at the base) held up by friction |
| 4 | Dominoes | Twenty thin boxes toppled by a ball rolled into the first |
| 5 | Chain and pendulum | A capsule chain on revolute joints hanging from a pivot, and a distance-joint pendulum swinging into a stack of boxes |
| 6 | Capsules and ramps | Capsules and balls sliding down angled static box and capsule ramps |
| 7 | Seesaw and mass ratio | A plank on a limited revolute hinge: a heavy ball dropped on one end throws light boxes off the other |
| 8 | Machines | A revolute paddle and a revolute roller driven by motors, and a prismatic slider with motor and travel limits that shuttles back and forth pushing boxes |
| 9 | Car and sensors | A chassis on two `WheelJoint` suspensions (3 Hz, damping 0.7, 0.3 m stroke) with a motor on the rear wheel, driven with Left/Right or A/D over bumps and a hill. A static sensor goal on the right turns amber while the car overlaps it, and the HUD counts entries and shows the speed. The scene registers a collision listener and counts a sensor `Begin` event when the first of the chassis or either wheel enters the goal |
| 10 (key 0) | Joints gallery | A cantilever of five welded segments with a ball dropped on its tip, a soft weld (angular spring) beam, a `DistanceJoint` rope with `setLengthRange(0, 4)`, a `DistanceJoint` spring box released stretched, and a wheel-joint suspension rig whose wheel is bounced by a motorized arm |

## Command-line options

`PhysicsVisualization --smoke-test [capture.png]` renders five hidden frames,
optionally saves a PNG and exits with status 0. Three hidden options help produce
captures of a given scene; without them the program starts in scene 1 and
behaves as before.

| Option | Effect |
|--------|--------|
| `--scene N` | Start in scene N (1-10) |
| `--advance STEPS` | Simulate STEPS fixed steps before the first frame |
| `--throttle T` | Hold the car's throttle at T (-1 to 1) instead of reading the keyboard |

For example `PhysicsVisualization --scene 5 --advance 150 --smoke-test five.png`
captures the chain and pendulum scene after 2.5 simulated seconds, and
`PhysicsVisualization --scene 9 --advance 360 --throttle 1 --smoke-test goal.png`
captures the car sitting in the goal after driving there.

## Troubleshooting

- **Package not found:** verify that each prefix contains the installed CMake
  package. SFML must be version 3 with static libraries; the engine must be 1.x.
- **Compiler or link errors:** use packages built for the same compiler,
  architecture, configuration and runtime. Use a new build directory when
  changing a compiler or generator.
- **Missing runtime DLL:** the build copies a shared Engine DLL on Windows;
  make the compiler's runtime available in your current session or next to the
  executable. SFML itself is linked statically.
- **Display unavailable:** Linux smoke tests still need an OpenGL-capable
  display, such as Xvfb with Mesa software rendering.
