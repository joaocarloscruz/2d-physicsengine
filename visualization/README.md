# Physics Engine Visualization

This separate SFML application displays an interactive 2D physics simulation.
See [QUICKSTART](QUICKSTART.md) for the engine installation, visualizer build,
PowerShell helper and hidden `--smoke-test capture.png` render check.

## Dependencies

- SFML 3 with static Graphics, Window and System libraries
- Installed PhysicsEngine 0.2 package
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
| 1-8 (number row or keypad) | Switch to that scene (see below) |
| R | Reset the current scene |
| Left Mouse + Drag | Pick up and move objects with a damped spring |
| Right Mouse Click / B | Spawn a box at the cursor |
| Middle Mouse / C | Spawn a circle at the cursor |
| K | Spawn a capsule at the cursor |
| Space | Pause/unpause simulation |
| H | Show/hide the controls list in the HUD |
| D | Show/hide the whole HUD |
| Arrow keys | Move the camera |
| Mouse Wheel | Zoom in/out |
| ESC | Exit |

The simulation always advances with a fixed 60 Hz step. Joints are drawn as thin
yellow lines between their world anchors, with a round marker for revolute joints
and a square marker for distance and prismatic joints.

## HUD

The top-left panel shows the scene name, body and joint counts, the smoothed
duration of `World::step` in milliseconds, the frame rate and whether the
simulation is paused; a bottom bar lists the scenes with the current one in
brackets. Text needs a font file because SFML 3 has none built in. The
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

## Command-line options

`PhysicsVisualization --smoke-test [capture.png]` renders five hidden frames,
optionally saves a PNG and exits with status 0. Two hidden options help produce
captures of a given scene; without them the program starts in scene 1 and
behaves as before.

| Option | Effect |
|--------|--------|
| `--scene N` | Start in scene N (1-8) |
| `--advance STEPS` | Simulate STEPS fixed steps before the first frame |

For example `PhysicsVisualization --scene 5 --advance 150 --smoke-test five.png`
captures the chain and pendulum scene after 2.5 simulated seconds.

## Troubleshooting

- **Package not found:** verify that each prefix contains the installed CMake
  package. SFML must be version 3 with static libraries; the engine must be 0.2.
- **Compiler or link errors:** use packages built for the same compiler,
  architecture, configuration and runtime. Use a new build directory when
  changing a compiler or generator.
- **Missing runtime DLL:** the build copies a shared Engine DLL on Windows;
  make the compiler's runtime available in your current session or next to the
  executable. SFML itself is linked statically.
- **Display unavailable:** Linux smoke tests still need an OpenGL-capable
  display, such as Xvfb with Mesa software rendering.
