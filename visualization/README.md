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
| Left Mouse + Drag | Pick up and move objects |
| Right Mouse Click | Spawn a new box at cursor |
| Middle Mouse / C | Spawn a new circle at cursor |
| Space | Pause/unpause simulation |
| R | Reset the scene |
| D | Toggle debug info |
| Mouse Wheel | Zoom in/out |
| ESC | Exit |

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
