# Physics Engine Visualization

This folder contains a simple SFML-based visualization for the 2D physics engine.

## Prerequisites

- SFML 3.x built with a compatible C++ compiler/runtime
- CMake 3.16+ (building SFML 3.1 itself requires CMake 3.28+)
- The installed PhysicsEngine 0.2 package

## Building

Install the engine, then configure this directory with both package prefixes:
```bash
cmake --install build --prefix /path/to/physics-install
cmake -S visualization -B visualization/build -DCMAKE_PREFIX_PATH="/path/to/physics-install;/path/to/sfml-install"
cmake --build visualization/build --config Release
```

## Controls

For an automated startup/render check, run
`PhysicsVisualization --smoke-test capture.png`. It renders five hidden frames,
writes a screenshot, and exits. Normal startup opens an interactive window.

- **Left Click + Drag**: Move objects around
- **Right Click**: Add a new box at cursor position
- **Middle Click / C**: Add a new circle at cursor position
- **R**: Reset the simulation
- **Space**: Pause/unpause
- **ESC**: Exit

## Installing SFML

### Windows (MinGW)
Download SFML from https://www.sfml-dev.org/download.php and extract it.

Set the `SFML_DIR` environment variable to the SFML installation path.

### Linux
```bash
# Build SFML 3 from source if your distribution only packages SFML 2.
```

### macOS
```bash
brew install sfml
```
