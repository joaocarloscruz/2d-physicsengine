# Physics Engine Visualization

This folder contains a simple SFML-based visualization for the 2D physics engine.

## Prerequisites

- SFML 2.5+ installed on your system
- CMake 3.10+

## Building

From the visualization directory:
```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Controls

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
sudo apt-get install libsfml-dev
```

### macOS
```bash
brew install sfml
```
