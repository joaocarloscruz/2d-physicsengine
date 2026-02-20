# Quick Start Guide for Visualization

## Requirements

Before building the visualization, you need to install SFML:

### Windows
1. Download SFML 2.6.x or 3.x (GCC MinGW) from https://www.sfml-dev.org/download.php
2. Extract to a location (e.g., `C:\SFML-3.0.2`)
3. Set the `SFML_DIR` environment variable:
   ```powershell
   $env:SFML_DIR = "C:\SFML-3.0.2"
   # Or set it permanently in System Properties
   ```

### Linux
```bash
sudo apt-get install libsfml-dev
```

### macOS
```bash
brew install sfml
```

## Building

1. Make sure the main physics engine is built first:
   ```bash
   cd ..
   ./build.ps1
   ```

2. Build the visualization:
   ```bash
   cd visualization
   ./build.ps1
   ```

3. Run the visualization:
   ```bash
   ./build/PhysicsVisualization.exe
   ```

## Controls

| Key/Button | Action |
|------------|--------|
| **Left Mouse + Drag** | Pick up and move objects |
| **Right Mouse Click** | Spawn a new box at cursor |
| **Middle Mouse / C** | Spawn a new circle at cursor |
| **Space** | Pause/unpause simulation |
| **R** | Reset the scene |
| **D** | Toggle debug info |
| **Mouse Wheel** | Zoom in/out |
| **ESC** | Exit |

## Troubleshooting

### CMake can't find SFML
Make sure `SFML_DIR` is set correctly and points to the SFML installation directory (the one containing `lib` and `include` folders).

### Missing DLL errors
Copy the SFML DLLs from `SFML_DIR/bin` to the same directory as the executable, or add the bin directory to your PATH.

### Linking errors
Make sure the main physics engine (`libEngine.dll`) is built first and exists in `../build/`.

## Features

- **Real-time Physics**: See your 2D rigid body simulation in action
- **Interactive**: Click and drag objects around
- **Spawn Objects**: Add circles and boxes dynamically
- **Visual Feedback**: Different colors for static vs dynamic bodies
- **Rotation Visualization**: Lines show object rotation
- **Debug Info**: Body count, FPS, and control hints
