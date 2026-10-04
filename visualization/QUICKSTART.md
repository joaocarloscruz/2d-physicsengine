# Quick start for the visualizer

Use CMake 3.16+, a C++17-capable compiler, **SFML 3** with static Graphics,
Window and System libraries, and an installed **PhysicsEngine 0.2** package.
SFML 2 is incompatible. SFML 3.1's source build requires CMake 3.28+.
Build all packages with the same compiler, architecture and runtime; a prebuilt
MinGW SFML package cannot be linked into an MSVC application.

From the repository root, build and install the engine into a local directory:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel 2
cmake --install build --config Release --prefix /absolute/path/physics-install
```

Supply that prefix and your SFML 3 installation prefix to the visualizer:

```sh
cmake -S visualization -B visualization/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="/absolute/path/physics-install;/absolute/path/sfml-install"
cmake --build visualization/build --config Release --parallel 2
```

On Windows, run CMake from a compiler environment. Alternatively, the PowerShell
helper accepts portable tools and package paths explicitly, from any directory:

```powershell
& C:\src\2d-physicsengine\visualization\setup-and-build.ps1 `
    -EnginePrefix C:\deps\physics-install -SFMLPrefix C:\deps\sfml-install `
    -CMakeExecutable C:\tools\cmake\bin\cmake.exe `
    -CxxCompiler C:\tools\llvm-mingw\bin\clang++.exe `
    -Generator Ninja -MakeProgram C:\tools\ninja.exe
```

For MSVC packages, omit `-CxxCompiler` and use the matching Visual Studio
generator, for example `-Generator 'Visual Studio 17 2022'`. With an explicit
compiler the helper defaults to Ninja; otherwise CMake selects its default
generator. Relative package/build paths are based on the helper's directory.
The helper does not install dependencies or change PATH or CMAKE_PREFIX_PATH.
It returns the configure/build command's nonzero exit code on failure.

Run `visualization/build/PhysicsVisualization` (`.exe` on Windows). A
multi-configuration generator places it under `visualization/build/Release/`.
The Windows build copies a shared Engine DLL next to the executable. Your
compiler's runtime DLLs must also be available to the loader; portable toolchains
may require their `bin` directory on the current session's PATH.

For an automated render check:

```sh
visualization/build/PhysicsVisualization --smoke-test capture.png
# On a Linux machine without a display:
xvfb-run -a visualization/build/PhysicsVisualization --smoke-test capture.png
```

It renders five hidden frames, writes a PNG, and exits. See [README](README.md)
for controls, SFML source installation and troubleshooting.
