@echo off
echo Building Physics Engine Visualization...
echo.

if "%SFML_DIR%"=="" (
    echo WARNING: SFML_DIR environment variable not set!
    echo Please download SFML from https://www.sfml-dev.org/download.php
    echo and set SFML_DIR to the installation path.
    echo.
)

echo Building main physics engine...
pushd ..
call build.ps1
if errorlevel 1 (
    echo Failed to build main engine!
    popd
    exit /b 1
)
popd

if not exist "build" mkdir build
pushd build

echo Configuring visualization build...
cmake .. -G "MinGW Makefiles"
if errorlevel 1 (
    echo CMake configuration failed!
    echo Make sure SFML is installed and SFML_DIR is set correctly.
    popd
    exit /b 1
)

echo Building visualization...
cmake --build .
if errorlevel 1 (
    echo Build failed!
    popd
    exit /b 1
)

popd

echo Build successful!
echo Run 'build\PhysicsVisualization.exe' to start the visualization.
pause
