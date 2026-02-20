# Build script for the visualization
Write-Host "Building Physics Engine Visualization..." -ForegroundColor Cyan

# Check if SFML is available
if (-not $env:SFML_DIR -and -not $env:CMAKE_PREFIX_PATH) {
    Write-Host "WARNING: Neither SFML_DIR nor CMAKE_PREFIX_PATH is set!" -ForegroundColor Yellow
    Write-Host "Please download SFML from https://www.sfml-dev.org/download.php" -ForegroundColor Yellow
    Write-Host "and set CMAKE_PREFIX_PATH to the installation path:" -ForegroundColor Yellow
    Write-Host '  $env:CMAKE_PREFIX_PATH = "C:\path\to\SFML"' -ForegroundColor Cyan
    Write-Host ""
}

# Determine SFML path
$sfmlPath = $null
if ($env:CMAKE_PREFIX_PATH) {
    $sfmlPath = $env:CMAKE_PREFIX_PATH
} elseif ($env:SFML_DIR) {
    $sfmlPath = $env:SFML_DIR
}

# First, make sure the main engine is built
Write-Host "Building main physics engine..." -ForegroundColor Green
Push-Location ..
if (Test-Path "./build.ps1") {
    & ./build.ps1
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Failed to build main engine!" -ForegroundColor Red
        Pop-Location
        exit 1
    }
} else {
    Write-Host "Main engine build script not found!" -ForegroundColor Red
    Pop-Location
    exit 1
}
Pop-Location

# Create build directory if it doesn't exist
if (-not (Test-Path "./build")) {
    New-Item -ItemType Directory -Path "./build" | Out-Null
}

# Navigate to build directory
Push-Location "./build"

Write-Host "Configuring visualization build..." -ForegroundColor Green
if ($sfmlPath) {
    cmake .. -G "MinGW Makefiles" -DCMAKE_PREFIX_PATH="$sfmlPath"
} else {
    cmake .. -G "MinGW Makefiles"
}
if ($LASTEXITCODE -ne 0) {
    Write-Host "CMake configuration failed!" -ForegroundColor Red
    Write-Host "Make sure SFML is installed and CMAKE_PREFIX_PATH is set correctly:" -ForegroundColor Yellow
    Write-Host '  $env:CMAKE_PREFIX_PATH = "C:\path\to\SFML"' -ForegroundColor Cyan
    Pop-Location
    exit 1
}

Write-Host "Building visualization..." -ForegroundColor Green
cmake --build .
if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed!" -ForegroundColor Red
    Pop-Location
    exit 1
}

Pop-Location

Write-Host "Build successful!" -ForegroundColor Green
Write-Host "Run './build/PhysicsVisualization.exe' to start the visualization." -ForegroundColor Cyan
