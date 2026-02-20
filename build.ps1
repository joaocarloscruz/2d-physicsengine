# Build script for 2D Physics Engine
# Requires: MinGW installed (gcc.exe in PATH, or set MINGW_BIN env var, or in common paths), CMake in PATH

# Ensure we're in the script's directory
Set-Location $PSScriptRoot

# Add MinGW to PATH dynamically
if (!(Get-Command gcc -ErrorAction SilentlyContinue)) {
    $mingwBin = $null
    if ($env:MINGW_BIN) {
        $mingwBin = $env:MINGW_BIN
    } else {
        $mingwPaths = @("C:\Programming\mingw64\bin", "C:\mingw64\bin", "C:\MinGW\bin", "C:\msys64\mingw64\bin", "C:\msys64\mingw32\bin")
        foreach ($path in $mingwPaths) {
            if (Test-Path (Join-Path $path "gcc.exe")) {
                $mingwBin = $path
                break
            }
        }
    }
    if ($mingwBin) {
        $env:Path += ";$mingwBin"
    } else {
        Write-Host "MinGW not found. Please set MINGW_BIN environment variable or ensure gcc.exe is in PATH."
        exit 1
    }
}

# Create build directory
if (!(Test-Path build)) {
    mkdir build
}

# Go to build directory
cd build

# Configure
Write-Host "Configuring build..."
cmake -G "MinGW Makefiles" ..

# Build
Write-Host "Building project..."
cmake --build .

# Run tests
Write-Host "Running tests..."
.\run_tests.exe

# Return to root 
cd ..