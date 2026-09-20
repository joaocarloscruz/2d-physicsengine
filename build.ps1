param([string]$BuildDirectory = 'build', [string]$Configuration = 'Release')
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    if (!(Get-Command cmake -ErrorAction SilentlyContinue)) {
        throw 'CMake is required. Put CMake and a C++17 compiler on the current PATH.'
    }
    cmake -S . -B $BuildDirectory "-DCMAKE_BUILD_TYPE=$Configuration"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    cmake --build $BuildDirectory --config $Configuration --parallel
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    ctest --test-dir $BuildDirectory -C $Configuration --output-on-failure
    exit $LASTEXITCODE
} finally { Pop-Location }
