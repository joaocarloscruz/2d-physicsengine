# Quick setup and build script
# Automatically detects SFML installation

Write-Host "Setting up SFML path..." -ForegroundColor Cyan

# Try to auto-detect SFML
$possiblePaths = @(
    "C:/Programming/SFML-3.0.2",
    "C:/SFML-3.0.2",
    "C:/Program Files/SFML-3.0.2",
    "$env:USERPROFILE/SFML-3.0.2"
)

$sfmlFound = $false
foreach ($path in $possiblePaths) {
    if (Test-Path "$path/lib/cmake/SFML/SFMLConfig.cmake") {
        Write-Host "Found SFML at: $path" -ForegroundColor Green
        $env:CMAKE_PREFIX_PATH = $path
        $sfmlFound = $true
        break
    }
}

if (-not $sfmlFound) {
    Write-Host "Could not auto-detect SFML. Please enter the full path:" -ForegroundColor Yellow
    $userPath = Read-Host "SFML installation path"
    if ($userPath) {
        $env:CMAKE_PREFIX_PATH = $userPath
    }
}

Write-Host ""
Write-Host "Building with CMAKE_PREFIX_PATH=$env:CMAKE_PREFIX_PATH" -ForegroundColor Cyan
Write-Host ""

# Run the build
.\build.ps1
