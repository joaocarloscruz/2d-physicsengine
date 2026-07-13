# Build the WebAssembly module with an activated Emscripten SDK.
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (!(Get-Command emcmake -ErrorAction SilentlyContinue)) {
    throw "emcmake was not found. Activate the Emscripten SDK with emsdk_env.ps1 first."
}

emcmake cmake -S . -B build-wasm -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

cmake --build build-wasm --target physics_engine_wasm
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

Write-Host "WebAssembly output: $PSScriptRoot\build-wasm\wasm"
Write-Host "Node smoke test: node $PSScriptRoot\build-wasm\wasm\smoke-test.cjs"
