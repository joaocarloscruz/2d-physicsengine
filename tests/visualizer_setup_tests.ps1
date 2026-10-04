#Requires -Version 7.0
# Run with: pwsh -File tests/visualizer_setup_tests.ps1
# A fake CMake verifies command routing and failure propagation without a compiler.
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
$setupScript = Join-Path $repoRoot 'visualization/setup-and-build.ps1'
$scratchRoot = Join-Path ([System.IO.Path]::GetTempPath()) ('visualizer-setup-' + [guid]::NewGuid())
$scratch = [System.IO.Path]::GetFullPath($scratchRoot)
$log = Join-Path $scratch 'calls.jsonl'
$fakeCMake = Join-Path $scratch 'fake cmake.ps1'
$enginePrefix = Join-Path $scratch 'engine install'
$sfmlPrefix = Join-Path $scratch 'sfml install'
$pwsh = (Get-Process -Id $PID).Path

function Assert-True($Condition, [string]$Message) {
    if (-not $Condition) { throw $Message }
}

function Invoke-Setup([string[]]$ExtraArguments, [string]$FailStage = '') {
    if (Test-Path -LiteralPath $log) { Remove-Item -LiteralPath $log }
    $start = [System.Diagnostics.ProcessStartInfo]::new($pwsh)
    $start.WorkingDirectory = $scratch
    $start.UseShellExecute = $false
    $start.RedirectStandardOutput = $true
    $start.RedirectStandardError = $true
    $start.Environment['VISUALIZER_TEST_LOG'] = $log
    $start.Environment['VISUALIZER_TEST_FAIL'] = $FailStage
    $start.Environment['CMAKE_PREFIX_PATH'] = 'existing-prefix-must-be-preserved'
    foreach ($argument in @('-NoProfile', '-File', $setupScript,
        '-EnginePrefix', $enginePrefix, '-SFMLPrefix', $sfmlPrefix,
        '-CMakeExecutable', $fakeCMake) + $ExtraArguments) {
        $start.ArgumentList.Add($argument)
    }
    $process = [System.Diagnostics.Process]::Start($start)
    $stdout = $process.StandardOutput.ReadToEndAsync()
    $stderr = $process.StandardError.ReadToEndAsync()
    $process.WaitForExit()
    $result = @{
        ExitCode = $process.ExitCode
        Output = $stdout.GetAwaiter().GetResult() + $stderr.GetAwaiter().GetResult()
        Calls = @(if (Test-Path -LiteralPath $log) {
            Get-Content -LiteralPath $log | ForEach-Object { ConvertFrom-Json $_ }
        })
    }
    $process.Dispose()
    return $result
}

try {
    New-Item -ItemType Directory -Path $enginePrefix, $sfmlPrefix | Out-Null
    @'
@{ Arguments = @($args); Prefix = $env:CMAKE_PREFIX_PATH; Path = $env:PATH } |
    ConvertTo-Json -Compress | Add-Content -LiteralPath $env:VISUALIZER_TEST_LOG
if ($args[0] -eq '-S' -and $env:VISUALIZER_TEST_FAIL -eq 'configure') { exit 37 }
if ($args[0] -eq '--build' -and $env:VISUALIZER_TEST_FAIL -eq 'build') { exit 38 }
exit 0
'@ | Set-Content -LiteralPath $fakeCMake

    $result = Invoke-Setup @('-BuildDirectory', 'build with spaces', '-CxxCompiler', $pwsh)
    Assert-True ($result.ExitCode -eq 0) "Setup failed: $($result.Output)"
    Assert-True ($result.Calls.Count -eq 2) 'Expected exactly configure and build.'
    $configure = $result.Calls[0].Arguments
    $expectedSource = Join-Path $repoRoot 'visualization'
    $expectedBuild = Join-Path $expectedSource 'build with spaces'
    Assert-True ($configure[0] -eq '-S' -and $configure[1] -eq $expectedSource) 'Setup configured the wrong source directory.'
    Assert-True ($configure[2] -eq '-B' -and $configure[3] -eq $expectedBuild) 'Relative build directory depended on caller CWD.'
    Assert-True ($configure -contains "-DCMAKE_PREFIX_PATH=$enginePrefix;$sfmlPrefix") 'Both package prefixes must be passed as one argument.'
    Assert-True ($configure -contains 'Ninja') 'Explicit compiler must default to Ninja.'
    Assert-True ($configure -contains "-DCMAKE_CXX_COMPILER=$pwsh") 'Explicit compiler was dropped.'
    Assert-True ($result.Calls[1].Arguments[1] -eq $expectedBuild) 'Build used a different directory.'
    foreach ($call in $result.Calls) {
        Assert-True ($call.Prefix -eq 'existing-prefix-must-be-preserved') 'Setup modified CMAKE_PREFIX_PATH.'
        Assert-True ($call.Path -eq $env:PATH) 'Setup modified PATH.'
    }

    $result = Invoke-Setup @() 'configure'
    Assert-True ($result.ExitCode -eq 37 -and $result.Calls.Count -eq 1) 'Configure failure was not propagated or build ran after failure.'
    $result = Invoke-Setup @() 'build'
    Assert-True ($result.ExitCode -eq 38 -and $result.Calls.Count -eq 2) 'Build failure was not propagated.'
    $result = Invoke-Setup @('-Generator', 'Visual Studio 17 2022', '-CxxCompiler', $pwsh)
    Assert-True ($result.ExitCode -ne 0 -and $result.Calls.Count -eq 0) 'Incompatible Visual Studio/compiler combination was accepted.'
    $result = Invoke-Setup @('-MakeProgram', $pwsh)
    Assert-True ($result.ExitCode -ne 0 -and $result.Calls.Count -eq 0) 'MakeProgram without a generator was accepted.'

    # Missing packages and tools should fail before invoking CMake.
    Remove-Item -LiteralPath $sfmlPrefix
    $result = Invoke-Setup @()
    Assert-True ($result.ExitCode -ne 0 -and $result.Calls.Count -eq 0) 'Missing package prefix was accepted.'
    Remove-Item -LiteralPath $fakeCMake
    $result = Invoke-Setup @()
    Assert-True ($result.ExitCode -ne 0 -and $result.Calls.Count -eq 0) 'Missing CMake was accepted.'
    Write-Host 'Visualizer setup regression checks passed.'
} finally {
    # Delete only this test's unique directory after checking its absolute path.
    $tempRoot = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath())
    if ($scratch.StartsWith($tempRoot, [System.StringComparison]::OrdinalIgnoreCase) -and
        (Split-Path $scratch -Leaf) -like 'visualizer-setup-*') {
        Remove-Item -LiteralPath $scratch -Recurse -Force
    }
}
