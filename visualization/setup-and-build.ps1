<#
.SYNOPSIS
Configure and build the visualizer against installed SFML 3 and PhysicsEngine 0.2.
.DESCRIPTION
No packages are downloaded and no environment variables are changed. Relative
package and build paths are resolved against this script's directory. Use the
same compiler, architecture and runtime as both installed packages.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$SFMLPrefix,
    [Parameter(Mandatory = $true)][string]$EnginePrefix,
    [string]$BuildDirectory = 'build',
    [ValidateSet('Debug', 'Release', 'RelWithDebInfo', 'MinSizeRel')]
    [string]$Configuration = 'Release',
    [string]$Generator,
    [string]$CxxCompiler,
    [string]$MakeProgram,
    [string]$CMakeExecutable = 'cmake',
    [ValidateRange(1, 1024)][int]$Parallel = 2
)

$ErrorActionPreference = 'Stop'
# Handle native failures explicitly, including when a PowerShell 7 caller has
# enabled automatic native-command errors.
$PSNativeCommandUseErrorActionPreference = $false

function Get-ScriptPath([string]$Path) {
    if ([System.IO.Path]::IsPathRooted($Path)) {
        return [System.IO.Path]::GetFullPath($Path)
    }
    return [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot $Path))
}

function Invoke-CMake([string[]]$Arguments) {
    & $script:cmakeCommand @Arguments
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

try {
    $cmakeCommand = (Get-Command $CMakeExecutable -CommandType Application, ExternalScript -ErrorAction Stop).Source
    $sfmlPath = Get-ScriptPath $SFMLPrefix
    $enginePath = Get-ScriptPath $EnginePrefix
    foreach ($prefix in @($sfmlPath, $enginePath)) {
        if (-not (Test-Path -LiteralPath $prefix -PathType Container)) {
            throw "Package prefix does not exist: $prefix"
        }
        if ($prefix.Contains(';')) { throw 'Package prefixes must not contain semicolons.' }
    }
    if ($CxxCompiler) {
        $CxxCompiler = (Get-Command $CxxCompiler -CommandType Application -ErrorAction Stop).Source
        # CMake's Windows default may select Visual Studio and ignore a supplied
        # MinGW/Clang compiler. Ninja explicitly uses the selected compiler.
        if (-not $Generator) { $Generator = 'Ninja' }
        if ($Generator -like 'Visual Studio*') {
            throw 'Visual Studio selects its own compiler. Omit -CxxCompiler or use -Generator Ninja.'
        }
    }
    if ($MakeProgram) {
        $MakeProgram = (Get-Command $MakeProgram -CommandType Application -ErrorAction Stop).Source
        if (-not $Generator) {
            throw 'Specify -Generator when supplying -MakeProgram (for example, Ninja).'
        }
        if ($Generator -like 'Visual Studio*' -or $Generator -eq 'Xcode') {
            throw '-MakeProgram requires a makefile or Ninja generator.'
        }
    }
    $buildPath = Get-ScriptPath $BuildDirectory
    $configureArguments = @('-S', $PSScriptRoot, '-B', $buildPath,
        "-DCMAKE_BUILD_TYPE=$Configuration", "-DCMAKE_PREFIX_PATH=$enginePath;$sfmlPath")
    if ($Generator) { $configureArguments += @('-G', $Generator) }
    if ($CxxCompiler) { $configureArguments += "-DCMAKE_CXX_COMPILER=$CxxCompiler" }
    if ($MakeProgram) { $configureArguments += "-DCMAKE_MAKE_PROGRAM=$MakeProgram" }

    Invoke-CMake $configureArguments
    Invoke-CMake @('--build', $buildPath, '--config', $Configuration, '--parallel', "$Parallel")
    Write-Host "Visualizer built in $buildPath ($Configuration)."
    exit 0
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
}
