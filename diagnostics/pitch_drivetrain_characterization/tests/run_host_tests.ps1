param([string]$Compiler = 'C:\Strawberry\c\bin\g++.exe')
$ErrorActionPreference = 'Stop'
$projectPath = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $projectPath
try {
    New-Item -ItemType Directory -Force -Path '.pio\host_tests' | Out-Null
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic tests/characterization_math_test.cpp -o .pio/host_tests/math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Math test compilation failed' }
    & '.pio\host_tests\math.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Math tests failed' }
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -I tests/stubs -I ../../firmware/include tests/characterization_integration_test.cpp -o .pio/host_tests/integration.exe
    if ($LASTEXITCODE -ne 0) { throw 'Integration test compilation failed' }
    foreach ($scenario in @('normal', 'opposite', 'ratio10', 'transient', 'reset', 'persistent', 'encoder', 'blocked_encoder', 'magnet', 'noisy', 'init', 'abort')) {
        & '.pio\host_tests\integration.exe' $scenario
        if ($LASTEXITCODE -ne 0) { throw "Integration scenario failed: $scenario" }
    }
} finally {
    Pop-Location
}
