param([string]$Compiler = 'C:\Strawberry\c\bin\g++.exe')
$ErrorActionPreference = 'Stop'
Push-Location -LiteralPath (Split-Path -Parent $PSScriptRoot)
try {
    New-Item -ItemType Directory -Force -Path '.pio\host_tests' | Out-Null
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -DM07_HOST_TEST -I tests/stubs -I ../../firmware/include tests/direction_test.cpp -o .pio/host_tests/direction.exe
    if ($LASTEXITCODE -ne 0) { throw 'Direction fixture compilation failed' }
    foreach ($scenario in @('positive','negative','no_response','encoder_good','encoder_weak','encoder_short','abort','stale','blocked_bno','reset','excursion','timeout','blocked_uart')) {
        & '.pio\host_tests\direction.exe' $scenario
        if ($LASTEXITCODE -ne 0) { throw "Direction scenario failed: $scenario" }
    }
} finally { Pop-Location }
