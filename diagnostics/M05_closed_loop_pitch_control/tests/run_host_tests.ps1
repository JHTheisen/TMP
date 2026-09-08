param([string]$Compiler = 'C:\Strawberry\c\bin\g++.exe')
$ErrorActionPreference = 'Stop'
$projectPath = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $projectPath
try {
    New-Item -ItemType Directory -Force -Path '.pio\host_tests' | Out-Null
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -I tests/stubs tests/control_math_test.cpp -o .pio/host_tests/math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Math test compilation failed' }
    & '.pio\host_tests\math.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Math tests failed' }
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -I tests/stubs -I ../../firmware/include tests/pitch_control_integration_test.cpp -o .pio/host_tests/integration.exe
    if ($LASTEXITCODE -ne 0) { throw 'Integration test compilation failed' }
    foreach ($scenario in @(
        'normal', 'encoder_opposite', 'ratio7', 'ratio10', 'ratio30', 'pulses3200',
        'noise', 'allowed_noise', 'slip_backlash', 'disturbance', 'drift', 'transient', 'magnet', 'startup_reset',
        'wrong_way', 'frozen_cradle', 'frozen_motor', 'frozen_encoder',
        'bno_outage', 'encoder_outage', 'encoder_error', 'encoder_ambiguous', 'blocked_encoder', 'blocked_bno',
        'reset', 'invalid_quaternion', 'wrong_report', 'relative_guard', 'absolute_guard',
        'abort', 'motion_timeout', 'noisy_motion', 'move_rejected', 'axis_init', 'bus_init', 'encoder_init',
        'bno_init', 'report_init', 'unstable_baseline', 'startup_tilt', 'startup_abort'
    )) {
        & '.pio\host_tests\integration.exe' $scenario
        if ($LASTEXITCODE -ne 0) { throw "Integration scenario failed: $scenario" }
    }
} finally {
    Pop-Location
}
