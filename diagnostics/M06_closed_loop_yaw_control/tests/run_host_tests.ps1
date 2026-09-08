param(
    [string]$Compiler = 'C:\Strawberry\c\bin\g++.exe',
    [string]$HardwareInclude = ''
)
$ErrorActionPreference = 'Stop'
$projectPath = Split-Path -Parent $PSScriptRoot
if (!$HardwareInclude) { $HardwareInclude = Join-Path $projectPath '../../firmware/include' }
$HardwareInclude = (Resolve-Path -LiteralPath $HardwareInclude).Path
Push-Location -LiteralPath $projectPath
try {
    New-Item -ItemType Directory -Force -Path '.pio\host_tests' | Out-Null
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -I tests/stubs tests/control_math_test.cpp -o .pio/host_tests/math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Math test compilation failed' }
    & '.pio\host_tests\math.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Math tests failed' }
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -I tests/stubs -I $HardwareInclude tests/yaw_control_integration_test.cpp -o .pio/host_tests/integration.exe
    if ($LASTEXITCODE -ne 0) { throw 'Integration test compilation failed' }
    foreach ($scenario in @(
        'normal', 'wrap_forward', 'wrap_zero_noise', 'wrap_noise', 'high_gain', 'low_gain',
        'noise', 'allowed_noise', 'slip_backlash', 'disturbance', 'drift', 'transient',
        'startup_reset', 'startup_reset_retry', 'wrong_way', 'wrong_way_noise', 'return_wrong_way',
        'runaway', 'frozen_motor', 'frozen_feedback', 'bno_outage', 'pre_motion_outage',
        'blocked_bno', 'reset', 'reset_in_poll', 'invalid_quaternion', 'wrong_report',
        'guard_positive', 'guard_negative', 'guard_wrap', 'heading_jump', 'tilt_motion',
        'abort', 'motion_timeout', 'noisy_motion', 'move_rejected',
        'axis_init', 'bus_init', 'bno_init', 'bno_ack', 'report_init', 'speed_init', 'accel_init',
        'unstable_baseline', 'startup_tilt', 'startup_abort'
    )) {
        & '.pio\host_tests\integration.exe' $scenario
        if ($LASTEXITCODE -ne 0) { throw "Integration scenario failed: $scenario" }
    }
    # Separate build simulates a deliberate operator-selected negative polarity.
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -DM06_TRIAL_POSITIVE_STEP_YAW_SIGN=-1 -I tests/stubs -I $HardwareInclude tests/yaw_control_integration_test.cpp -o .pio/host_tests/negative_sign.exe
    if ($LASTEXITCODE -ne 0) { throw 'Negative-polarity test compilation failed' }
    foreach ($scenario in @('normal', 'wrap_forward', 'wrong_way', 'return_wrong_way')) {
        & '.pio\host_tests\negative_sign.exe' $scenario
        if ($LASTEXITCODE -ne 0) { throw "Negative-polarity scenario failed: $scenario" }
    }
} finally {
    Pop-Location
}
