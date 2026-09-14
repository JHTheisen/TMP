param([string]$Compiler = 'C:\Strawberry\c\bin\g++.exe')
$ErrorActionPreference = 'Stop'
$projectPath = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $projectPath
try {
    New-Item -ItemType Directory -Force -Path '.pio\host_tests' | Out-Null
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -I tests/stubs tests/control_math_test.cpp -o .pio/host_tests/math.exe
    if ($LASTEXITCODE -ne 0) { throw 'M07 math test compilation failed' }
    & '.pio\host_tests\math.exe'
    if ($LASTEXITCODE -ne 0) { throw 'M07 math tests failed' }
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -Wno-unused-variable -DM07_HOST_TEST -I tests/stubs tests/watchdog_test.cpp -o .pio/host_tests/watchdog.exe
    if ($LASTEXITCODE -ne 0) { throw 'M07 watchdog test compilation failed' }
    & '.pio\host_tests\watchdog.exe'
    if ($LASTEXITCODE -ne 0) { throw 'M07 watchdog tests failed' }
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -DM07_HOST_TEST -I tests/stubs -I ../../firmware/include tests/north_level_integration_test.cpp -o .pio/host_tests/integration.exe
    if ($LASTEXITCODE -ne 0) { throw 'M07 integration test compilation failed' }
    $scenarios = @('normal', 'wrap_positive', 'wrap_negative', 'accuracy_brief', 'accuracy_repeated',
        'accuracy_sustained', 'baseline_low', 'baseline_intermittent', 'brief_gap', 'stale_gap', 'blocked_bno',
        'fresh_after_stop', 'reset', 'reset_in_poll', 'startup_reset', 'invalid_vector', 'wrong_report',
        'abort', 'startup_abort', 'wrong_direction', 'runaway', 'yaw_guard_positive', 'yaw_guard_negative',
        'pitch_guard_positive', 'pitch_guard_negative', 'no_probe_progress', 'no_slew_progress',
        'overall_timeout', 'braking_timeout', 'burst_timeout', 'move_rejected', 'yaw_slew_rejected',
        'pitch_slew_rejected', 'blocked_uart', 'frozen_feedback', 'settle_running', 'low_gain', 'first_test',
        'axis_init', 'report_init')
    foreach ($scenario in $scenarios) {
        & '.pio\host_tests\integration.exe' $scenario
        if ($LASTEXITCODE -ne 0) { throw "M07 integration scenario failed: $scenario" }
    }
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -I tests/stubs tests/pose_math_test.cpp -o .pio/host_tests/pose_math.exe
    if ($LASTEXITCODE -ne 0) { throw 'M08 pose math test compilation failed' }
    & '.pio\host_tests\pose_math.exe'
    if ($LASTEXITCODE -ne 0) { throw 'M08 pose math tests failed' }
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -DM07_HOST_TEST -I tests/stubs -I ../../firmware/include tests/m08_pose_integration_test.cpp -o .pio/host_tests/pose_integration.exe
    if ($LASTEXITCODE -ne 0) { throw 'M08 pose integration test compilation failed' }
    $poseScenarios = @('coordinated', 'lower_gain', 'no_op', 'carriage_only', 'yaw_only', 'pitch_only', 'relative', 'small_relative',
        'repeated_absolute', 'invalid', 'relative_bounds', 'busy', 'startup_busy', 'fragmented', 'idle_feedback',
        'idle_stale', 'idle_reset', 'idle_motion', 'missing_timing', 'blocked_output', 'abort', 'stale', 'blocked_bno', 'reset',
        'carriage_rejected', 'carriage_timeout', 'pitch_low_entry', 'pitch_accuracy_drop',
        'carriage_accuracy_brief', 'carriage_accuracy_sustained')
    foreach ($scenario in $poseScenarios) {
        & '.pio\host_tests\pose_integration.exe' $scenario
        if ($LASTEXITCODE -ne 0) { throw "M08 pose integration scenario failed: $scenario" }
    }
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -DM07_HOST_TEST -I tests/stubs -I ../../firmware/include tests/pitch_readiness_test.cpp -o .pio/host_tests/pitch_readiness.exe
    if ($LASTEXITCODE -ne 0) { throw 'M08 pitch readiness test compilation failed' }
    $pitchScenarios = @('accuracy_zero', 'accuracy_one', 'unstable_heading', 'invalid_baseline', 'stale_baseline',
        'unstable_pitch', 'calibration_recovers', 'late_accuracy', 'baseline_criteria', 'idle_stale', 'idle_reset',
        'stale', 'blocked_bno', 'invalid_motion', 'wrong_report', 'reset', 'reset_in_poll', 'abort',
        'wrong_direction', 'no_progress', 'pitch_guard', 'yaw_guard', 'timeout', 'blocked_uart', 'roll_mapping')
    foreach ($scenario in $pitchScenarios) {
        & '.pio\host_tests\pitch_readiness.exe' $scenario
        if ($LASTEXITCODE -ne 0) { throw "M08 pitch readiness scenario failed: $scenario" }
    }
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -DM07_HOST_TEST -I tests/stubs -I ../../firmware/include tests/manual_integration_test.cpp -o .pio/host_tests/manual.exe
    if ($LASTEXITCODE -ne 0) { throw 'M09 manual test compilation failed' }
    $manualScenarios = @('startup_busy', 'admission', 'pitch_only_ready', 'center', 'axes', 'reverse', 'pose_after',
        'rate_update', 'busy', 'blocked_uart', 'host_loss', 'partial', 'malformed_lease', 'blocked_host', 'stale', 'invalid',
        'wrong_report', 'blocked_bno', 'accuracy', 'reset', 'abort', 'wrong_direction', 'no_progress', 'yaw_guard',
        'pitch_guard', 'braking_timeout', 'timeout', 'speed_rejected', 'late_packet',
        'low_axes', 'low_reverse', 'low_stale', 'low_invalid', 'low_wrong_report', 'low_reset',
        'low_host_loss', 'low_abort', 'low_yaw_guard', 'low_pitch_guard', 'low_wrong_direction', 'low_no_progress')
    foreach ($scenario in $manualScenarios) {
        & '.pio\host_tests\manual.exe' $scenario
        if ($LASTEXITCODE -ne 0) { throw "M09 manual scenario failed: $scenario" }
    }
    & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -DM07_HOST_TEST -I tests/stubs -I ../../firmware/include tests/manual_carriage_test.cpp -o .pio/host_tests/manual_carriage.exe
    if ($LASTEXITCODE -ne 0) { throw 'M09 carriage test compilation failed' }
    $carriageScenarios = @('protocol', 'axes', 'reverse', 'rate', 'pose_after', 'limits', 'near_limit', 'boundary_stop',
        'host_loss', 'malformed', 'stale', 'invalid', 'wrong_report', 'blocked_bno', 'reset', 'abort',
        'braking_timeout', 'stop_stale', 'limit_guard', 'timeout', 'speed_rejected')
    foreach ($scenario in $carriageScenarios) {
        & '.pio\host_tests\manual_carriage.exe' $scenario
        if ($LASTEXITCODE -ne 0) { throw "M09 carriage scenario failed: $scenario" }
    }
} finally { Pop-Location }
