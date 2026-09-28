param([string]$Compiler = 'C:\Strawberry\c\bin\g++.exe',
      [string]$Python = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe")
$ErrorActionPreference = 'Stop'
$projectPath = Split-Path -Parent $PSScriptRoot
$previousPythonPath = $env:PYTHONPATH
Push-Location -LiteralPath $projectPath
try {
    if (Test-Path '.pio/python_deps') { $env:PYTHONPATH = "$projectPath\.pio\python_deps;$previousPythonPath" }
    New-Item -ItemType Directory -Force -Path '.pio\host_tests' | Out-Null
    function Build-Test([string]$Source, [string]$Name) {
        & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -Wno-unused-variable -DM07_HOST_TEST -I tests/stubs -I ../../firmware/include "tests/$Source" -o ".pio/host_tests/$Name.exe"
        if ($LASTEXITCODE -ne 0) { throw "Test compilation failed: $Source" }
    }
    function Run-Cases([string]$Name, [string[]]$Cases) {
        foreach ($scenario in $Cases) {
            & ".pio/host_tests/$Name.exe" $scenario
            if ($LASTEXITCODE -ne 0) { throw "Test failed: $Name $scenario" }
        }
    }
    # Separately test encoder restoration before the integrated motion suites.
    foreach ($unit in @('encoder_acquisition','control_math','pose_math','watchdog','bno_observer','bno_diagnostics','bno_trace','sensor_handoff','sensor_result','pitch_direction_diagnostics')) {
        Build-Test "$($unit)_test.cpp" $unit
        & ".pio/host_tests/$unit.exe"
        if ($LASTEXITCODE -ne 0) { throw "Unit test failed: $unit" }
    }
    Build-Test 'sensor_worker_stall_test.cpp' 'sensor_worker_stall'
    Run-Cases 'sensor_worker_stall' @('jog','host_loss','stop','center','encoder','buffered_noise')
    Build-Test 'm09_lifecycle_test.cpp' 'lifecycle'
    Run-Cases 'lifecycle' @('startup_valid','startup_missing_bno','startup_init_failure','startup_report_failure',
        'startup_bus_a_failure','startup_bus_b_failure','startup_low_accuracy','startup_invalid','startup_stale',
        'encoders','encoder_failure','idle_recovery','reset_recovery','north_qualification','manual_without_sensors',
        'angular_requires_bno','carriage_without_bno','carriage_overflow','missing_timing','pitch_mapping')
    Build-Test 'manual_integration_test.cpp' 'manual'
    Run-Cases 'manual' @('startup','admission','axes','rates','reverse','stop_without_bno','missing_bno','low_accuracy',
        'stale','invalid','wrong_report','reset','no_orientation_limits','frozen_feedback','host_loss','malformed_lease',
        'late_packet','blocked_main','slow_encoders','braking_timeout','axis_rejected','unexpected_stop','long_manual',
        'abort','axis_init','speed_init')
    Build-Test 'manual_carriage_test.cpp' 'manual_carriage'
    Run-Cases 'manual_carriage' @('protocol','axes','reverse','rates','unbounded','sensor_independent','host_loss','braking_timeout','axis_rejected')
    Build-Test 'm08_pose_integration_test.cpp' 'pose_integration'
    Run-Cases 'pose_integration' @('coordinated','relative','pitch_low','pitch_carriage_low','no_op','invalid','stale',
        'blocked_bno','invalid_feedback','wrong_report','accuracy','reset','runaway','pitch_guard','no_progress','timeout','abort')
    Build-Test 'pose_restoration_test.cpp' 'pose_restoration'
    Run-Cases 'pose_restoration' @('first_pose','first_carriage','pitch_low','unqualified','fallback_bounds','admission','yaw_hold','yaw_recovered',
        'velocity_receipt','observe_receipt','settle_receipt','sample_order','receipt_rollover','baseline_receipt')
    Build-Test 'pose_stop_test.cpp' 'pose_stop'
    Run-Cases 'pose_stop' @('angular','carriage','pending','repeat','timeout','stall','abort')
    Build-Test 'keyframe_math_test.cpp' 'keyframe_math'
    & '.pio/host_tests/keyframe_math.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Keyframe timing math failed' }
    Build-Test 'keyframe_motion_test.cpp' 'keyframe_motion'
    Run-Cases 'keyframe_motion' @('snapshot','reject','admission','travel','missing_bno','stall_continue','configuration','partial_start',
        'complete','zero_axis','no_op','stop','brake_timeout','abort','stale','reset','pitch_guard','stall_stop','unexpected_stop')
    # Keep old startup/fallback regression evidence against M08, which is unchanged.
    Build-Test 'north_level_integration_test.cpp' 'm08_startup'
    $scenarios = @('normal', 'wrap_positive', 'wrap_negative', 'accuracy_brief', 'accuracy_repeated',
        'accuracy_sustained', 'baseline_low', 'baseline_intermittent', 'brief_gap', 'stale_gap', 'blocked_bno',
        'fresh_after_stop', 'reset', 'reset_in_poll', 'startup_reset', 'invalid_vector', 'wrong_report',
        'abort', 'startup_abort', 'wrong_direction', 'runaway', 'yaw_guard_positive', 'yaw_guard_negative',
        'pitch_guard_positive', 'pitch_guard_negative', 'no_probe_progress', 'no_slew_progress',
        'overall_timeout', 'braking_timeout', 'burst_timeout', 'move_rejected', 'yaw_slew_rejected',
        'pitch_slew_rejected', 'blocked_uart', 'frozen_feedback', 'settle_running', 'low_gain', 'first_test',
        'axis_init', 'report_init')
    Run-Cases 'm08_startup' $scenarios
    Build-Test 'pitch_readiness_test.cpp' 'm08_pitch_readiness'
    $pitchScenarios = @('accuracy_zero', 'accuracy_one', 'unstable_heading', 'invalid_baseline', 'stale_baseline',
        'unstable_pitch', 'calibration_recovers', 'late_accuracy', 'baseline_criteria', 'idle_stale', 'idle_reset',
        'stale', 'blocked_bno', 'invalid_motion', 'wrong_report', 'reset', 'reset_in_poll', 'abort',
        'wrong_direction', 'no_progress', 'pitch_guard', 'yaw_guard', 'timeout', 'blocked_uart', 'roll_mapping')
    Run-Cases 'm08_pitch_readiness' $pitchScenarios
    # unittest's normal progress uses stderr; Windows PowerShell must not turn
    # passing test output into a terminating NativeCommandError under redirection.
    $ErrorActionPreference = 'Continue'
    & $Python -B -m unittest discover -s tests -p 'test_*.py' -v
    $pythonExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($pythonExit -ne 0) { throw 'Python tests failed' }
    & (Join-Path $PSScriptRoot 'verify_m08_baseline.ps1')
} finally { $env:PYTHONPATH = $previousPythonPath; Pop-Location }
