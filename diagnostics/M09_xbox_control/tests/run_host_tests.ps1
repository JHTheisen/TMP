param([string]$Compiler = 'C:\Strawberry\c\bin\g++.exe',
      [string]$Python = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe",
      [switch]$SkipPython)
$ErrorActionPreference = 'Stop'
$projectPath = Split-Path -Parent $PSScriptRoot
$previousPythonPath = $env:PYTHONPATH
Push-Location -LiteralPath $projectPath
try {
    if (Test-Path '.pio/python_deps') { $env:PYTHONPATH = "$projectPath\.pio\python_deps;$previousPythonPath" }
    New-Item -ItemType Directory -Force -Path '.pio\host_tests' | Out-Null
    function Build-Test([string]$Name) {
        & $Compiler -std=c++11 -Wall -Wextra -Werror -pedantic -Wno-unused-variable -DM07_HOST_TEST -I tests/stubs -I ../../firmware/include "tests/$($Name)_test.cpp" -o ".pio/host_tests/$Name.exe"
        if ($LASTEXITCODE -ne 0) { throw "Test compilation failed: $Name" }
    }
    function Run-Cases([string]$Name, [string[]]$Cases) {
        Build-Test $Name
        foreach ($scenario in $Cases) {
            & ".pio/host_tests/$Name.exe" $scenario
            if ($LASTEXITCODE -ne 0) { throw "Test failed: $Name $scenario" }
        }
    }
    foreach ($unit in @('encoder_acquisition','encoder_position','control_math','pose_math','watchdog','sensor_handoff','keyframe_math','pitch_direction_diagnostics')) {
        Build-Test $unit
        & ".pio/host_tests/$unit.exe"
        if ($LASTEXITCODE -ne 0) { throw "Unit test failed: $unit" }
    }
    Run-Cases 'encoder_motion' @('bus_recovery','north','level','pose','alignment_stop','admission','references','protocol','no_step_substitution','wrap','goto','tracking','stop','takeover','disconnect','bad_magnet','worker_stale','blocked_foreground','lease','abort','stall','wrong_direction')
    Run-Cases 'encoder_direction' @('geometry','log_goto','yaw_positive_slew','pitch_positive_slew','yaw_negative_slew','pitch_negative_slew','yaw_positive_precision','yaw_negative_precision','pitch_positive_precision','pitch_negative_precision','yaw_positive_inverted','pitch_negative_inverted','yaw_positive_track','yaw_negative_track','pitch_positive_track','pitch_negative_track','wrong_wiring')
    Run-Cases 'manual_integration' @('startup','admission','axes','rates','reverse','stop_without_encoders','missing_encoders','bad_magnet','no_orientation_limits','frozen_feedback','host_loss','malformed_lease','late_packet','blocked_main','slow_encoders','braking_timeout','axis_rejected','unexpected_stop','first_jog','idle_force_stop_recovery','long_manual','abort','axis_init','speed_init')
    Run-Cases 'pose_stop' @('angular','carriage','pending','repeat','timeout','stall','abort')
    Run-Cases 'manual_carriage' @('protocol','axes','reverse','rates','unbounded','sensor_independent','host_loss','braking_timeout','axis_rejected','delayed_start')
    Run-Cases 'keyframe_motion' @('snapshot','reject','admission','travel','stepped','missing_encoders','stall_continue','configuration','partial_start','complete','zero_axis','no_op','stop','brake_timeout','abort','stale','reset','pitch_guard','stall_stop','unexpected_stop')
    Run-Cases 'celestial_low_rate' @('stable','minimum_rate','zero','below_minimum','reverse','pending_period','brake_stuck','startup_latch','startup_fails','startup_timeout','unexpected_stop','forced_restart','takeover','lease','sensor_pause')
    Run-Cases 'celestial_zero_rate' @('exact_zero','near_zero','long_zero','same_direction','positive_zero_negative','negative_zero_positive','unexpected_stop','failed_reversal','stop','takeover','lease')
    & $Compiler -std=c++11 -DTEST -I .pio/libdeps/esp32dev/FastAccelStepper/src tests/fas_low_rate_test.cpp -o .pio/host_tests/fas_low_rate.exe
    if ($LASTEXITCODE -ne 0) { throw 'FastAccelStepper test compilation failed' }
    & '.pio/host_tests/fas_low_rate.exe'
    if ($LASTEXITCODE -ne 0) { throw 'FastAccelStepper ramp test failed' }
    if (-not $SkipPython) {
        $ErrorActionPreference = 'Continue'
        & $Python -B -m unittest discover -s tests -p 'test_*.py' -v
        $pythonExit = $LASTEXITCODE
        $ErrorActionPreference = 'Stop'
        if ($pythonExit -ne 0) { throw 'Python tests failed' }
    }
} finally { $env:PYTHONPATH = $previousPythonPath; Pop-Location }
