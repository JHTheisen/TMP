# Milestone 4 Pitch Drivetrain Characterization

This test characterizes the pitch drivetrain using the motor-shaft AS5600 and
the cradle BNO085. The AS5600 must communicate, return valid raw-angle samples,
remain fresh, and maintain an unambiguous continuous unwrap. Read failures,
unsafe acquisition gaps, and ambiguous half-turns remain hard failures.

The AS5600 magnet-status flags are advisory for this characterization. The
firmware still reads and reports `DETECTED`, `TOO-WEAK`, `TOO-STRONG`, or
`NOT-DETECTED`, and counts non-ideal samples in the final summary. A non-ideal
status alone does not fail the test when the raw-angle data remain valid and
the characterization completes. Angle-data integrity and the existing BNO,
motion, freshness, and safety checks remain mandatory.

Run the native checks with:

```powershell
.\tests\run_host_tests.ps1
```