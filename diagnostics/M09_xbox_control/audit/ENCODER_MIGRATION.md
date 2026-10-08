# Migration record — 2026-10-08

Started on main at `294fc4ea9b91b9450d8be19e6b321398ad932a21`, one commit ahead
of origin/main (`7a79876`). There were no tracked uncommitted changes. The two
separate worktrees appeared as untracked directories at the parent repository;
neither was modified. No reset, implementation commit or push was performed.

Before editing, `git bundle create encoder-migration-checkpoint.bundle --all`
saved all refs and complete history; `git bundle verify` passed. The ignored
bundle lives at the project root. To recover into a separate directory, use
`git clone <absolute-path-to-bundle> <new-directory>` and inspect main at the
recorded commit. Existing repository branches and history remain intact.

The starting README hardware table identified pitch AS5600 as before reduction.
Neither a verified yaw geometry nor a complete signed pair of encoder scales was
established. Geometry therefore remains explicitly unconfigured at boot. Existing
52.0/67.2 pulse-per-degree tracking values were retained only for motor-rate
planning; they are not used to derive encoder orientation.

Superseded acquisition/calibration audits, transport patches, library stubs and
historical orientation suites were removed from the active project. They remain
recoverable in Git and the checkpoint. Generic motion math, keyframe, carriage,
watchdog, thread handoff and actual installed low-rate ramp tests were retained.
Encoder-specific integration tests replace the retired orientation fixtures.

The starting host also had a truncated conditional in ManualSession.receive's
M09 BUSY branch. Its syntax was repaired so the current host and tests can run.
Hardware was not driven and firmware was not uploaded for this migration.
