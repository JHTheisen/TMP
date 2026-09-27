"""Flushed, per-run diagnostic text logs. No serial or motor dependencies."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time
import traceback


class SessionLog:
    def __init__(self, directory, settings):
        self.path = None
        self.error = ""
        self._file = None
        self._samples = {}
        self._state = None
        self._start = time.monotonic()
        try:
            directory = Path(directory).expanduser().resolve()
            directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            # Exclusive creation never overwrites an earlier run, even on collision.
            for suffix in range(1000):
                path = directory / f"m09_{stamp}_{os.getpid()}_{suffix}.log"
                try:
                    self._file = path.open("x", encoding="utf-8", buffering=1)
                    self.path = path
                    break
                except FileExistsError:
                    continue
            if self._file is None:
                raise OSError("Could not allocate a unique session log")
            self.event("SESSION", "start " + json.dumps(settings, sort_keys=True))
        except OSError as error:
            self._failed(error)

    def _failed(self, error):
        self.error = str(error)
        print(f"Diagnostic logging unavailable: {error}", file=sys.stderr)
        if self._file is not None:
            try:
                self._file.close()
            except OSError:
                pass
            self._file = None

    def event(self, kind, message):
        if self._file is None:
            return
        stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        elapsed = time.monotonic() - self._start
        try:
            self._file.write(f"{stamp} +{elapsed:.3f}s {kind} {message}\n")
            self._file.flush()  # Survives Python exceptions/termination; not a power-loss fsync guarantee.
        except OSError as error:
            self._failed(error)  # Disk trouble does not add a motor interlock.

    def sampled(self, key, kind, message, now, period, signature=None):
        previous = self._samples.get(key)
        if previous is None or previous[1] != signature or now - previous[0] >= period:
            self.event(kind, message)
            self._samples[key] = (now, signature)

    def received(self, line, now):
        fields = dict(token.split("=", 1) for token in line.split() if "=" in token)
        prefix = line.split(" ", 1)[0]
        if prefix in ("BNO_STATE", "BNO_RAW", "BNO_REPORT", "ENCODER_STATE", "MANUAL_STATE", "POSE_STATE", "STATE"):
            signature = tuple(fields.get(key) for key in (
                "available", "valid", "has_sample", "fresh", "accuracy", "raw_status", "accepted",
                "reason", "euler_valid", "diagnostic_quality", "report_id", "north_usable", "status", "magnet_good", "active"))
            self.sampled((prefix, fields.get("bus")), "RX", line + " | parsed=" + json.dumps(fields, sort_keys=True),
                         now, 0.5, signature)
        elif line in ("M09 READY", "M09 MANUAL", "M09 BUSY", "M09 ABORTED"):
            self.sampled("firmware_mode", "RX", line, now, 5.0, line)
        else:
            self.event("RX", line)

    def transmitted(self, data, now):
        command = data.decode("ascii", errors="replace").strip()
        if command.startswith("JOG "):
            fields = command.split()[1:]
            signs = tuple(0 if int(value) == 0 else (1 if int(value) > 0 else -1) for value in fields)
            # Starts, stops and reversals are immediate; changing magnitude/held
            # velocity is sampled at 2 Hz instead of logging the 50 Hz stream.
            self.sampled("jog", "TX_ATTEMPT", command, now, 0.5, signs)
        elif command == "STATUS":
            self.sampled("status", "TX_ATTEMPT", command, now, 5.0)
        else:
            self.event("TX_ATTEMPT", command)

    def state(self, session):
        value = (session.state, session.ready, session.pending_stop, session.readiness_note)
        if value != self._state:
            self.event("HOST_STATE", f"state={value[0]} ready={value[1]} pending_stop={value[2]} reason={value[3]}")
            self._state = value

    def exception(self, context):
        self.event("EXCEPTION", context + "\n" + traceback.format_exc().rstrip())

    def close(self, result):
        self.event("SESSION", f"exit result={result}")
        if self._file is not None:
            try:
                self._file.close()
            except OSError as error:
                self._failed(error)
            self._file = None
