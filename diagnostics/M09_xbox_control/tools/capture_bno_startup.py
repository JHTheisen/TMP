"""Capture stock M08/M09 startup serial output. Opening/resetting can start motors.

No calibration, pose or jog commands are sent. X is sent on exit to stop/latch
the tested firmware. Use identical physical conditions for each comparison run.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--label", required=True, help="e.g. M08_A1 or M09_B1")
    parser.add_argument("--seconds", type=float, default=35)
    parser.add_argument("--reset", choices=("none", "rts"), default="none")
    parser.add_argument("--open-lines", choices=("inactive", "host-default"), default="inactive")
    parser.add_argument("--status-poll", action="store_true", help="M09 only: send STATUS every 500 ms, as the Xbox host does")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parents[1] / "audit")
    args = parser.parse_args()
    if not 1 <= args.seconds <= 180 or not args.label.replace("_", "").replace("-", "").isalnum():
        parser.error("seconds must be 1..180; label must contain only letters, digits, underscores or hyphens")
    import serial
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    base = args.output_dir / f"{args.label}_{stamp}"
    summary = {"label": args.label, "started_utc": stamp, "port": args.port,
               "reset": args.reset, "open_lines": args.open_lines, "seconds": args.seconds,
               "hardware_capture": True, "boot_banners": [], "first_accuracy_seconds": {},
               "state_samples": 0, "readiness": [], "last_state": None,
               "status_poll": args.status_poll, "status_replies": [],
               "limitation": "Stock firmware stops STATE telemetry after readiness/abort; absent later samples do not prove accuracy stayed low."}
    port = serial.Serial(port=None, baudrate=115200, timeout=0.05, write_timeout=0.2)
    port.port = args.port
    # Match these on both images. Some USB drivers may still glitch lines on open.
    port.dtr = port.rts = args.open_lines == "host-default"
    opened = False
    started = time.monotonic()
    pending = ""
    try:
        with base.with_suffix(".log").open("x", encoding="utf-8") as log:
            port.open()
            opened = True
            started = time.monotonic()
            log.write(f"# {json.dumps(summary)}\n")
            if args.reset == "rts":
                port.dtr = False
                port.rts = True
                time.sleep(0.1)
                port.rts = False
                log.write(f"{time.monotonic() - started:.3f} # intentional RTS reset released\n")
            next_status = 0.0
            while time.monotonic() - started < args.seconds:
                elapsed = time.monotonic() - started
                if args.status_poll and elapsed >= next_status:
                    if port.write(b"STATUS\n") != 7:
                        raise RuntimeError("Incomplete STATUS write")
                    log.write(f"{elapsed:.3f} # TX STATUS\n")
                    next_status = elapsed + 0.5
                pending += port.read(min(max(port.in_waiting, 1), 4096)).decode("utf-8", errors="replace")
                while "\n" in pending:
                    line, pending = pending.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    elapsed = round(time.monotonic() - started, 3)
                    log.write(f"{elapsed:.3f} {line}\n")
                    if line.startswith(("M08_go_to_pose:", "M09_xbox_control:")):
                        summary["boot_banners"].append({"at": elapsed, "line": line})
                    if line.startswith("STATE "):
                        fields = dict(token.split("=", 1) for token in line.split()[2:] if "=" in token)
                        summary["state_samples"] += 1
                        summary["last_state"] = {"at": elapsed, "phase": line.split()[1], **fields}
                        if "accuracy" in fields:
                            summary["first_accuracy_seconds"].setdefault(fields["accuracy"], elapsed)
                    if line.startswith("M09 "):
                        summary["status_replies"].append({"at": elapsed, "line": line})
                    if "READY:" in line or line.startswith(("Reason:", "FINAL RESULT:", "NORTH REFERENCE")):
                        summary["readiness"].append({"at": elapsed, "line": line})
                if len(pending) > 16384:
                    raise RuntimeError("Unterminated serial output exceeded capture limit")
                log.flush()
            if pending:
                log.write(f"{time.monotonic() - started:.3f} # partial: {pending}\n")
    except BaseException as error:
        summary["error"] = str(error)
        raise
    finally:
        if opened:
            try:
                summary["exit_abort_sent"] = port.write(b"X\n") == 2
                port.flush()
                # Confirm the final state before closing; reopening may reset this board.
                exit_deadline = time.monotonic() + 1.0
                exit_data = ""
                while time.monotonic() < exit_deadline:
                    exit_data += port.read(min(max(port.in_waiting, 1), 4096)).decode("utf-8", errors="replace")
                summary["exit_response"] = exit_data.strip()
                with base.with_suffix(".log").open("a", encoding="utf-8") as log:
                    log.write(f"{time.monotonic() - started:.3f} # TX X; exit response follows\n{exit_data}\n")
            except Exception as error:
                summary["exit_abort_error"] = str(error)
            port.close()
        base.with_suffix(".json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary, indent=2))
        print(f"Capture: {base.with_suffix('.log')}")


if __name__ == "__main__":
    main()
