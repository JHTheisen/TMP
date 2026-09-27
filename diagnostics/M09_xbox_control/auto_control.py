"""Single-request Xbox AUTO protocol; no serial, pygame, or hardware access."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Keyframe:
    epoch: int
    steps: tuple
    heading: str
    pitch: str


class AutoSession:
    """In-memory A/B captures and explicit, correlated autonomous requests."""
    def __init__(self):
        self.enabled = False
        self.phase = "OFF"
        self.note = "Y: enter AUTO; manual arming remains separate."
        self.increment = 1
        self.duration = 10
        self.frames = {}
        self.epoch = None
        self.request_id = 0
        self.pending_id = None
        self.purpose = None
        self.deadline = None
        self.last_rx = None
        self.continuation = None
        self.last_command = ""

    def invalidate(self, reason):
        self.frames.clear()
        self.epoch = None
        self.note = "Captures cleared: " + reason

    def cancel(self, now, reason="STOP sent; wait for READY."):
        self.phase = "STOPPING" if self.enabled else "OFF"
        self.pending_id = self.purpose = self.continuation = None
        self.deadline = now + 4.0 if self.enabled else None
        self.note = reason

    def enter(self, now):
        self.enabled = True
        self.increment = 1
        self.last_rx = now
        self.cancel(now)

    def leave(self, now):
        self.enabled = False
        self.cancel(now, "Manual disarmed; center sticks and arm deliberately.")

    def _new_request(self, purpose, phase, now):
        self.request_id = self.request_id % 4294967295 + 1
        self.pending_id = self.request_id
        self.purpose = purpose
        self.phase = phase
        self.deadline = now + 4.0
        return self.pending_id

    def _command(self, line):
        self.last_command = line
        return (line + "\n").encode("ascii")

    def request(self, action, now, centered):
        """Return one command or a refusal. Rejected requests are never queued."""
        if not self.enabled or self.phase != "READY" or not centered:
            self.note = "Not queued: wait for READY, centered sticks and released controls."
            return None
        if action in ("capture_a", "capture_b", "return_a", "play"):
            if action in ("return_a", "play") and "A" not in self.frames:
                self.note = "Capture A first."
                return None
            if action == "play" and "B" not in self.frames:
                self.note = "Capture B first."
                return None
            ident = self._new_request(action, "SNAPSHOT", now)
            self.note = "Waiting for a fresh stopped snapshot."
            return self._command(f"SNAP {ident}")
        moves = {"pitch_up": (0, self.increment), "pitch_down": (0, -self.increment),
                 "yaw_right": (self.increment, 0), "yaw_left": (-self.increment, 0)}
        if action not in moves:
            return None
        yaw, pitch = moves[action]
        self._new_request(action, "MOVE_PENDING", now)
        self.note = "MOVE requested; firmware checks sensor/reference validity."
        return self._command(f"MOVE {yaw} {pitch} 0")

    def _finish(self, note):
        self.phase = "FINISH_WAIT"
        self.pending_id = self.purpose = self.continuation = None
        self.note = note + " Waiting for READY."

    def receive(self, line, now):
        self.last_rx = now
        fields = dict(token.split("=", 1) for token in line.split() if "=" in token)
        if line.startswith(("M09_xbox_control:", "KEYFRAME_INVALIDATED ")) or line == "M09 ABORTED":
            self.invalidate(line)
            if self.enabled:
                self.cancel(now, self.note)
                if line == "M09 ABORTED":
                    self.phase = "FAULT"
                    self.deadline = None
                    self.note += "; firmware abort is latched; reset required."
            return
        if not self.enabled:
            return
        if line == "M09 READY":
            # A READY polling reply alone cannot acknowledge any request.
            if self.phase in ("STOPPING", "FINISH_WAIT"):
                self.phase = "READY"
                self.deadline = None
                self.note = self.note.replace(" Waiting for READY.", "")
            return
        matching = fields.get("id") == str(self.pending_id)
        if line.startswith("KEYFRAME_SNAPSHOT ") and matching and self.phase == "SNAPSHOT":
            try:
                epoch = int(fields["epoch"])
                steps = tuple(int(fields[name]) for name in ("yaw_steps", "pitch_steps", "carriage_steps"))
                if not 1 <= epoch <= 4294967295 or any(not -2147483648 <= value <= 2147483647 for value in steps):
                    raise ValueError("out of range")
            except (ValueError, KeyError):
                self.note = "Malformed snapshot ignored; waiting for correlated valid response."
                return
            if self.epoch is not None and epoch != self.epoch:
                self.invalidate("firmware position epoch changed; capture A and B again")
                self._finish(self.note)
                return
            self.epoch = epoch
            snapshot = Keyframe(epoch, steps, fields.get("heading", "?"), fields.get("physical_pitch", "?"))
            if self.purpose in ("capture_a", "capture_b"):
                slot = "A" if self.purpose == "capture_a" else "B"
                self.frames[slot] = snapshot
                self.phase = "READY"  # The correlated reply itself guarantees stopped READY.
                self.pending_id = self.purpose = self.deadline = None
                self.note = f"Captured {slot}: steps {steps}; startup-relative and unhomed."
                return
            if self.purpose == "play" and steps != self.frames["A"].steps:
                self._finish("Play refused: not at A. Press left-stick click to return to A first.")
                return
            target = self.frames["B" if self.purpose == "play" else "A"]
            if target.epoch != epoch:
                self.invalidate("keyframe epoch mismatch")
                self._finish(self.note)
                return
            ident = self._new_request(self.purpose, "KEYMOVE_PREPARED", now)
            self.continuation = self._command(f"KEYMOVE {ident} {epoch} {target.steps[0]} {target.steps[1]} {target.steps[2]} {self.duration * 1000}")
            self.note = "Stopped position verified; preparing one finite transition."
        elif matching and line.startswith(("SNAP REJECTED ", "KEYMOVE REJECTED ")):
            self._finish(line)
            self.deadline = now + 4.0
        elif matching and self.phase == "KEYMOVE_PENDING" and line.startswith("KEYMOVE ACCEPTED "):
            self.phase = "KEYMOVE_ACTIVE"
            self.deadline = now + self.duration + 5.0
            self.note = "Playing generated steps; measured-angle settling is not claimed."
        elif matching and self.phase in ("KEYMOVE_PENDING", "KEYMOVE_ACTIVE") and line.startswith("KEYMOVE RESULT "):
            if fields.get("status") == "FAILED":
                self.invalidate("keyframe motion failed")
            self._finish(line)
            self.deadline = now + 4.0
        elif self.phase == "MOVE_PENDING" and line.startswith(("POSE ACCEPTED", "PITCH-ONLY ACCEPTED")):
            self.phase = "MOVE_ACTIVE"
            self.deadline = now + 95.0  # Existing firmware POSE timeout is 90 s.
            self.note = "Angular MOVE active; further movement presses are not queued."
        elif self.phase == "MOVE_PENDING" and line.startswith(("POSE REJECTED:", "POSE ALREADY AT TARGET:")):
            self._finish(line)
            self.deadline = now + 4.0
        elif self.phase == "MOVE_ACTIVE" and ("FINAL RESULT:" in line or line.startswith("OPERATION FAILED:")):
            if "FAIL" in line:
                self.invalidate("angular operation failed")
            self._finish(line)
            self.deadline = now + 4.0

    def frame(self, now):
        """Timeout STOP or a bounded follow-up to the same deliberate request."""
        if not self.enabled:
            return None
        if self.phase not in ("READY", "FAULT") and (
                (self.deadline is not None and now >= self.deadline) or
                (self.last_rx is not None and now - self.last_rx > 3.0)):
            self.invalidate("autonomous acknowledgment/telemetry timed out")
            self.pending_id = self.purpose = self.continuation = None
            self.phase = "FAULT"
            self.deadline = None
            self.note += "; STOP sent. Toggle Y off/on for deliberate recovery."
            return b"STOP\n"
        if self.continuation is not None:
            command, self.continuation = self.continuation, None
            self.phase = "KEYMOVE_PENDING"
            self.deadline = now + 4.0
            return command
        return None

    def display_lines(self):
        captures = " | ".join(f"{slot}={self.frames[slot].steps if slot in self.frames else 'not saved'}" for slot in ("A", "B"))
        return [f"AUTO | {self.phase} | JOG DISABLED | increment {self.increment} deg | duration {self.duration} s",
                "LB (4): capture A | RB (5): capture B | X button (2): duration | Home (10): Play A->B",
                "LS click (8): return A | RS click (9): 1/2 deg | D-pad: up/down pitch, right/left yaw",
                "A/Space/F12: STOP | B / keyboard X: abort | Y (3): manual disarmed",
                f"Generated steps (startup-relative, unhomed): {captures}", self.note]
