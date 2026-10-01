"""Single-request manual/keyframe workflow; no pygame, serial or hardware access."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Keyframe:
    epoch: int
    steps: tuple
    heading: str
    pitch: str


class AutoSession:
    """Keyframe actions are available from manual; only active actions gate JOG."""
    def __init__(self):
        self.enabled = True
        self.phase = "READY"
        self.note = "Manual positioning; LB/RB capture, Home plays A to B."
        self.increment = 1
        self.duration = 10
        self.frames = {}
        self.epoch = None
        self.request_id = 0
        self.pending_id = self.purpose = self.deadline = self.last_rx = None
        self.continuation = None
        self.last_command = ""
        self.action = None
        self.play_duration = 10
        self.settle_until = None
        self.motion_started = None
        self.raw_command = None

    @property
    def busy(self):
        return self.phase not in ("READY", "OFF", "FAULT")

    @property
    def overridable(self):
        return self.busy and self.action not in (None, "capture_a", "capture_b")

    def invalidate(self, reason):
        self.frames.clear()
        self.epoch = None
        self.note = "Captures cleared: " + reason

    def cancel(self, now, reason="STOP sent; wait for READY."):
        self.phase = "STOPPING"
        self.pending_id = self.purpose = self.continuation = self.action = None
        self.settle_until = None
        self.deadline = now + 4.0
        self.note = reason

    def enter(self, now):
        self.enabled = True
        self.last_rx = now
        self.cancel(now)

    def leave(self, now):
        self.cancel(now)

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

    def _snapshot(self, now):
        ident = self._new_request(self.action, "SNAPSHOT", now)
        self.continuation = self._command(f"SNAP {ident}")

    def raw_sent(self, command, now):
        """Observe an explicitly sent autonomous command for the same takeover.

        This does not parse/admit its arguments or generate a second request.
        Idle raw editing still suppresses JOG; only an in-flight motion can be
        reclaimed by deliberate sticks.
        """
        words = command.split()
        if words and words[0] in ("LEVEL", "NORTH", "POSE", "MOVE", "KEYMOVE", "KEYRETURN"):
            self.action = "raw"
            self.raw_command = words[0]
            self.phase = "MOVE_PENDING"
            self.pending_id = self.continuation = None
            self.deadline = now + 4.0
            self.last_rx = now

    def _alignment(self):
        return self.action.upper() if self.action in ("level", "north") else (
            self.raw_command if self.action == "raw" and self.raw_command in ("LEVEL", "NORTH") else None)

    def request(self, action, now, centered):
        if not self.enabled or self.phase != "READY":
            self.note = "Not queued: wait for current action to finish and release controls."
            return None
        if action not in ("capture_a", "capture_b") and not centered:
            self.note = "Not queued: center sticks before requesting movement."
            return None
        if action in ("return_a", "play") and "A" not in self.frames:
            self.note = "Capture A first."
            return None
        if action == "play" and "B" not in self.frames:
            self.note = "Capture B first."
            return None
        self.action = action
        self.raw_command = None
        self.play_duration = self.duration
        self.phase = "PREFLIGHT_STOP"
        self.pending_id = self.continuation = None
        self.last_rx = now
        self.deadline = now + 4.0
        self.note = "Stopping manual motion before a fresh position request."
        return self._command("STOP")

    def _finish(self, note):
        self.phase = "FINISH_WAIT"
        self.pending_id = self.purpose = self.continuation = self.action = None
        self.note = note + " Waiting for READY."

    def receive(self, line, now):
        self.last_rx = now
        fields = dict(token.split("=", 1) for token in line.split() if "=" in token)
        if line.startswith(("M09_xbox_control:", "KEYFRAME_INVALIDATED ")) or line == "M09 ABORTED":
            self.invalidate(line)
            if line.startswith("KEYFRAME_INVALIDATED ") and self._alignment():
                # A forced watchdog stop loses captures but firmware may recover
                # this BNO-dependent command. Do not invent a persistent host latch.
                return
            self.cancel(now, self.note)
            if line == "M09 ABORTED":
                self.phase = "FAULT"
                self.deadline = None
                self.note += "; firmware abort is latched; reset required."
            return
        if line == "M09 READY":
            if self.phase == "PREFLIGHT_STOP":
                if self.action in ("capture_a", "capture_b", "return_a", "play"):
                    self._snapshot(now)
                elif self.action in ("level", "north"):
                    self._new_request(self.action, "MOVE_PENDING", now)
                    self.continuation = self._command(self.action.upper())
                else:
                    moves = {"pitch_up": (0, self.increment), "pitch_down": (0, -self.increment),
                             "yaw_right": (self.increment, 0), "yaw_left": (-self.increment, 0)}
                    yaw, pitch = moves[self.action]
                    self._new_request(self.action, "MOVE_PENDING", now)
                    self.continuation = self._command(f"MOVE {yaw} {pitch} 0")
            elif self.phase == "RETURN_WAIT":
                self.phase = "AT_A_SETTLE"
                self.settle_until = now + .2
                self.deadline = now + 4.0
                self.note = "Generated-step return complete; checking stopped A before timed playback."
            elif self.phase in ("STOPPING", "FINISH_WAIT"):
                self.phase = "READY"
                self.deadline = None
            elif self.action == "raw" and self.phase == "MOVE_ACTIVE":
                self.phase = "READY"
                self.pending_id = self.purpose = self.deadline = self.action = None
            return
        alignment = self._alignment()
        if alignment and line.startswith(alignment + " ACCEPTED") and self.phase == "MOVE_PENDING":
            self.phase = "MOVE_ACTIVE"
            self.motion_started = now
            self.deadline = None  # Firmware owns displacement/progress timing.
        if alignment and line.startswith((alignment + " ACCEPTED", alignment + " DEADLINE", alignment + " RESUMED")):
            if self.phase == "MOVE_ACTIVE":
                try:
                    duration = int(fields["deadline_ms"])
                    if 0 < duration < 2147483648:
                        self.deadline = self.motion_started + duration / 1000.0 + 5.0
                except (ValueError, KeyError):
                    pass
            return
        if alignment and line.startswith((alignment + " REJECTED:", alignment + " RESULT:")):
            self._finish(line)
            self.deadline = now + 4.0
            return
        if self.action == "raw" and line.startswith(("KEYMOVE ACCEPTED", "KEYMOVE RESULT", "KEYMOVE REJECTED", "KEYRETURN REJECTED")):
            if line.startswith("KEYMOVE ACCEPTED"):
                self.phase = "MOVE_ACTIVE"
                self.deadline = None  # Firmware deadline, plus the common receive lease below.
            else:
                self._finish(line)
                self.deadline = now + 4.0
            return
        matching = self.pending_id is not None and fields.get("id") == str(self.pending_id)
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
                self.invalidate("firmware position epoch changed")
                self._finish(self.note)
                return
            self.epoch = epoch
            snapshot = Keyframe(epoch, steps, fields.get("heading", "?"), fields.get("physical_pitch", "?"))
            if self.action in ("capture_a", "capture_b"):
                slot = "A" if self.action == "capture_a" else "B"
                self.frames[slot] = snapshot
                self.phase = "READY"
                self.pending_id = self.purpose = self.deadline = self.action = None
                self.note = f"Captured {slot}: steps {steps}; startup-relative and unhomed."
                return
            at_a = steps == self.frames["A"].steps
            if self.purpose == "verify_a" and not at_a:
                self._finish("Stopped A verification failed; playback canceled.")
                return
            travel = self.action == "return_a" or not at_a
            target = self.frames["A" if travel else "B"]
            if target.epoch != epoch:
                self.invalidate("keyframe epoch mismatch")
                self._finish(self.note)
                return
            ident = self._new_request("return" if travel else "play", "KEYMOVE_PREPARED", now)
            line = f"{'KEYRETURN' if travel else 'KEYMOVE'} {ident} {epoch} {target.steps[0]} {target.steps[1]} {target.steps[2]}"
            self.continuation = self._command(line if travel else line + f" {self.play_duration * 1000}")
            self.note = "Returning to A at travel speed." if travel else "Timed A to B; generated-step endpoints only."
        elif matching and line.startswith(("SNAP REJECTED ", "KEYMOVE REJECTED ", "KEYRETURN REJECTED ")):
            self._finish(line)
            self.deadline = now + 4.0
        elif matching and self.phase == "KEYMOVE_PENDING" and line.startswith("KEYMOVE ACCEPTED "):
            self.phase = "KEYMOVE_ACTIVE"
            self.deadline = now + (65 if self.purpose == "return" else self.play_duration + 5)
        elif matching and self.phase in ("KEYMOVE_PENDING", "KEYMOVE_ACTIVE") and line.startswith("KEYMOVE RESULT "):
            if fields.get("status") == "FAILED":
                self.invalidate("keyframe motion failed")
            if fields.get("status") == "PASS" and self.purpose == "return" and self.action == "play":
                self.phase = "RETURN_WAIT"
                self.pending_id = self.continuation = None
            else:
                self._finish(line)
            self.deadline = now + 4.0
        elif self.phase == "MOVE_PENDING" and line.startswith(("POSE ACCEPTED", "PITCH-ONLY ACCEPTED")):
            self.phase = "MOVE_ACTIVE"
            self.deadline = now + 95.0
        elif self.phase == "MOVE_PENDING" and line.startswith(("POSE REJECTED:", "POSE ALREADY AT TARGET:")):
            self._finish(line)
            self.deadline = now + 4.0
        elif self.phase == "MOVE_ACTIVE" and ("FINAL RESULT:" in line or line.startswith("OPERATION FAILED:")):
            self._finish(line)
            self.deadline = now + 4.0

    def frame(self, now):
        if self.busy and ((self.deadline is not None and now >= self.deadline) or
                          (self.last_rx is not None and now - self.last_rx > 3.0)):
            self.invalidate("autonomous acknowledgment/telemetry timed out; coordinate confidence unknown")
            self.cancel(now, self.note)
            self.phase = "FAULT"
            self.deadline = None
            return b"STOP\n"
        if self.phase == "AT_A_SETTLE" and now >= self.settle_until:
            self._snapshot(now)
            self.purpose = "verify_a"
        if self.continuation is not None:
            command, self.continuation = self.continuation, None
            if self.phase == "KEYMOVE_PREPARED":
                self.phase = "KEYMOVE_PENDING"
                self.deadline = now + 4.0
            return command
        return None

    def display_lines(self):
        captures = " | ".join(f"{slot}={self.frames[slot].steps if slot in self.frames else 'not saved'}" for slot in ("A", "B"))
        return [f"MANUAL + KEYFRAMES | {self.phase} | increment {self.increment} deg | duration {self.duration} s",
                "LB (4): capture A | RB (5): capture B | X button (2): duration | Home (10): Play A->B",
                "LS click (8): return A | RS click (9): 1/2 deg | D-pad: up/down pitch, right/left yaw",
                "A (0): LEVEL pitch | Y (3): NORTH yaw (magnetic)",
                "Space/F12: STOP | B / keyboard X: abort | Move sticks to take over automatic motion",
                f"Generated steps (startup-relative, unhomed): {captures}", self.note]
