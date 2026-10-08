"""Nonblocking launch/reboot geometry setup; never captures zero references."""


class EncoderSetup:
    def __init__(self, yaw_ratio=None, pitch_ratio=None):
        self.command = (None if yaw_ratio is None else
                        f"ENCODER_CONFIG {yaw_ratio * 360:.12g} {pitch_ratio * 360:.12g}\n".encode("ascii"))
        self.reset()

    def reset(self):
        self.phase = "stop" if self.command else "done"
        self.ready = self.protocol = False
        self.started = None
        self.error = None

    @property
    def pending(self):
        return self.phase not in ("done", "aborted")

    def receive(self, line):
        if line.startswith("M09_xbox_control:"):
            self.reset()
        elif line == "M09 ABORTED":
            self.phase = "aborted"
        elif self.pending:
            if line.startswith("ORIENTATION_STATE "):
                fields = dict(token.split("=", 1) for token in line.split() if "=" in token)
                self.protocol = fields.get("protocol") == "2" and fields.get("feedback") == "AS5600"
            elif line == "M09 READY":
                self.ready = True
                if self.phase == "ack_ready":
                    self.phase = "done"
            elif line in ("M09 BUSY", "M09 MANUAL"):
                self.ready = False
            elif self.phase == "ack" and line.startswith("CALIBRATION CONFIGURED:"):
                self.phase = "ack_ready"
            elif self.phase == "ack" and line.startswith("CALIBRATION REJECTED:"):
                self.error = line

    def frame(self, now):
        if not self.pending:
            return None
        if self.started is None:
            self.started = now
        if self.error or now - self.started > 10:
            raise RuntimeError(self.error or "Encoder setup timed out; matching protocol-2 firmware and stopped READY required")
        if self.phase == "stop":
            self.phase, self.ready = "ready", False
            return b"STOP\n"
        if self.phase == "ready" and self.ready and self.protocol:
            self.phase = "ack"
            return self.command
        return None
