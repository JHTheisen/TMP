"""Still-photo waypoints and camera events; never generates motor pulses."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PhotoSettings:
    segments: int = 8
    settle_seconds: float = 1.0
    post_seconds: float = 0.5

    def __post_init__(self):
        if isinstance(self.segments, bool) or not isinstance(self.segments, int) or not 1 <= self.segments <= 64:
            raise ValueError("Photo segments must be an integer from 1 to 64")
        for delay in (self.settle_seconds, self.post_seconds):
            if not math.isfinite(delay) or not 0 <= delay <= 60:
                raise ValueError("Photo delays must be between 0 and 60 seconds")


@dataclass(frozen=True)
class CameraTrigger:
    segment: int
    segments: int
    epoch: int
    steps: tuple
    requested_at: float


class SteppedPlayback:
    def __init__(self, start, end, settings, duration_seconds):
        if start.epoch != end.epoch:
            raise ValueError("Photo keyframes must have the same capture epoch")
        self.start, self.end, self.settings = start, end, settings
        self.index = 0
        self.segment_ms = max(1000, math.ceil(duration_seconds * 1000 / settings.segments))

    def target(self):
        """Next absolute generated-step waypoint; the last is exactly B."""
        n, total = self.index + 1, self.settings.segments
        def interpolate(a, b):
            delta = b - a
            amount = (abs(delta) * n * 2 + total) // (2 * total)
            return a + (amount if delta >= 0 else -amount)
        return tuple(interpolate(a, b) for a, b in zip(self.start.steps, self.end.steps))

    def trigger(self, now):
        self.index += 1
        return CameraTrigger(self.index, self.settings.segments, self.start.epoch,
                             self.last_target, now)

    @property
    def complete(self):
        return self.index == self.settings.segments
