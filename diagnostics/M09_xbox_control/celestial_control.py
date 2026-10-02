"""Bounded, asynchronous sky-coordinate preparation; never accesses serial/motors.

One daemon worker owns the optional astronomy library. Cancellation invalidates
results immediately; the UI never waits for imports, calculations or thread exit.
"""
from dataclasses import dataclass
from datetime import datetime, timezone
import queue
import threading

from celestial_coordinates import AstropyConverter, EquatorialTarget

UPDATE_INTERVAL = 1.0
RESULT_MAX_AGE = 2.0
UPDATE_TIMEOUT = 2.5  # Stop before the firmware's independent 3-second target lease.


def parse_tracking_command(command):
    words = command.split()
    if len(words) != 3 or words[0].upper() != "TRACK_RADEC":
        raise ValueError("Use TRACK_RADEC <RA hours or hh:mm:ss> <Dec degrees or +/-dd:mm:ss>.")
    return EquatorialTarget.parse(words[1], words[2])


@dataclass(frozen=True)
class Calculation:
    generation: int
    requested_at: float
    utc: object = None
    horizontal: object = None
    mount: object = None
    error: str = ""


class CoordinateWorker:
    def __init__(self, observer, reference, converter_factory=AstropyConverter, utc_clock=None):
        self.observer, self.reference = observer, reference
        self._factory = converter_factory
        self._utc_clock = utc_clock or (lambda: datetime.now(timezone.utc))
        self._jobs, self._results = queue.Queue(maxsize=1), queue.Queue(maxsize=1)
        self._thread = None
        self._closed = threading.Event()

    @staticmethod
    def _replace(mailbox, item):
        try:
            mailbox.get_nowait()
        except queue.Empty:
            pass
        mailbox.put_nowait(item)

    def submit(self, generation, target, now):
        self._replace(self._jobs, (generation, target, now))
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="celestial-coordinates", daemon=True)
            self._thread.start()

    def poll(self):
        try:
            return self._results.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self._closed.set()
        self._replace(self._jobs, None)

    def _run(self):
        converter = None
        while not self._closed.is_set():
            job = self._jobs.get()
            if job is None:
                return
            generation, target, requested_at = job
            try:
                if converter is None:
                    converter = self._factory(self.observer)
                utc = self._utc_clock()
                horizontal = converter.altaz(target, utc)
                mount = self.reference.mount_target(horizontal)
                result = Calculation(generation, requested_at, utc, horizontal, mount)
            except Exception as error:
                result = Calculation(generation, requested_at, error=f"{type(error).__name__}: {error}")
            self._replace(self._results, result)


class CelestialTracker:
    """Latest-result mailbox and cadence. AutoSession owns lifecycle and STOP."""
    def __init__(self, observer, reference, worker=None):
        self.observer, self.reference = observer, reference
        self.worker = worker or CoordinateWorker(observer, reference)
        self.generation = 0
        self.target = None
        self.pending = False
        self.latest = None
        self.last_sent = None
        self.sequence = 0

    def begin(self, target, now):
        self.cancel()
        self.target = target
        self.latest = None
        self.last_sent = None
        self.sequence = 0
        self._request(now)

    def cancel(self):
        self.generation += 1
        self.pending = False
        self.target = None

    def _request(self, now):
        self.worker.submit(self.generation, self.target, now)
        self.pending = True

    def result(self, now):
        result = self.worker.poll()
        if result is None or result.generation != self.generation or self.target is None:
            return None
        self.pending = False
        if result.error:
            raise ValueError(result.error)
        if now - result.requested_at > RESULT_MAX_AGE:
            # Initialization can be slow. A new calculation uses current UTC;
            # never point using the timestamp from an old startup job.
            self._request(now)
            return None
        self.latest = result
        return result

    def update(self, now):
        if self.last_sent is not None and now - self.last_sent >= UPDATE_TIMEOUT:
            raise ValueError("sky target calculation/update delayed; tracking canceled")
        result = self.result(now)
        if result is not None:
            return result
        if not self.pending and self.last_sent is not None and now - self.last_sent >= UPDATE_INTERVAL:
            self._request(now)
        return None

    def sent(self, now):
        self.last_sent = now

    def close(self):
        self.cancel()
        self.worker.close()
