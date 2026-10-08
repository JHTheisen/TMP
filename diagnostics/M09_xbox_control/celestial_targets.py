"""Offline named sky targets and display-only horizon previews; no motor I/O.

PyEphem's built-in solar-system ephemerides include Pluto. Its bundled bright
star catalog supplies J2000 positions and proper motions; no name resolver or
download is used. See https://rhodesmill.org/pyephem/quick.html and /radec.html.
"""
from dataclasses import dataclass
import math
import queue
import threading

from celestial_coordinates import AstronomyUnavailable, HorizontalTarget, utc_datetime


@dataclass(frozen=True)
class NamedTarget:
    name: str
    kind: str

    def describe(self):
        return self.name


SOLAR_TARGETS = tuple(NamedTarget(name, "solar") for name in (
    "Sun", "Moon", "Mercury", "Venus", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune", "Pluto"))
STAR_TARGETS = tuple(NamedTarget(name, "star") for name in (
    "Polaris", "Vega", "Capella", "Arcturus", "Sirius"))
TARGETS = {target.name: target for target in SOLAR_TARGETS + STAR_TARGETS}


class NamedTargetConverter:
    """Compute topocentric apparent positions at each requested UTC instant."""
    def __init__(self, observer):
        try:
            import ephem
        except (ImportError, OSError) as error:
            raise AstronomyUnavailable("Named targets require ephem; install requirements.txt") from error
        self._ephem = ephem
        self._observer = ephem.Observer()
        self._observer.lat = math.radians(observer.latitude_deg)
        self._observer.lon = math.radians(observer.longitude_deg)
        self._observer.elevation = observer.elevation_m
        self._observer.pressure = 0  # Same unrefracted geometric horizon as manual RA/Dec.
        self._bodies = {}

    def altaz(self, target, utc=None):
        if TARGETS.get(target.name) != target:
            raise ValueError("Unknown named celestial target")
        instant = utc_datetime(utc)
        # PyEphem datetime input is UTC without a timezone object.
        self._observer.date = instant.replace(tzinfo=None)
        if target.name not in self._bodies:
            self._bodies[target.name] = (self._ephem.star(target.name) if target.kind == "star"
                                         else getattr(self._ephem, target.name)())
        body = self._bodies[target.name]
        body.compute(self._observer)
        # Do not treat apparent RA/Dec as fixed ICRS or lose lunar parallax by
        # round-tripping through an infinite-distance star coordinate.
        return HorizontalTarget(math.degrees(float(body.az)), math.degrees(float(body.alt)), instant)


@dataclass(frozen=True)
class TargetSnapshot:
    requested_at: float
    positions: dict
    errors: dict


class TargetPreview:
    """Independent, bounded UI work; never shares the tracking worker's mailbox."""
    REFRESH_SECONDS = 15.0
    MAX_AGE = 30.0

    def __init__(self, observer, converter_factory=NamedTargetConverter, utc_clock=utc_datetime):
        self.observer = observer
        self._factory, self._utc_clock = converter_factory, utc_clock
        self._results = queue.Queue(maxsize=1)
        self._pending = False
        self._next_at = float("-inf")
        self.latest = None

    def poll(self, now):
        try:
            self.latest = self._results.get_nowait()
            self._pending = False
        except queue.Empty:
            pass
        if not self._pending and now >= self._next_at:
            self._next_at = now + self.REFRESH_SECONDS
            self._pending = True
            try:
                threading.Thread(target=self._calculate, args=(now,), name="target-preview", daemon=True).start()
            except RuntimeError as error:
                self._pending = False
                self.latest = TargetSnapshot(now, {}, {name: str(error) for name in TARGETS})
        return self.latest

    def _calculate(self, now):
        positions, errors = {}, {}
        try:
            converter, utc = self._factory(self.observer), self._utc_clock()
            for name, target in TARGETS.items():
                try:
                    positions[name] = converter.altaz(target, utc)
                except Exception as error:
                    errors[name] = str(error)
        except Exception as error:
            errors = {name: str(error) for name in TARGETS}
        self._results.put_nowait(TargetSnapshot(now, positions, errors))
