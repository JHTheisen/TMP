"""Offline sky coordinates, deliberately independent of serial and motor control.

Input targets are fixed ICRS RA/Dec (catalog/J2000-like coordinates), not apparent
RA/Dec of date. Astropy owns Earth orientation, precession, nutation and the
topocentric transformation. Importing this module never imports Astropy; only
constructing AstropyConverter does, and the host does that on its worker thread.
"""
from dataclasses import dataclass
from datetime import datetime, timezone
import math
import re
import warnings


class AstronomyUnavailable(RuntimeError):
    """A local dependency/configuration failure affecting celestial work only."""


def _finite(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def _parse_angle(value, name, unit):
    """Decimal or sexagesimal input; this is notation parsing, not astronomy."""
    text = str(value).strip().lower()
    if re.fullmatch(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?", text):
        return _finite(text, name)
    # Explicit h/m/s or d/m/s, colon separated, or quoted whitespace separated.
    if unit in text:
        match = re.fullmatch(
            r"([+-]?\d+(?:\.\d*)?)\s*" + unit +
            r"(?:\s*(\d+(?:\.\d*)?)\s*m(?:\s*(\d+(?:\.\d*)?)\s*s)?)?", text)
        if not match:
            raise ValueError(f"Invalid {name}; use decimal {'hours' if unit == 'h' else 'degrees'} or sexagesimal notation")
        fields = [part for part in match.groups() if part is not None]
    else:
        separator = ":" if ":" in text else None
        fields = [part.strip() for part in text.split(separator)]
        if len(fields) not in (2, 3):
            raise ValueError(f"Invalid {name}; use decimal {'hours' if unit == 'h' else 'degrees'} or HH:MM:SS/DD:MM:SS")
    if not re.fullmatch(r"[+-]?\d+(?:\.\d*)?", fields[0]) or any(
            not re.fullmatch(r"\d+(?:\.\d*)?", part) for part in fields[1:]):
        raise ValueError(f"Invalid {name} sexagesimal components")
    numbers = [_finite(part, name) for part in fields]
    if len(numbers) > 1 and not numbers[0].is_integer():
        raise ValueError(f"{name}: use whole hours/degrees before minutes")
    if len(numbers) > 2 and not numbers[1].is_integer():
        raise ValueError(f"{name}: use whole minutes before seconds")
    if any(not 0 <= part < 60 for part in numbers[1:]):
        raise ValueError(f"{name} minutes and seconds must be in [0, 60)")
    magnitude = abs(numbers[0]) + sum(part / (60 ** index) for index, part in enumerate(numbers[1:], 1))
    # Preserve '-00:30:00': float(-00) alone loses the declination sign.
    return -magnitude if fields[0].startswith("-") else magnitude


@dataclass(frozen=True)
class Observer:
    latitude_deg: float
    longitude_deg: float  # East positive; west negative.
    elevation_m: float = 0.0

    def __post_init__(self):
        for field in ("latitude_deg", "longitude_deg", "elevation_m"):
            object.__setattr__(self, field, _finite(getattr(self, field), field))
        if not -90 <= self.latitude_deg <= 90:
            raise ValueError("Observer latitude must be between -90 and 90 degrees")
        if not -180 <= self.longitude_deg <= 180:
            raise ValueError("Observer longitude must be between -180 and 180 degrees (east positive)")

    def describe(self):
        return (f"observer lat={self.latitude_deg:.6f} lon={self.longitude_deg:.6f} "
                f"elevation_m={self.elevation_m:.1f}; longitude east positive; host UTC")


@dataclass(frozen=True)
class EquatorialTarget:
    ra_hours: float
    dec_deg: float

    def __post_init__(self):
        object.__setattr__(self, "ra_hours", _finite(self.ra_hours, "RA") % 24.0)
        object.__setattr__(self, "dec_deg", _finite(self.dec_deg, "Dec"))
        if not -90 <= self.dec_deg <= 90:
            raise ValueError("Declination must be between -90 and 90 degrees")

    @classmethod
    def parse(cls, ra, dec):
        return cls(_parse_angle(ra, "RA", "h"), _parse_angle(dec, "Dec", "d"))

    def describe(self):
        return f"ICRS RA={self.ra_hours:.8f}h Dec={self.dec_deg:+.8f}deg"


def utc_datetime(value=None):
    if value is None:
        return datetime.now(timezone.utc)
    if isinstance(value, str):
        try:
            value = datetime.fromisoformat(value.strip().replace("Z", "+00:00"))
        except ValueError as exc:
            raise ValueError("UTC time must be an ISO timestamp with Z or a timezone offset") from exc
    if not isinstance(value, datetime) or value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("Celestial time must be a timezone-aware datetime or ISO timestamp")
    return value.astimezone(timezone.utc)


@dataclass(frozen=True)
class HorizontalTarget:
    azimuth_deg: float  # True/geographic north = 0; east = 90.
    altitude_deg: float
    utc: datetime = None
    warnings: tuple = ()

    def __post_init__(self):
        object.__setattr__(self, "azimuth_deg", _finite(self.azimuth_deg, "Azimuth") % 360.0)
        object.__setattr__(self, "altitude_deg", _finite(self.altitude_deg, "Altitude"))
        if not -90 <= self.altitude_deg <= 90:
            raise ValueError("Altitude must be between -90 and 90 degrees")
        if self.utc is not None:
            object.__setattr__(self, "utc", utc_datetime(self.utc))

    @property
    def below_horizon(self):
        return self.altitude_deg < 0


@dataclass(frozen=True)
class MountTarget:
    heading_deg: float  # Desired absolute BNO magnetic heading, as in POSE.
    pitch_deg: float  # Desired BNO physical Euler pitch, NOT generated steps.


@dataclass(frozen=True)
class HeadingReference:
    """Keep true sky, BNO orientation and optical alignment explicitly separate.

    true optical azimuth = heading_direction * BNO heading + declination + heading_offset
    optical altitude = BNO physical pitch + pitch_offset

    Declination is east-positive. None retains an explicitly reported raw
    magnetic approximation; it does not claim magnetic north is true north.
    Boresight offsets describe the optical axis relative to BNO, not motor signs.
    """
    magnetic_declination_deg: float = None
    heading_offset_deg: float = 0.0
    pitch_offset_deg: float = 0.0
    heading_direction: int = 1

    def __post_init__(self):
        for field in ("magnetic_declination_deg", "heading_offset_deg", "pitch_offset_deg"):
            value = getattr(self, field)
            if value is not None or field != "magnetic_declination_deg":
                object.__setattr__(self, field, _finite(value, field))
        direction = _finite(self.heading_direction, "Heading direction")
        if direction not in (-1, 1):
            raise ValueError("Heading direction must be +1 or -1; it maps BNO angles, not motor signs")
        object.__setattr__(self, "heading_direction", int(direction))

    def describe(self):
        reference = ("raw magnetic NORTH approximation (true-north correction unconfigured)"
                     if self.magnetic_declination_deg is None else
                     f"configured true-north correction declination={self.magnetic_declination_deg:+.3f}deg east")
        return (f"{reference}; BNO-to-azimuth direction={self.heading_direction:+d}; "
                f"heading_offset={self.heading_offset_deg:+.3f}deg pitch_offset={self.pitch_offset_deg:+.3f}deg")

    def mount_target(self, horizontal):
        if horizontal.below_horizon:
            raise ValueError(f"Target is below the horizon (altitude={horizontal.altitude_deg:.3f}deg)")
        heading = ((horizontal.azimuth_deg - (self.magnetic_declination_deg or 0.0) % 360.0 -
                    self.heading_offset_deg % 360.0) / self.heading_direction) % 360.0
        pitch = horizontal.altitude_deg - self.pitch_offset_deg
        # Existing combined POSE absolute pitch guard is strict at +/-75 degrees.
        # Firmware separately checks its continuous yaw/travel reference; a
        # wrapped heading alone cannot prove that an unwound route is reachable.
        if not -75.0 < pitch < 75.0:
            raise ValueError(f"Target BNO pitch {pitch:.3f}deg reaches the existing +/-75deg pitch guard")
        return MountTarget(heading, pitch)

    def horizontal_from_mount(self, heading_deg, pitch_deg, utc=None):
        """Invert the existing optical/BNO mapping for a captured pointing."""
        heading = _finite(heading_deg, "BNO heading")
        pitch = _finite(pitch_deg, "BNO physical pitch")
        if not 0 <= heading <= 360:
            raise ValueError("BNO heading must be between 0 and 360 degrees")
        if not -90 <= pitch <= 90:
            raise ValueError("BNO physical pitch must be between -90 and 90 degrees")
        azimuth = (self.heading_direction * heading + (self.magnetic_declination_deg or 0.0) +
                   self.heading_offset_deg) % 360.0
        altitude = pitch + self.pitch_offset_deg
        return HorizontalTarget(azimuth, altitude, utc)


def shortest_difference(target, current):
    """Same [-180, 180) convention as firmware; independent of motor signs."""
    target = _finite(target, "Target heading") % 360.0
    current = _finite(current, "Current heading") % 360.0
    return (target - current + 180.0) % 360.0 - 180.0


class AstropyConverter:
    """Instantiate/use on the astronomy worker, never in the operator loop."""

    def __init__(self, observer):
        try:
            from astropy import units
            from astropy.coordinates import AltAz, EarthLocation, SkyCoord
            from astropy.time import Time
            from astropy.utils import iers
        except (ImportError, OSError) as exc:
            raise AstronomyUnavailable(
                "Celestial coordinates require Astropy; install requirements.txt with Python 3.11+; manual control remains available"
            ) from exc
        # Bundled Earth-orientation/leap-second data only. Outdated predictions
        # produce diagnostics rather than a hidden network stall or manual fault.
        iers.conf.auto_download = False
        iers.conf.auto_max_age = None
        iers.conf.iers_degraded_accuracy = "warn"
        self.observer = observer
        self._units, self._AltAz, self._SkyCoord, self._Time = units, AltAz, SkyCoord, Time
        self._iers = iers
        self._location = EarthLocation.from_geodetic(
            lon=observer.longitude_deg * units.deg, lat=observer.latitude_deg * units.deg,
            height=observer.elevation_m * units.m)
        self._target = self._coordinate = None

    def altaz(self, target, utc=None):
        instant = utc_datetime(utc)
        units = self._units
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            if target != self._target:
                self._coordinate = self._SkyCoord(
                    ra=target.ra_hours * units.hourangle, dec=target.dec_deg * units.deg, frame="icrs")
                self._target = target
            time = self._Time(instant, scale="utc")
            # pressure=0 intentionally excludes atmospheric refraction: no local
            # weather is assumed. All celestial/Earth frame math stays in Astropy.
            horizontal = self._coordinate.transform_to(self._AltAz(
                obstime=time, location=self._location, pressure=0 * units.hPa))
            _, status = self._iers.earth_orientation_table.get().ut1_utc(time, return_status=True)
        notes = list(dict.fromkeys(str(item.message) for item in caught))
        if int(status) in (self._iers.TIME_BEFORE_IERS_RANGE, self._iers.TIME_BEYOND_IERS_RANGE):
            notes.append("UTC is outside bundled IERS coverage; pointing accuracy is degraded; update astropy-iers-data when online")
        return HorizontalTarget(float(horizontal.az.deg), float(horizontal.alt.deg), instant, tuple(notes))

    def equatorial(self, horizontal, utc=None):
        """Convert a captured local boresight into the fixed ICRS target to track."""
        instant = utc_datetime(utc or horizontal.utc)
        units = self._units
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            frame = self._AltAz(obstime=self._Time(instant, scale="utc"), location=self._location,
                                pressure=0 * units.hPa)
            local = self._SkyCoord(az=horizontal.azimuth_deg * units.deg,
                                   alt=horizontal.altitude_deg * units.deg, frame=frame)
            coordinate = local.icrs
        # Warnings are surfaced by the normal forward calculation immediately
        # after capture; the fixed target itself stays independent of UI state.
        return EquatorialTarget(float(coordinate.ra.hour), float(coordinate.dec.deg))
