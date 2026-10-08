"""RA/Dec notation, reference frames and real offline Astropy transformations."""
from datetime import datetime, timedelta, timezone
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from celestial_coordinates import (
    AstronomyUnavailable, AstropyConverter, EquatorialTarget, HeadingReference,
    HorizontalTarget, Observer, shortest_difference, utc_datetime,
)


class CoordinateInputTests(unittest.TestCase):
    def test_north_alignment_method_requires_explicit_declination(self):
        reference = HeadingReference(5, 2, 1, north_reference="magnetic")
        mount = reference.mount_target(HorizontalTarget(97, 31))
        self.assertEqual(mount.heading_deg, 90)
        self.assertEqual(mount.pitch_deg, 30)
        for kwargs in ({"north_reference": "magnetic"}, {"magnetic_declination_deg": 5},
                       {"north_reference": "invalid"}):
            with self.assertRaises(ValueError):
                HeadingReference(**kwargs)


    def test_decimal_hours_and_degrees_are_unambiguous(self):
        target = EquatorialTarget.parse("5.5", "-22.25")
        self.assertEqual(target.ra_hours, 5.5)
        self.assertEqual(target.dec_deg, -22.25)
        self.assertIn("ICRS", target.describe())

    def test_sexagesimal_colons_units_and_quoted_spaces(self):
        for ra in ("5:30:15", "05h30m15s", "5 30 15"):
            for dec in ("-22:15:30", "-22d15m30s", "-22 15 30"):
                with self.subTest(ra=ra, dec=dec):
                    target = EquatorialTarget.parse(ra, dec)
                    self.assertAlmostEqual(target.ra_hours, 5.5041666667)
                    self.assertAlmostEqual(target.dec_deg, -22.2583333333)

    def test_two_components_and_fractional_seconds(self):
        target = EquatorialTarget.parse("05:30", "+22:15:30.5")
        self.assertEqual(target.ra_hours, 5.5)
        self.assertAlmostEqual(target.dec_deg, 22.2584722222)
        self.assertEqual(EquatorialTarget.parse("5.5h", "22.5d"), EquatorialTarget(5.5, 22.5))

    def test_negative_subdegree_declination_keeps_sign(self):
        for dec in ("-00:30:00", "-00d30m00s", "-00 30 00"):
            self.assertEqual(EquatorialTarget.parse("0", dec).dec_deg, -0.5)

    def test_ra_wraps_without_wrapping_declination(self):
        for ra, expected in (("24", 0), ("25:30:00", 1.5), ("-00:30:00", 23.5), ("48", 0)):
            self.assertEqual(EquatorialTarget.parse(ra, "-90").ra_hours, expected)
        self.assertEqual(EquatorialTarget.parse("0", "90").dec_deg, 90)
        for dec in ("90:00:01", "-90:00:01", "91", "-100"):
            with self.assertRaises(ValueError, msg=dec):
                EquatorialTarget.parse("1", dec)

    def test_invalid_notation_and_nonfinite_values_are_rejected(self):
        for ra, dec in (("nan", "0"), ("inf", "0"), ("0", "-inf"), ("", "0"),
                        ("1:60:00", "0"), ("1:10:60", "0"), ("1", "2:60"),
                        ("1:2:3:4", "0"), ("1:-2:3", "0"), ("--1", "0"),
                        ("1h2d", "0"), ("1d", "0"), ("1", "2h"),
                        ("1_2", "0"), ("1.5:20:30", "0"), ("1:2.5:30", "0"),
                        ("1e999", "0"), ("1", ""), ("None", "0")):
            with self.subTest(ra=ra, dec=dec), self.assertRaises(ValueError):
                EquatorialTarget.parse(ra, dec)
        for value in (float("nan"), float("inf"), None):
            with self.assertRaises(ValueError):
                EquatorialTarget(value, 0)

    def test_observer_validation_and_east_positive_description(self):
        observer = Observer(41.3, -74, 390)
        self.assertEqual(observer.longitude_deg, -74)
        self.assertIn("east positive", observer.describe())
        self.assertEqual(Observer(-90, 180).elevation_m, 0)
        for values in ((91, 0), (-91, 0), (0, 181), (0, -181), (0, float("nan")), (0, 0, float("inf"))):
            with self.assertRaises(ValueError, msg=values):
                Observer(*values)

    def test_utc_explicit_offsets_and_naive_time_rejection(self):
        utc = datetime(2012, 7, 13, 3, tzinfo=timezone.utc)
        self.assertEqual(utc_datetime("2012-07-12T23:00:00-04:00"), utc)
        self.assertEqual(utc_datetime("2012-07-13T03:00:00Z"), utc)
        self.assertEqual(utc_datetime(utc), utc)
        for instant in ("2012-07-13T03:00:00", "bad clock", datetime(2012, 7, 13), 0):
            with self.assertRaises(ValueError):
                utc_datetime(instant)
        before = datetime.now(timezone.utc)
        now = utc_datetime()
        self.assertGreaterEqual(now, before)
        self.assertLessEqual(now, datetime.now(timezone.utc))

    def test_module_import_and_parsing_work_without_site_packages(self):
        code = ("import sys; from celestial_coordinates import EquatorialTarget; "
                "assert EquatorialTarget.parse('5:30:00','-00:30:00').dec_deg == -.5; "
                "assert not any(m.startswith('astropy') for m in sys.modules)")
        result = subprocess.run([sys.executable, "-S", "-B", "-c", code],
                                cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_missing_astropy_has_local_useful_error(self):
        original = __import__

        def missing(name, *args, **kwargs):
            if name.startswith("astropy"):
                raise ModuleNotFoundError("No module named 'astropy'")
            return original(name, *args, **kwargs)

        with mock.patch("builtins.__import__", side_effect=missing):
            # Other config/parsing remains usable with the optional library gone.
            self.assertEqual(EquatorialTarget.parse("1", "-2").dec_deg, -2)
            with self.assertRaisesRegex(AstronomyUnavailable, "manual control remains available"):
                AstropyConverter(Observer(0, 0))


class ReferenceTests(unittest.TestCase):
    def test_true_north_calibration_requirement_is_reported(self):
        reference = HeadingReference()
        self.assertIn("aligned to true north", reference.describe())
        mapped = reference.mount_target(HorizontalTarget(100, 25))
        self.assertEqual((mapped.heading_deg, mapped.pitch_deg), (100, 25))

    def test_declination_and_optical_offsets_remain_distinct(self):
        reference = HeadingReference(10, 3, -2, north_reference="magnetic")
        self.assertIn("configured true-north", reference.describe())
        mapped = reference.mount_target(HorizontalTarget(100, 25))
        self.assertEqual((mapped.heading_deg, mapped.pitch_deg), (87, 27))
        self.assertEqual(HeadingReference(-10, north_reference="magnetic").mount_target(HorizontalTarget(355, 25)).heading_deg, 5)
        self.assertIn("configured true-north", HeadingReference(0, north_reference="magnetic").describe())

    def test_mount_to_horizontal_is_the_exact_inverse_used_by_track_here(self):
        for reference in (HeadingReference(), HeadingReference(12.5, -7.25, 3.5, "magnetic")):
            for azimuth, altitude in ((0, 0), (123.456, 42.25), (359.9, 12.5)):
                horizontal = HorizontalTarget(azimuth, altitude)
                mount = reference.mount_target(horizontal)
                captured = reference.horizontal_from_mount(mount.heading_deg, mount.pitch_deg)
                self.assertAlmostEqual(captured.azimuth_deg, azimuth)
                self.assertAlmostEqual(captured.altitude_deg, altitude)

    def test_shortest_wrap_crossings_both_directions(self):
        self.assertAlmostEqual(shortest_difference(0.1, 359.9), 0.2)
        self.assertAlmostEqual(shortest_difference(359.9, 0.1), -0.2)
        self.assertEqual(shortest_difference(180, 0), -180)
        self.assertEqual(shortest_difference(721, -1), 2)
        with self.assertRaises(ValueError):
            shortest_difference(float("nan"), 0)

    def test_below_horizon_and_mechanical_pitch_refusals(self):
        self.assertTrue(HorizontalTarget(0, -0.01).below_horizon)
        self.assertFalse(HorizontalTarget(0, 0).below_horizon)
        with self.assertRaisesRegex(ValueError, "below the horizon"):
            HeadingReference().mount_target(HorizontalTarget(0, -1))
        with self.assertRaisesRegex(ValueError, "pitch guard"):
            HeadingReference().mount_target(HorizontalTarget(0, 75))
        with self.assertRaisesRegex(ValueError, "pitch guard"):
            HeadingReference(pitch_offset_deg=80).mount_target(HorizontalTarget(0, 5))
        self.assertEqual(HeadingReference().mount_target(HorizontalTarget(0, 74.9)).pitch_deg, 74.9)

    def test_nonfinite_reference_or_result_rejected(self):
        for kwargs in ({"magnetic_declination_deg": float("nan")}, {"heading_offset_deg": float("inf")},
                       {"pitch_offset_deg": None}):
            with self.assertRaises(ValueError):
                HeadingReference(**kwargs)
        for az, alt in ((float("nan"), 0), (0, float("inf")), (0, 91)):
            with self.assertRaises(ValueError):
                HorizontalTarget(az, alt)

    def test_large_finite_angle_wrapping_cannot_overflow(self):
        difference = shortest_difference(1e308, -1e308)
        self.assertTrue(-180 <= difference < 180)
        mapped = HeadingReference(-1e308, -1e308, north_reference="magnetic").mount_target(HorizontalTarget(350, 25))
        self.assertTrue(0 <= mapped.heading_deg < 360)


@unittest.skipUnless(importlib.util.find_spec("astropy"), "Install requirements.txt to run real astronomy conversions")
class AstropyConversionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.observer = Observer(41.3, -74, 390)
        cls.target = EquatorialTarget(23.46206906 / 15, 30.66017511)
        cls.start = datetime(2012, 7, 13, 3, tzinfo=timezone.utc)
        cls.converter = AstropyConverter(cls.observer)

    def test_published_m33_bear_mountain_reference(self):
        # Independent expected value published by Astropy's observing example:
        # https://docs.astropy.org/en/stable/coordinates/example_gallery_plot_obs_planning.html
        # M33, 2012-07-12 23:00 EDT, Bear Mountain: altitude 0.13 degrees.
        point = self.converter.altaz(self.target, self.start)
        self.assertAlmostEqual(point.altitude_deg, 0.13, delta=0.005)
        self.assertTrue(45 < point.azimuth_deg < 50)  # Rising in the northeast.
        self.assertEqual(point.utc, self.start)
        self.assertFalse(point.below_horizon)

    def test_both_axes_evolve_smoothly_over_five_minutes(self):
        points = [self.converter.altaz(self.target, self.start + timedelta(seconds=n)) for n in range(0, 301, 30)]
        for first, second in zip(points, points[1:]):
            self.assertTrue(0 < second.altitude_deg - first.altitude_deg < 0.1)
            self.assertTrue(0 < shortest_difference(second.azimuth_deg, first.azimuth_deg) < 0.1)
        self.assertTrue(0.5 < points[-1].altitude_deg - points[0].altitude_deg < 1)
        self.assertTrue(0.5 < points[-1].azimuth_deg - points[0].azimuth_deg < 1)

    def test_before_rise_is_below_horizon(self):
        point = self.converter.altaz(self.target, self.start - timedelta(hours=1))
        self.assertTrue(point.below_horizon)
        with self.assertRaisesRegex(ValueError, "below the horizon"):
            HeadingReference().mount_target(point)

    def test_negative_declination_and_ra_wrap_produce_identical_results(self):
        first = self.converter.altaz(EquatorialTarget.parse("-1:30:00", "-16:42:58"), self.start)
        second = self.converter.altaz(EquatorialTarget.parse("22:30:00", "-16:42:58"), self.start)
        self.assertAlmostEqual(first.azimuth_deg, second.azimuth_deg)
        self.assertAlmostEqual(first.altitude_deg, second.altitude_deg)

    def test_no_network_requests_during_initialization_or_transforms(self):
        with mock.patch("socket.socket.connect", side_effect=AssertionError("Astronomy attempted network access")) as connect:
            converter = AstropyConverter(Observer(-30, 18, 100))
            converter.altaz(EquatorialTarget(5, -20), "2026-10-01T00:00:00Z")
            connect.assert_not_called()
        from astropy.utils import iers
        self.assertFalse(iers.conf.auto_download)

    def test_captured_altaz_round_trips_through_fixed_icrs_target(self):
        original = HorizontalTarget(218.25, 37.5, self.start)
        target = self.converter.equatorial(original, self.start)
        restored = self.converter.altaz(target, self.start)
        self.assertAlmostEqual(restored.azimuth_deg, original.azimuth_deg, places=7)
        self.assertAlmostEqual(restored.altitude_deg, original.altitude_deg, places=7)

    def test_outside_bundled_iers_coverage_has_explicit_diagnostic(self):
        point = self.converter.altaz(self.target, "2100-01-01T00:00:00Z")
        self.assertTrue(any("outside bundled IERS coverage" in warning for warning in point.warnings))
        self.assertTrue(0 <= point.azimuth_deg < 360)
        self.assertTrue(-90 <= point.altitude_deg <= 90)


if __name__ == "__main__":
    unittest.main()
