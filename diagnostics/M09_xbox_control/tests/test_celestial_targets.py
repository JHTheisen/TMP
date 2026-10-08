"""Named-target astronomy, preview isolation, and the existing GOTO/TRACK path."""
from datetime import datetime, timedelta, timezone
import math
import os
from pathlib import Path
import sys
import unittest
from unittest import mock

os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from celestial_coordinates import Observer, HeadingReference, HorizontalTarget, AstropyConverter, EquatorialTarget, shortest_difference
from celestial_targets import TARGETS, SOLAR_TARGETS, STAR_TARGETS, NamedTargetConverter, TargetPreview, TargetSnapshot
from celestial_control import CoordinateWorker
import test_celestial_ui as ui_fixture
import test_operator_dashboard as dashboard_fixture

UTC = datetime(2026, 10, 7, 17, tzinfo=timezone.utc)
OBSERVER = Observer(43.9425, -86.0394, 200)


class NamedAstronomyTests(unittest.TestCase):
    def test_all_requested_bodies_are_offline_finite_positions(self):
        self.assertEqual([target.name for target in SOLAR_TARGETS],
                         ["Sun", "Moon", "Mercury", "Venus", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune", "Pluto"])
        self.assertEqual([target.name for target in STAR_TARGETS], ["Polaris", "Vega", "Capella", "Arcturus", "Sirius"])
        with mock.patch("socket.socket", side_effect=AssertionError("No network")):
            converter = NamedTargetConverter(OBSERVER)
            for target in TARGETS.values():
                with self.subTest(target=target.name):
                    result = converter.altaz(target, UTC)
                    self.assertTrue(0 <= result.azimuth_deg < 360)
                    self.assertTrue(-90 <= result.altitude_deg <= 90)
                    self.assertEqual(result.utc, UTC)
                    self.assertEqual(result.below_horizon, result.altitude_deg < 0)

    def test_sun_moon_and_planet_agree_with_independent_astropy_ephemeris(self):
        from astropy import units as u
        from astropy.coordinates import AltAz, EarthLocation, get_body
        from astropy.time import Time
        AstropyConverter(OBSERVER)  # Applies the application's existing offline IERS settings.
        location = EarthLocation.from_geodetic(OBSERVER.longitude_deg*u.deg, OBSERVER.latitude_deg*u.deg,
                                               OBSERVER.elevation_m*u.m)
        time = Time(UTC)
        converter = NamedTargetConverter(OBSERVER)
        for name in ("Sun", "Moon", "Jupiter"):
            with self.subTest(name=name):
                expected = get_body(name.lower(), time, location, ephemeris="builtin").transform_to(
                    AltAz(obstime=time, location=location, pressure=0*u.hPa))
                actual = converter.altaz(TARGETS[name], UTC)
                self.assertLess(abs(shortest_difference(actual.azimuth_deg, expected.az.deg)), 0.1)
                self.assertAlmostEqual(actual.altitude_deg, expected.alt.deg, delta=0.1)

    def test_catalog_vega_matches_manual_icrs_and_solar_bodies_are_not_fixed_stars(self):
        import ephem
        converter = NamedTargetConverter(OBSERVER)
        vega = converter.altaz(TARGETS["Vega"], UTC)
        expected = AstropyConverter(OBSERVER).altaz(EquatorialTarget.parse("18:36:56.336", "+38:47:01.28"), UTC)
        self.assertAlmostEqual(vega.altitude_deg, expected.altitude_deg, delta=.02)
        self.assertLess(abs(shortest_difference(vega.azimuth_deg, expected.azimuth_deg)), .02)
        for name, days, minimum in (("Sun", 30, 15), ("Moon", 1, 5), ("Mercury", 30, 1)):
            with self.subTest(name=name):
                body = getattr(ephem, name)(UTC.replace(tzinfo=None))
                star = EquatorialTarget(math.degrees(float(body.a_ra))/15, math.degrees(float(body.a_dec)))
                later = UTC + timedelta(days=days)
                moving = converter.altaz(TARGETS[name], later)
                fixed = AstropyConverter(OBSERVER).altaz(star, later)
                self.assertGreater(max(abs(moving.altitude_deg-fixed.altitude_deg),
                                       abs(shortest_difference(moving.azimuth_deg, fixed.azimuth_deg))), minimum)

    def test_horizon_and_observer_location(self):
        northern = NamedTargetConverter(OBSERVER)
        southern = NamedTargetConverter(Observer(-44, -86, 200))
        self.assertFalse(northern.altaz(TARGETS["Polaris"], UTC).below_horizon)
        self.assertTrue(southern.altaz(TARGETS["Polaris"], UTC).below_horizon)
        self.assertFalse(northern.altaz(TARGETS["Sun"], UTC).below_horizon)
        self.assertTrue(northern.altaz(TARGETS["Sun"], UTC + timedelta(hours=12)).below_horizon)
        self.assertAlmostEqual(northern._observer.elevation, OBSERVER.elevation_m)
        self.assertEqual(northern._observer.pressure, 0)

    def test_worker_recomputes_named_body_each_update_and_uses_existing_reference(self):
        reference = HeadingReference(magnetic_declination_deg=5, north_reference="magnetic")
        times = iter((UTC, UTC + timedelta(minutes=10)))
        worker = CoordinateWorker(OBSERVER, reference, utc_clock=lambda: next(times))
        try:
            results = []
            for now in (0, 1):
                worker.submit(7, TARGETS["Sun"], now)
                result = worker._results.get(timeout=5)
                self.assertFalse(result.error)
                self.assertEqual(result.target, TARGETS["Sun"])
                self.assertAlmostEqual(result.mount.heading_deg, (result.horizontal.azimuth_deg-5) % 360)
                results.append(result)
            self.assertNotEqual(results[0].horizontal, results[1].horizontal)
            self.assertGreater(abs(shortest_difference(results[1].mount.heading_deg, results[0].mount.heading_deg)), 1)
        finally:
            worker.close()

    def test_preview_failure_is_only_a_display_result(self):
        preview = TargetPreview(OBSERVER, converter_factory=mock.Mock(side_effect=RuntimeError("ephemeris unavailable")))
        preview.poll(0)
        snapshot = preview._results.get(timeout=5)
        self.assertEqual(set(snapshot.errors), set(TARGETS))
        self.assertFalse(snapshot.positions)


class TargetSelectorTests(unittest.TestCase):
    def test_tiles_select_without_motion_keep_manual_fields_and_reject_below_horizon(self):
        import pygame
        pygame.init()
        try:
            helper = dashboard_fixture.OperatorDashboardTests()
            screen, dashboard, session, auto = helper.make_dashboard()
            dashboard.ra.value, dashboard.dec.value = "12:47:52", "-05:07:52"
            dashboard.show_targets = True
            dashboard.target_snapshot = TargetSnapshot(2, {name: HorizontalTarget(100, -5 if name == "Sun" else 30, UTC)
                                                           for name in TARGETS}, {})
            for size in ((820, 620), (1100, 760)):
                screen = pygame.display.set_mode(size)
                helper.draw(screen, dashboard, session, auto)
                for rect in dashboard.buttons.values():
                    self.assertTrue(screen.get_rect().contains(rect))
            click = lambda name: pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=dashboard.buttons[name].center)
            self.assertEqual(dashboard.handle_events([click("target:Sun")]), [])
            with self.assertRaisesRegex(ValueError, "below the horizon"):
                dashboard.selected_target(2)
            self.assertEqual(dashboard.handle_events([click("target:Vega")]), [])
            self.assertEqual(dashboard.selected_target(2), TARGETS["Vega"])
            self.assertEqual(dashboard.handle_events([click("track_selected")]), ["track_selected"])
            self.assertEqual(dashboard.celestial_command(), "TRACK_RADEC 12:47:52 -05:07:52")
            self.assertEqual(auto.phase, "READY")
            self.assertEqual(dashboard.handle_events([click("stop")]), ["stop"])
            with self.assertRaisesRegex(ValueError, "current target position"):
                dashboard.selected_target(40)
        finally:
            pygame.quit()

    def test_selected_sun_uses_one_existing_goto_then_updates_without_encoder_gate(self):
        import pygame
        class Preview:
            def __init__(self, observer):
                self.observer = observer
            def poll(self, now):
                return TargetSnapshot(now, {name: HorizontalTarget(100, 30, UTC) for name in TARGETS}, {})
        click = lambda x, y: pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(x, y))
        with mock.patch("celestial_targets.TargetPreview", Preview):
            result, writes, _, _, auto, rows, log = ui_fixture.CelestialUiTests().run_ui(
                [(.8, [click(1000, 249)]), (1.0, [click(100, 312)]), (1.4, [click(1000, 520)])],
                until=4.5, real_session=True, gui_entry=True)
        self.assertEqual(result, 0)
        self.assertEqual(sum(data.startswith(b"CELESTIAL_GOTO ") for _, data in writes), 1)
        self.assertGreater(sum(data.startswith(b"CELESTIAL_UPDATE ") for _, data in writes), 1)
        self.assertEqual(auto.phase, "CELESTIAL_TRACK")
        self.assertIn("CELESTIAL_REQUEST Sun", log)
        self.assertTrue(any("Sun | Alt=" in row for row in rows))

    def test_below_horizon_selection_sends_no_celestial_motion_or_extra_stop(self):
        import pygame
        class Preview:
            def __init__(self, observer):
                pass
            def poll(self, now):
                return TargetSnapshot(now, {name: HorizontalTarget(100, -10, UTC) for name in TARGETS}, {})
        click = lambda x, y: pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(x, y))
        with mock.patch("celestial_targets.TargetPreview", Preview):
            helper = ui_fixture.CelestialUiTests()
            baseline = helper.run_ui([], until=2.5, real_session=True, gui_entry=True)
            result, writes, _, _, _, _, log = helper.run_ui(
                [(.8, [click(1000, 249)]), (1.0, [click(100, 312)]), (1.4, [click(1000, 520)])],
                until=2.5, real_session=True, gui_entry=True)
        self.assertEqual(result, 0)
        self.assertEqual(writes, baseline[1])
        self.assertIn("Sun is below the horizon", log)

    def test_preview_failure_and_browsing_during_track_do_not_stop_or_retarget(self):
        import pygame
        class Preview:
            def __init__(self, observer):
                pass
            def poll(self, now):
                if now > 2:
                    return TargetSnapshot(now, {}, {name: "Preview unavailable" for name in TARGETS})
                return TargetSnapshot(now, {name: HorizontalTarget(100, 30, UTC) for name in TARGETS}, {})
        click = lambda x, y: pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(x, y))
        actions = [(.8, [click(1000, 249)]), (1.0, [click(100, 312)]), (1.4, [click(1000, 520)])]
        with mock.patch("celestial_targets.TargetPreview", Preview):
            helper = ui_fixture.CelestialUiTests()
            baseline = helper.run_ui(actions, until=4.5, real_session=True, gui_entry=True)
            result, writes, _, _, auto, _, _ = helper.run_ui(
                actions + [(2.5, [click(1000, 249)]), (2.7, [click(330, 460)])],
                until=4.5, real_session=True, gui_entry=True)
        self.assertEqual(result, 0)
        self.assertEqual(writes, baseline[1])
        self.assertEqual(auto.phase, "CELESTIAL_TRACK")
        self.assertEqual(auto.celestial_source, "Sun")


if __name__ == "__main__":
    unittest.main()
