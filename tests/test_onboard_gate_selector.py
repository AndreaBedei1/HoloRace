import inspect
import math
import unittest

from underwater_racing.config.track_registry import build_track
from underwater_racing.geometry.gate_geometry import CrossingResult
from underwater_racing.racing import onboard_gate_selector as selector_module
from underwater_racing.racing.beacon import BeaconMeasurement
from underwater_racing.racing.onboard_gate_selector import OnboardGateSelector
from underwater_racing.racing.race_state import RaceState


def measurement(gate_id=1, distance_m=0.5, bearing_error_deg=0.0, vertical_error_m=0.0):
    return BeaconMeasurement(
        active_gate_id=gate_id,
        beacon_id=gate_id,
        beacon_position=[0.0, 0.0, -3.0],
        target_position=[0.0, 0.0, -5.0],
        distance_m=distance_m,
        direction_world=[1.0, 0.0, 0.0],
        bearing_error_rad=math.radians(bearing_error_deg),
        vertical_error_m=vertical_error_m,
        elevation_error_rad=0.0,
    )


class OnboardGateSelectorTests(unittest.TestCase):
    def test_advances_only_from_beacon_measurement_conditions(self):
        selector = OnboardGateSelector(build_track("straight"), consecutive_ticks_required=2)

        self.assertFalse(selector.update_from_measurement(measurement(gate_id=2)).switched)
        self.assertFalse(selector.update_from_measurement(measurement(distance_m=1.0)).switched)
        self.assertFalse(
            selector.update_from_measurement(measurement(bearing_error_deg=30.0)).switched
        )
        self.assertFalse(
            selector.update_from_measurement(measurement(vertical_error_m=0.7)).switched
        )
        self.assertEqual(selector.active_gate_id, 1)

        self.assertFalse(selector.update_from_measurement(measurement()).switched)
        update = selector.update_from_measurement(measurement())

        self.assertTrue(update.switched)
        self.assertEqual(update.completed_gate_id, 1)
        self.assertEqual(update.to_gate_id, 2)
        self.assertEqual(selector.active_gate_id, 2)
        self.assertEqual(selector.completed_gate_ids, [1])

    def test_close_streak_resets_when_conditions_fail(self):
        selector = OnboardGateSelector(build_track("straight"), consecutive_ticks_required=2)

        self.assertFalse(selector.update_from_measurement(measurement()).switched)
        self.assertFalse(selector.update_from_measurement(measurement(distance_m=1.2)).switched)
        self.assertFalse(selector.update_from_measurement(measurement()).switched)
        self.assertEqual(selector.active_gate_id, 1)

        self.assertTrue(selector.update_from_measurement(measurement()).switched)
        self.assertEqual(selector.active_gate_id, 2)

    def test_optional_range_trend_uses_measurement_history(self):
        selector = OnboardGateSelector(
            build_track("straight"),
            consecutive_ticks_required=10,
            enable_range_trend=True,
            range_trend_ticks_required=2,
            range_trend_min_delta_m=0.01,
        )

        self.assertFalse(selector.update_from_measurement(measurement(distance_m=0.8)).switched)
        self.assertFalse(selector.update_from_measurement(measurement(distance_m=0.95)).switched)
        update = selector.update_from_measurement(measurement(distance_m=1.1))

        self.assertTrue(update.switched)
        self.assertEqual(update.reason, "range_increasing_after_close")
        self.assertEqual(selector.active_gate_id, 2)

    def test_selector_module_has_no_referee_or_pose_dependencies(self):
        source = inspect.getsource(selector_module)

        self.assertNotIn("CrossingResult", source)
        self.assertNotIn("CrossingDetector", source)
        self.assertNotIn("RaceState", source)
        self.assertNotIn("PoseSensor", source)
        self.assertNotIn("parse_vehicle_pose", source)

    def test_referee_state_advances_independently_from_onboard_selector(self):
        track = build_track("straight")
        referee_state = RaceState(track)
        selector = OnboardGateSelector(track, consecutive_ticks_required=1)
        crossing = CrossingResult(
            gate_id=1,
            crossed=True,
            inside_opening=True,
            lateral_error_m=0.0,
            vertical_error_m=0.0,
            plane_crossed=True,
            direction_ok=True,
        )

        self.assertTrue(referee_state.update_from_crossing(crossing))
        self.assertEqual(referee_state.active_gate_id, 2)
        self.assertEqual(selector.active_gate_id, 1)

        self.assertTrue(selector.update_from_measurement(measurement()).switched)
        self.assertEqual(referee_state.active_gate_id, 2)
        self.assertEqual(selector.active_gate_id, 2)


if __name__ == "__main__":
    unittest.main()

