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
    def test_close_threshold_alone_marks_reached_without_switching(self):
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

        self.assertFalse(update.switched)
        self.assertTrue(update.gate_reached)
        self.assertEqual(update.reached_gate_id, 1)
        self.assertEqual(selector.active_gate_id, 1)
        self.assertEqual(selector.completed_gate_ids, [])
        self.assertTrue(selector.gate_reached)

    def test_close_streak_resets_when_conditions_fail(self):
        selector = OnboardGateSelector(build_track("straight"), consecutive_ticks_required=2)

        self.assertFalse(selector.update_from_measurement(measurement()).switched)
        self.assertFalse(selector.update_from_measurement(measurement(distance_m=1.2)).switched)
        self.assertFalse(selector.update_from_measurement(measurement()).switched)
        self.assertEqual(selector.active_gate_id, 1)
        self.assertFalse(selector.gate_reached)

        update = selector.update_from_measurement(measurement())

        self.assertFalse(update.switched)
        self.assertTrue(update.gate_reached)
        self.assertEqual(selector.active_gate_id, 1)

    def test_switches_only_after_range_increases_by_clearance_margin(self):
        selector = OnboardGateSelector(
            build_track("straight"),
            consecutive_ticks_required=1,
            clearance_margin_m=0.4,
        )

        reached = selector.update_from_measurement(measurement(distance_m=0.5))
        self.assertTrue(reached.gate_reached)
        self.assertFalse(reached.switched)
        self.assertEqual(selector.active_gate_id, 1)
        self.assertEqual(selector.min_distance_after_reached_m, 0.5)

        self.assertFalse(selector.update_from_measurement(measurement(distance_m=0.4)).switched)
        self.assertEqual(selector.min_distance_after_reached_m, 0.4)
        self.assertFalse(selector.update_from_measurement(measurement(distance_m=0.8)).switched)
        update = selector.update_from_measurement(measurement(distance_m=0.81))

        self.assertTrue(update.switched)
        self.assertEqual(update.reason, "range_increased_after_reach")
        self.assertEqual(update.completed_gate_id, 1)
        self.assertEqual(update.to_gate_id, 2)
        self.assertEqual(selector.active_gate_id, 2)
        self.assertEqual(selector.completed_gate_ids, [1])

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

        reached = selector.update_from_measurement(measurement(distance_m=0.5))
        self.assertEqual(referee_state.active_gate_id, 2)
        self.assertFalse(reached.switched)
        self.assertTrue(reached.gate_reached)
        self.assertEqual(selector.active_gate_id, 1)

        self.assertTrue(selector.update_from_measurement(measurement(distance_m=1.0)).switched)
        self.assertEqual(referee_state.active_gate_id, 2)
        self.assertEqual(selector.active_gate_id, 2)


if __name__ == "__main__":
    unittest.main()
