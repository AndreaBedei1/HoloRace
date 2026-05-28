import inspect
import math
import unittest

from underwater_racing.config.track_registry import build_track
from underwater_racing.racing import onboard_corridor_navigator as navigator_module
from underwater_racing.racing.beacon import BeaconMeasurement, VirtualGuidanceProvider
from underwater_racing.racing.onboard_corridor_navigator import (
    APPROACH,
    TRANSIT,
    OnboardCorridorNavigator,
)


def measurement(gate_id=1, target_name="approach", distance_m=0.5):
    return BeaconMeasurement(
        active_gate_id=gate_id,
        beacon_id=gate_id,
        beacon_position=[0.0, 0.0, -3.0],
        target_position=[0.0, 0.0, -5.25],
        distance_m=distance_m,
        direction_world=[1.0, 0.0, 0.0],
        bearing_error_rad=0.0,
        vertical_error_m=0.0,
        elevation_error_rad=0.0,
        target_name=target_name,
    )


class OnboardCorridorNavigatorTests(unittest.TestCase):
    def test_phase_change_and_switch_use_only_guidance_measurements(self):
        navigator = OnboardCorridorNavigator(build_track("straight"))

        self.assertEqual(navigator.phase, APPROACH)
        update = navigator.update_from_measurement(measurement(distance_m=0.5))

        self.assertTrue(update.phase_changed)
        self.assertFalse(update.switched)
        self.assertEqual(update.to_phase, TRANSIT)
        self.assertEqual(navigator.phase, TRANSIT)
        self.assertEqual(navigator.active_gate_id, 1)

        switch = navigator.update_from_measurement(measurement(target_name="exit", distance_m=0.5))

        self.assertTrue(switch.switched)
        self.assertEqual(switch.completed_gate_id, 1)
        self.assertEqual(switch.to_gate_id, 2)
        self.assertEqual(navigator.phase, APPROACH)

    def test_active_target_moves_from_approach_to_exit(self):
        navigator = OnboardCorridorNavigator(build_track("straight"))
        provider = VirtualGuidanceProvider()
        target = navigator.active_target

        self.assertEqual(target.target_name, "approach")
        first_measurement = provider.get_measurement(
            vehicle_position=target.position,
            vehicle_yaw_deg=0.0,
            target=target,
        )
        navigator.update_from_measurement(first_measurement)

        self.assertEqual(navigator.active_target.target_name, "exit")

    def test_navigator_module_has_no_referee_or_pose_dependencies(self):
        source = inspect.getsource(navigator_module)

        self.assertNotIn("CrossingResult", source)
        self.assertNotIn("CrossingDetector", source)
        self.assertNotIn("RaceState", source)
        self.assertNotIn("PoseSensor", source)
        self.assertNotIn("parse_vehicle_pose", source)


if __name__ == "__main__":
    unittest.main()

