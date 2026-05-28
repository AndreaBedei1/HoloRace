import math
import unittest

from underwater_racing.control.simple_gate_follower import RoverCommand, SimpleGateFollower
from underwater_racing.holoocean.vehicle_loader import BlueROVThrusterAdapter
from underwater_racing.racing.beacon import BeaconMeasurement


def measurement(distance_m=0.2, bearing_error_deg=60.0, vertical_error_m=0.0):
    return BeaconMeasurement(
        active_gate_id=1,
        beacon_id=1,
        beacon_position=[0.0, 0.0, -3.0],
        target_position=[0.0, 0.0, -5.0],
        distance_m=distance_m,
        direction_world=[1.0, 0.0, 0.0],
        bearing_error_rad=math.radians(bearing_error_deg),
        vertical_error_m=vertical_error_m,
        elevation_error_rad=0.0,
    )


class ControlAndVehicleTests(unittest.TestCase):
    def test_yaw_sign_inverts_thruster_contribution(self):
        command = RoverCommand(yaw=0.5)
        normal = BlueROVThrusterAdapter(yaw_sign=1.0).to_action(command)
        inverted = BlueROVThrusterAdapter(yaw_sign=-1.0).to_action(command)

        self.assertEqual(normal[:4], [0.0, 0.0, 0.0, 0.0])
        self.assertEqual(inverted[:4], [0.0, 0.0, 0.0, 0.0])
        for normal_value, inverted_value in zip(normal[4:], inverted[4:]):
            self.assertAlmostEqual(normal_value, -inverted_value)

    def test_minimum_turning_surge_for_moderate_bearing_error(self):
        follower = SimpleGateFollower(min_turning_surge=0.15)
        command = follower.compute_command(measurement(distance_m=0.5, bearing_error_deg=60.0))

        self.assertAlmostEqual(command.surge, 0.15)
        self.assertGreater(command.yaw, 0.0)

    def test_no_minimum_turning_surge_when_target_is_behind(self):
        follower = SimpleGateFollower(min_turning_surge=0.15)
        command = follower.compute_command(measurement(distance_m=0.5, bearing_error_deg=120.0))

        self.assertAlmostEqual(command.surge, 0.0)

    def test_yaw_command_is_rate_limited_and_less_aggressive(self):
        follower = SimpleGateFollower()
        command = follower.compute_command(measurement(distance_m=2.0, bearing_error_deg=90.0))

        self.assertAlmostEqual(command.yaw, follower.max_yaw_delta_per_step)
        self.assertLess(command.yaw, 0.35)

    def test_yaw_is_suppressed_near_target(self):
        follower = SimpleGateFollower()
        command = follower.compute_command(measurement(distance_m=0.04, bearing_error_deg=170.0))

        self.assertAlmostEqual(command.yaw, 0.0)

    def test_no_spin_when_distance_is_near_zero(self):
        follower = SimpleGateFollower()
        follower.compute_command(measurement(distance_m=2.0, bearing_error_deg=90.0))
        command = follower.compute_command(measurement(distance_m=0.01, bearing_error_deg=-170.0))

        self.assertAlmostEqual(command.yaw, 0.0)


if __name__ == "__main__":
    unittest.main()
