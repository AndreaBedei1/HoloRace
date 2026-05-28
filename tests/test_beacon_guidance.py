import math
import unittest

from underwater_racing.racing.beacon import VirtualGateBeacon
from underwater_racing.racing.gate import RaceGate


class BeaconGuidanceTests(unittest.TestCase):
    def test_beacon_targets_navigation_point_not_beacon_position(self):
        gate = RaceGate(id=1, center=[0.0, 0.0, -5.0], yaw_deg=0.0)
        beacon = VirtualGateBeacon(gate)

        measurement = beacon.get_measurement(vehicle_position=[-3.0, 0.0, -5.0], vehicle_yaw_deg=0.0)

        self.assertEqual(measurement.active_gate_id, 1)
        self.assertEqual(measurement.beacon_id, 1)
        self.assertEqual(measurement.target_position, [0.0, 0.0, -5.25])
        self.assertNotEqual(measurement.beacon_position, measurement.target_position)
        self.assertAlmostEqual(measurement.distance_m, math.sqrt(3.0**2 + 0.25**2))
        self.assertAlmostEqual(measurement.direction_world[0], 3.0 / measurement.distance_m)
        self.assertAlmostEqual(measurement.direction_world[1], 0.0)
        self.assertAlmostEqual(measurement.direction_world[2], -0.25 / measurement.distance_m)
        self.assertAlmostEqual(measurement.bearing_error_rad, 0.0)
        self.assertAlmostEqual(measurement.vertical_error_m, -0.25)

    def test_bearing_error_uses_vehicle_yaw(self):
        gate = RaceGate(id=1, center=[0.0, 0.0, -5.0], yaw_deg=0.0)
        beacon = VirtualGateBeacon(gate)

        measurement = beacon.get_measurement(vehicle_position=[-3.0, 0.0, -5.0], vehicle_yaw_deg=90.0)

        self.assertAlmostEqual(measurement.bearing_error_rad, -math.pi / 2.0)


if __name__ == "__main__":
    unittest.main()
