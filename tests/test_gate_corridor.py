import math
import unittest

from underwater_racing.racing.gate import RaceGate
from underwater_racing.racing.gate_corridor import build_gate_corridor


class GateCorridorTests(unittest.TestCase):
    def test_corridor_points_for_yaw_zero(self):
        gate = RaceGate(id=1, center=[0.0, 0.0, -5.0], yaw_deg=0.0)
        corridor = build_gate_corridor(gate)

        self.assertEqual(corridor.gate_id, 1)
        self.assertEqual(corridor.center_point, [0.0, 0.0, -5.25])
        self.assertEqual(corridor.approach_point, [-2.0, 0.0, -5.25])
        self.assertEqual(corridor.exit_point, [2.0, 0.0, -5.25])
        self.assertEqual(corridor.crossing_yaw_deg, 0.0)

    def test_corridor_points_for_rotated_gate(self):
        gate = RaceGate(id=2, center=[6.0, 2.0, -5.0], yaw_deg=20.0)
        corridor = build_gate_corridor(gate)
        dx = 2.0 * math.cos(math.radians(20.0))
        dy = 2.0 * math.sin(math.radians(20.0))

        self.assertAlmostEqual(corridor.center_point[0], 6.0)
        self.assertAlmostEqual(corridor.center_point[1], 2.0)
        self.assertAlmostEqual(corridor.center_point[2], -5.25)
        self.assertAlmostEqual(corridor.approach_point[0], 6.0 - dx)
        self.assertAlmostEqual(corridor.approach_point[1], 2.0 - dy)
        self.assertAlmostEqual(corridor.exit_point[0], 6.0 + dx)
        self.assertAlmostEqual(corridor.exit_point[1], 2.0 + dy)


if __name__ == "__main__":
    unittest.main()

