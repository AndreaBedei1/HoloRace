import unittest

from underwater_racing.geometry.gate_geometry import gate_bar_boxes
from underwater_racing.racing.gate import RaceGate


class GateGeometryTests(unittest.TestCase):
    def test_gate_opening_and_beacon_position(self):
        gate = RaceGate(
            id=1,
            center=[0.0, 0.0, -5.0],
            yaw_deg=0.0,
            opening_size_m=1.5,
            frame_thickness_m=0.25,
            frame_depth_m=0.30,
            beacon_clearance_m=1.0,
        )

        self.assertEqual(gate.opening_size_m, 1.5)
        self.assertAlmostEqual(gate.outer_size_m, 2.0)
        self.assertEqual(gate.target_position, [0.0, 0.0, -5.0])
        self.assertEqual(gate.beacon_position, [0.0, 0.0, -3.0])

    def test_gate_is_four_boxes_around_opening(self):
        gate = RaceGate(
            id=1,
            center=[0.0, 0.0, 0.0],
            yaw_deg=0.0,
            opening_size_m=1.5,
            frame_thickness_m=0.25,
            frame_depth_m=0.30,
        )

        boxes = {box.name: box for box in gate_bar_boxes(gate)}
        self.assertEqual(set(boxes), {"top", "bottom", "left", "right"})
        self.assertEqual(boxes["top"].scale, [0.30, 2.0, 0.25])
        self.assertEqual(boxes["left"].scale, [0.30, 0.25, 2.0])
        self.assertEqual(boxes["top"].location, [0.0, 0.0, 0.875])
        self.assertEqual(boxes["left"].location, [0.0, -0.875, 0.0])

    def test_rejects_non_standard_opening_size(self):
        with self.assertRaises(ValueError):
            RaceGate(id=1, center=[0.0, 0.0, 0.0], yaw_deg=0.0, opening_size_m=2.0)


if __name__ == "__main__":
    unittest.main()
