import math
import unittest

from underwater_racing.geometry.gate_geometry import gate_bar_boxes
from underwater_racing.geometry.transforms import norm, subtract
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
        self.assertEqual(gate.center, [0.0, 0.0, -5.0])
        self.assertEqual(gate.target_position, [0.0, 0.0, -5.25])
        self.assertEqual(gate.beacon_position, [0.0, 0.0, -3.0])

    def test_custom_navigation_target_offset(self):
        gate = RaceGate(
            id=1,
            center=[0.0, 0.0, -5.0],
            yaw_deg=0.0,
            navigation_target_z_offset_m=-0.1,
        )

        self.assertEqual(gate.target_position, [0.0, 0.0, -5.1])

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

    def test_rotated_gate_boxes_stay_compact_around_center(self):
        gate = RaceGate(id=2, center=[6.0, 2.0, -5.0], yaw_deg=20.0)
        boxes = gate_bar_boxes(gate)
        side_offset = gate.opening_size_m / 2.0 + gate.frame_thickness_m / 2.0

        self.assertEqual(len(boxes), 4)
        self.assertEqual(len({box.tag for box in boxes}), 4)
        for box in boxes:
            distance = norm(subtract(box.location, gate.center))
            self.assertLessEqual(distance, 1.5)
            self.assertAlmostEqual(distance, side_offset)

        by_name = {box.name: box for box in boxes}
        self.assertAlmostEqual(by_name["top"].location[2], gate.center[2] + side_offset)
        self.assertAlmostEqual(by_name["bottom"].location[2], gate.center[2] - side_offset)
        self.assertAlmostEqual(by_name["left"].location[2], gate.center[2])
        self.assertAlmostEqual(by_name["right"].location[2], gate.center[2])
        self.assertAlmostEqual(
            by_name["left"].location[0],
            gate.center[0] + side_offset * math.sin(math.radians(gate.yaw_deg)),
        )
        self.assertAlmostEqual(
            by_name["right"].location[0],
            gate.center[0] - side_offset * math.sin(math.radians(gate.yaw_deg)),
        )

    def test_axis_aligned_visual_gate_boxes_ignore_referee_yaw(self):
        gate = RaceGate(id=2, center=[6.0, 2.0, -5.0], yaw_deg=20.0)
        boxes = {box.name: box for box in gate_bar_boxes(gate, visual_yaw_deg=0.0)}

        self.assertEqual(boxes["left"].location, [6.0, 1.125, -5.0])
        self.assertEqual(boxes["right"].location, [6.0, 2.875, -5.0])
        self.assertEqual(boxes["top"].rotation, [0.0, 0.0, 0.0])

    def test_rejects_non_standard_opening_size(self):
        with self.assertRaises(ValueError):
            RaceGate(id=1, center=[0.0, 0.0, 0.0], yaw_deg=0.0, opening_size_m=2.0)


if __name__ == "__main__":
    unittest.main()
