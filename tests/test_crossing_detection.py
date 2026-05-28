import unittest

from underwater_racing.racing.gate import RaceGate
from underwater_racing.racing.scoring import detect_gate_crossing


class CrossingDetectionTests(unittest.TestCase):
    def setUp(self):
        self.gate = RaceGate(id=1, center=[0.0, 0.0, -5.0], yaw_deg=0.0)

    def test_correct_crossing_inside_opening(self):
        result = detect_gate_crossing(
            self.gate,
            previous_position=[-1.0, 0.2, -5.1],
            current_position=[1.0, 0.2, -5.1],
        )

        self.assertTrue(result.crossed)
        self.assertTrue(result.inside_opening)
        self.assertAlmostEqual(result.lateral_error_m, 0.2)
        self.assertAlmostEqual(result.vertical_error_m, -0.1)

    def test_crossing_outside_opening(self):
        result = detect_gate_crossing(
            self.gate,
            previous_position=[-1.0, 1.0, -5.0],
            current_position=[1.0, 1.0, -5.0],
        )

        self.assertTrue(result.crossed)
        self.assertFalse(result.inside_opening)

    def test_wrong_direction_is_not_a_correct_crossing(self):
        result = detect_gate_crossing(
            self.gate,
            previous_position=[1.0, 0.0, -5.0],
            current_position=[-1.0, 0.0, -5.0],
        )

        self.assertFalse(result.crossed)
        self.assertTrue(result.plane_crossed)
        self.assertFalse(result.direction_ok)


if __name__ == "__main__":
    unittest.main()
