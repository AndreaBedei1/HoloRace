import unittest

from underwater_racing.config.track_registry import available_tracks, build_track
from underwater_racing.config.tracks.path_yaw import yaws_for_path


class TrackConfigTests(unittest.TestCase):
    def assert_unique_gate_and_beacon_ids(self, track_name):
        track = build_track(track_name)
        gate_ids = [gate.id for gate in track.gates]
        beacon_ids = [gate.beacon_id for gate in track.gates]

        self.assertEqual(len(gate_ids), len(set(gate_ids)))
        self.assertEqual(len(beacon_ids), len(set(beacon_ids)))
        self.assertEqual(gate_ids, list(range(1, len(track.gates) + 1)))
        self.assertEqual(beacon_ids, gate_ids)

    def test_registry_exposes_expected_tracks(self):
        self.assertEqual(available_tracks(), ["single", "straight", "zigzag", "zigzag_smooth"])

    def test_straight_track_has_unique_gate_and_beacon_ids(self):
        self.assert_unique_gate_and_beacon_ids("straight")

    def test_zigzag_track_has_unique_gate_and_beacon_ids(self):
        self.assert_unique_gate_and_beacon_ids("zigzag")

    def test_zigzag_smooth_track_has_unique_gate_and_beacon_ids(self):
        self.assert_unique_gate_and_beacon_ids("zigzag_smooth")

    def test_all_tracks_keep_standard_opening_size(self):
        for track_name in available_tracks():
            with self.subTest(track=track_name):
                track = build_track(track_name)
                self.assertTrue(track.gates)
                for gate in track.gates:
                    self.assertEqual(gate.opening_size_m, 1.5)

    def test_straight_track_layout(self):
        track = build_track("straight")
        self.assertEqual([gate.center for gate in track.gates], [
            [0.0, 0.0, -5.0],
            [6.0, 0.0, -5.0],
            [12.0, 0.0, -5.0],
            [18.0, 0.0, -5.0],
        ])
        self.assertEqual([gate.yaw_deg for gate in track.gates], [0.0, 0.0, 0.0, 0.0])

    def test_zigzag_track_layout(self):
        track = build_track("zigzag")
        self.assertEqual([gate.center for gate in track.gates], [
            [0.0, 0.0, -5.0],
            [6.0, 2.0, -5.0],
            [12.0, -2.0, -5.0],
            [18.0, 2.0, -5.0],
            [24.0, 0.0, -5.0],
        ])
        self.assertEqual(
            [gate.yaw_deg for gate in track.gates],
            yaws_for_path([gate.center for gate in track.gates]),
        )

    def test_zigzag_smooth_track_layout(self):
        track = build_track("zigzag_smooth")
        self.assertEqual([gate.center for gate in track.gates], [
            [0.0, 0.0, -5.0],
            [8.0, 1.5, -5.0],
            [16.0, -1.5, -5.0],
            [24.0, 1.5, -5.0],
            [32.0, 0.0, -5.0],
        ])
        self.assertEqual(
            [gate.yaw_deg for gate in track.gates],
            yaws_for_path([gate.center for gate in track.gates]),
        )


if __name__ == "__main__":
    unittest.main()
