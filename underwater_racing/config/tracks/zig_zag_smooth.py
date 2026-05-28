"""Wider zigzag track for controller tuning."""

from underwater_racing.config.tracks.path_yaw import yaws_for_path
from underwater_racing.racing.gate import RaceGate
from underwater_racing.racing.track import RaceTrack


def build_track() -> RaceTrack:
    centers = [
        [0.0, 0.0, -5.0],
        [8.0, 1.5, -5.0],
        [16.0, -1.5, -5.0],
        [24.0, 1.5, -5.0],
        [32.0, 0.0, -5.0],
    ]
    return RaceTrack.from_gates(
        [
            RaceGate(id=index + 1, center=center, yaw_deg=yaw, beacon_id=index + 1)
            for index, (center, yaw) in enumerate(zip(centers, yaws_for_path(centers)))
        ]
    )
