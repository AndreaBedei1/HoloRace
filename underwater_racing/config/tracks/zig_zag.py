"""Zigzag multi-gate track definition."""

from underwater_racing.racing.gate import RaceGate
from underwater_racing.racing.track import RaceTrack


def build_track() -> RaceTrack:
    centers = [
        [0.0, 0.0, -5.0],
        [6.0, 2.0, -5.0],
        [12.0, -2.0, -5.0],
        [18.0, 2.0, -5.0],
        [24.0, 0.0, -5.0],
    ]
    return RaceTrack.from_gates(
        [
            RaceGate(id=index + 1, center=center, yaw_deg=0.0, beacon_id=index + 1)
            for index, center in enumerate(centers)
        ]
    )
