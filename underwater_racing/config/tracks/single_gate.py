"""Single-gate track definition."""

from underwater_racing.racing.gate import RaceGate
from underwater_racing.racing.track import RaceTrack


def build_track() -> RaceTrack:
    return RaceTrack.from_gates(
        [
            RaceGate(id=1, center=[0.0, 0.0, -5.0], yaw_deg=0.0, beacon_id=1),
        ]
    )

