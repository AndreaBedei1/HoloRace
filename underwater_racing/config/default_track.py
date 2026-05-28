"""Default single-gate track used by the first demo."""

from underwater_racing.racing.gate import RaceGate
from underwater_racing.racing.track import RaceTrack

DEFAULT_OPENING_SIZE_M = 1.5
DEFAULT_FRAME_THICKNESS_M = 0.25
DEFAULT_FRAME_DEPTH_M = 0.30
DEFAULT_BEACON_CLEARANCE_M = 1.0

FIRST_GATE_CENTER = [0.0, 0.0, -5.0]
FIRST_GATE_YAW_DEG = 0.0


def build_default_track() -> RaceTrack:
    """Return a one-gate track while keeping the ordered track abstraction."""
    gate = RaceGate(
        id=1,
        center=FIRST_GATE_CENTER,
        yaw_deg=FIRST_GATE_YAW_DEG,
        opening_size_m=DEFAULT_OPENING_SIZE_M,
        frame_thickness_m=DEFAULT_FRAME_THICKNESS_M,
        frame_depth_m=DEFAULT_FRAME_DEPTH_M,
        beacon_clearance_m=DEFAULT_BEACON_CLEARANCE_M,
        beacon_id=1,
    )
    return RaceTrack(gates=[gate])
