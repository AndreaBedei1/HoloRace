"""Default vehicle and demo runtime configuration."""

ROVER_NAME = "rov0"
ROVER_AGENT_TYPE = "BlueROV2"
ROVER_CONTROL_SCHEME = 0

# Start in front of a yaw=0 gate. The gate plane normal points along +X.
ROVER_START_LOCATION = [-5.0, 0.0, -5.0]
ROVER_START_ROTATION = [0.0, 0.0, 0.0]

DEFAULT_WORLD = "OpenWater"
WORLD_FALLBACKS = ["OpenWater", "SimpleUnderwater", "PierHarbor", "Dam"]

TICKS_PER_SEC = 30
MAX_DEMO_DURATION_S = 45.0

ROVER_CONFIG_SOURCE = (
    "HolooceanLibrary/src/lib/rover.py BlueROV2 schema, adapted to a minimal "
    "Pose/Velocity/IMU/Depth/Collision sensor set"
)
