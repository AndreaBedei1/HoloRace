"""BlueROV-compatible vehicle configuration and command adapter."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List

from underwater_racing.config.default_vehicle import (
    ROVER_AGENT_TYPE,
    ROVER_CONTROL_SCHEME,
    ROVER_NAME,
    ROVER_START_LOCATION,
    ROVER_START_ROTATION,
)
from underwater_racing.control.simple_gate_follower import RoverCommand


def _sensor(sensor_type: str, socket: str, hz: int, sensor_name: str | None = None) -> Dict[str, Any]:
    cfg: Dict[str, Any] = {
        "sensor_type": sensor_type,
        "socket": socket,
        "Hz": hz,
    }
    if sensor_name:
        cfg["sensor_name"] = sensor_name
    return cfg


def minimal_bluerov_sensors(hz: int = 30) -> List[Dict[str, Any]]:
    """Return only the sensors needed by the racing demo."""
    return [
        _sensor("PoseSensor", "PoseSocket", hz),
        _sensor("VelocitySensor", "VelocitySocket", hz),
        _sensor("IMUSensor", "IMUSocket", hz),
        _sensor("DepthSensor", "DepthSocket", hz),
        _sensor("CollisionSensor", "CollisionSocket", hz),
    ]


def build_bluerov_config(
    name: str = ROVER_NAME,
    location: Iterable[float] = ROVER_START_LOCATION,
    rotation: Iterable[float] = ROVER_START_ROTATION,
    control_scheme: int = ROVER_CONTROL_SCHEME,
    sensors: List[Dict[str, Any]] | None = None,
) -> Dict[str, Any]:
    """Build a minimal config matching the existing HolooceanLibrary BlueROV2 schema."""
    return {
        "agent_name": name,
        "agent_type": ROVER_AGENT_TYPE,
        "control_scheme": control_scheme,
        "location": [float(v) for v in location],
        "rotation": [float(v) for v in rotation],
        "sensors": sensors if sensors is not None else minimal_bluerov_sensors(),
    }


@dataclass
class BlueROVThrusterAdapter:
    """Convert abstract surge/sway/heave/yaw commands to the 8-thruster vector."""

    max_thrust: float = 12.0
    surge_thrust: float = 12.0
    sway_thrust: float = 8.0
    heave_thrust: float = 8.0
    yaw_thrust: float = 3.0

    def to_action(self, command: RoverCommand) -> List[float]:
        cmd = [0.0] * 8

        for index in range(4):
            cmd[index] += command.heave * self.heave_thrust

        for index in range(4, 8):
            cmd[index] += command.surge * self.surge_thrust

        sway = command.sway * self.sway_thrust
        cmd[4] += sway
        cmd[5] -= sway
        cmd[6] += sway
        cmd[7] -= sway

        yaw = command.yaw * self.yaw_thrust
        cmd[4] -= yaw
        cmd[5] += yaw
        cmd[6] += yaw
        cmd[7] -= yaw

        return [max(-self.max_thrust, min(self.max_thrust, value)) for value in cmd]

    def zero_action(self) -> List[float]:
        return [0.0] * 8
