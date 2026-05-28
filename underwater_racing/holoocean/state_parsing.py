"""Parse HoloOcean state dictionaries without depending on NumPy."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List


@dataclass(frozen=True)
class VehiclePose:
    position: List[float]
    yaw_deg: float


def get_sensor_value(state: Dict[str, Any], sensor_key: str, agent_name: str | None = None) -> Any:
    if agent_name and agent_name in state and isinstance(state[agent_name], dict):
        return state[agent_name].get(sensor_key)
    return state.get(sensor_key)


def _matrix_value(matrix: Any, row: int, col: int) -> float:
    try:
        return float(matrix[row][col])
    except TypeError:
        return float(matrix[row, col])


def parse_pose_matrix(pose_sensor_value: Any) -> VehiclePose | None:
    if pose_sensor_value is None:
        return None

    x = _matrix_value(pose_sensor_value, 0, 3)
    y = _matrix_value(pose_sensor_value, 1, 3)
    z = _matrix_value(pose_sensor_value, 2, 3)
    yaw = math.degrees(
        math.atan2(
            _matrix_value(pose_sensor_value, 1, 0),
            _matrix_value(pose_sensor_value, 0, 0),
        )
    )
    return VehiclePose(position=[x, y, z], yaw_deg=yaw)


def parse_vehicle_pose(
    state: Dict[str, Any],
    agent_name: str | None = None,
    sensor_key: str = "PoseSensor",
) -> VehiclePose | None:
    return parse_pose_matrix(get_sensor_value(state, sensor_key, agent_name))


def has_collision(state: Dict[str, Any], agent_name: str | None = None) -> bool:
    value = get_sensor_value(state, "CollisionSensor", agent_name)
    if value is None:
        return False
    if hasattr(value, "any"):
        return bool(value.any())
    if isinstance(value, (list, tuple)):
        return any(bool(item) for item in value)
    return bool(value)
