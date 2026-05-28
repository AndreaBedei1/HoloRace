"""Gate approach and exit corridor targets."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

from underwater_racing.geometry.transforms import add, scale, yaw_axes
from underwater_racing.racing.gate import RaceGate


@dataclass(frozen=True)
class GateCorridor:
    gate_id: int
    approach_point: List[float]
    center_point: List[float]
    exit_point: List[float]
    crossing_yaw_deg: float


def build_gate_corridor(
    gate: RaceGate,
    approach_distance_m: float = 2.0,
    exit_distance_m: float = 2.0,
) -> GateCorridor:
    if approach_distance_m <= 0.0:
        raise ValueError("approach_distance_m must be positive")
    if exit_distance_m <= 0.0:
        raise ValueError("exit_distance_m must be positive")

    normal, _, _ = yaw_axes(gate.yaw_deg)
    center = gate.target_position
    return GateCorridor(
        gate_id=gate.id,
        approach_point=add(center, scale(normal, -approach_distance_m)),
        center_point=center,
        exit_point=add(center, scale(normal, exit_distance_m)),
        crossing_yaw_deg=gate.yaw_deg,
    )
