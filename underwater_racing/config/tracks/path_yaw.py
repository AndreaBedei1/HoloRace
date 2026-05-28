"""Helpers for path-consistent gate yaw configuration."""

from __future__ import annotations

import math
from typing import Sequence


def compute_gate_yaw_from_path(
    previous_center: Sequence[float] | None,
    current_center: Sequence[float],
    next_center: Sequence[float] | None = None,
) -> float:
    if previous_center is None and next_center is None:
        raise ValueError("At least one neighboring center is required")

    if previous_center is None:
        dx = float(next_center[0]) - float(current_center[0])
        dy = float(next_center[1]) - float(current_center[1])
    elif next_center is None:
        dx = float(current_center[0]) - float(previous_center[0])
        dy = float(current_center[1]) - float(previous_center[1])
    else:
        dx = float(next_center[0]) - float(previous_center[0])
        dy = float(next_center[1]) - float(previous_center[1])

    return math.degrees(math.atan2(dy, dx))


def yaws_for_path(centers: list[list[float]]) -> list[float]:
    yaws: list[float] = []
    for index, center in enumerate(centers):
        previous_center = centers[index - 1] if index > 0 else None
        next_center = centers[index + 1] if index + 1 < len(centers) else None
        yaws.append(compute_gate_yaw_from_path(previous_center, center, next_center))
    return yaws
