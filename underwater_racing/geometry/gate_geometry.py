"""Gate geometry, procedural box layout, and plane crossing checks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

from underwater_racing.geometry.transforms import (
    add,
    dot,
    interpolate,
    rotate_yaw,
    subtract,
    yaw_axes,
)


@dataclass(frozen=True)
class BoxProp:
    """A HoloOcean box primitive in world coordinates."""

    name: str
    location: List[float]
    rotation: List[float]
    scale: List[float]
    tag: str


@dataclass(frozen=True)
class CrossingResult:
    gate_id: int
    crossed: bool
    inside_opening: bool
    lateral_error_m: float
    vertical_error_m: float
    plane_crossed: bool = False
    direction_ok: bool = False
    crossing_point: Optional[List[float]] = None


def gate_bar_boxes(gate: "RaceGate", visual_yaw_deg: float | None = None) -> List[BoxProp]:
    """Return the four boxes that form a square gate frame."""
    yaw_deg = gate.yaw_deg if visual_yaw_deg is None else visual_yaw_deg
    outer = gate.outer_size_m
    half_opening = gate.opening_size_m / 2.0
    half_thickness = gate.frame_thickness_m / 2.0
    side_offset = half_opening + half_thickness

    specs = [
        (
            "top",
            [0.0, 0.0, side_offset],
            [gate.frame_depth_m, outer, gate.frame_thickness_m],
        ),
        (
            "bottom",
            [0.0, 0.0, -side_offset],
            [gate.frame_depth_m, outer, gate.frame_thickness_m],
        ),
        (
            "left",
            [0.0, -side_offset, 0.0],
            [gate.frame_depth_m, gate.frame_thickness_m, outer],
        ),
        (
            "right",
            [0.0, side_offset, 0.0],
            [gate.frame_depth_m, gate.frame_thickness_m, outer],
        ),
    ]

    boxes: List[BoxProp] = []
    for name, local_offset, scale in specs:
        location = add(gate.center, rotate_yaw(local_offset, yaw_deg))
        boxes.append(
            BoxProp(
                name=name,
                location=location,
                rotation=[0.0, 0.0, yaw_deg],
                scale=scale,
                tag=f"gate_{gate.id}_{name}",
            )
        )
    return boxes


def signed_distance_to_gate_plane(gate: "RaceGate", position: Sequence[float]) -> float:
    normal, _, _ = yaw_axes(gate.yaw_deg)
    return dot(subtract(position, gate.center), normal)


def project_to_gate_opening(gate: "RaceGate", position: Sequence[float]) -> tuple[float, float]:
    """Project a world point onto gate local lateral and vertical axes."""
    _, right, up = yaw_axes(gate.yaw_deg)
    rel = subtract(position, gate.center)
    return dot(rel, right), dot(rel, up)


def is_inside_opening(gate: "RaceGate", position: Sequence[float]) -> bool:
    lateral, vertical = project_to_gate_opening(gate, position)
    half = gate.opening_size_m / 2.0
    return abs(lateral) <= half and abs(vertical) <= half


def detect_gate_crossing(
    gate: "RaceGate",
    previous_position: Sequence[float],
    current_position: Sequence[float],
) -> CrossingResult:
    """Detect a correct-direction gate-plane crossing between two positions."""
    prev_dist = signed_distance_to_gate_plane(gate, previous_position)
    curr_dist = signed_distance_to_gate_plane(gate, current_position)
    forward_crossing = prev_dist < 0.0 <= curr_dist
    backward_crossing = prev_dist > 0.0 >= curr_dist
    plane_crossed = forward_crossing or backward_crossing
    direction_ok = forward_crossing

    if not plane_crossed:
        return CrossingResult(
            gate_id=gate.id,
            crossed=False,
            inside_opening=False,
            lateral_error_m=0.0,
            vertical_error_m=0.0,
            plane_crossed=False,
            direction_ok=curr_dist >= prev_dist,
            crossing_point=None,
        )

    denom = curr_dist - prev_dist
    t = 0.0 if denom == 0.0 else -prev_dist / denom
    crossing_point = interpolate(previous_position, current_position, t)
    lateral, vertical = project_to_gate_opening(gate, crossing_point)
    half = gate.opening_size_m / 2.0
    inside = abs(lateral) <= half and abs(vertical) <= half

    return CrossingResult(
        gate_id=gate.id,
        crossed=direction_ok,
        inside_opening=inside,
        lateral_error_m=lateral,
        vertical_error_m=vertical,
        plane_crossed=plane_crossed,
        direction_ok=direction_ok,
        crossing_point=crossing_point,
    )


class CrossingDetector:
    """Stateful detector for one active gate."""

    def __init__(self, gate: "RaceGate", previous_position: Sequence[float] | None = None):
        self.gate = gate
        self.previous_position = list(previous_position) if previous_position is not None else None

    def reset(self, gate: "RaceGate", previous_position: Sequence[float] | None = None) -> None:
        self.gate = gate
        self.previous_position = list(previous_position) if previous_position is not None else None

    def update(self, current_position: Sequence[float]) -> CrossingResult:
        if self.previous_position is None:
            self.previous_position = list(current_position)
            return CrossingResult(
                gate_id=self.gate.id,
                crossed=False,
                inside_opening=False,
                lateral_error_m=0.0,
                vertical_error_m=0.0,
            )

        result = detect_gate_crossing(self.gate, self.previous_position, current_position)
        self.previous_position = list(current_position)
        return result


from underwater_racing.racing.gate import RaceGate  # noqa: E402
