"""HoloOcean primitive prop spawning for gates and beacon markers."""

from __future__ import annotations

from typing import Any, List

from underwater_racing.geometry.gate_geometry import BoxProp, gate_bar_boxes
from underwater_racing.racing.gate import RaceGate


def spawn_box_prop(env: Any, box: BoxProp, material: str = "steel") -> None:
    env.spawn_prop(
        "box",
        location=box.location,
        rotation=box.rotation,
        scale=box.scale,
        sim_physics=False,
        material=material,
        tag=box.tag,
    )


def spawn_gate(env: Any, gate: RaceGate, material: str = "steel") -> List[BoxProp]:
    boxes = gate_bar_boxes(gate)
    for box in boxes:
        spawn_box_prop(env, box, material=material)
    return boxes


def spawn_beacon_marker(env: Any, gate: RaceGate, material: str = "gold") -> None:
    env.spawn_prop(
        "sphere",
        location=gate.beacon_position,
        rotation=[0.0, 0.0, 0.0],
        scale=[0.25, 0.25, 0.25],
        sim_physics=False,
        material=material,
        tag=f"beacon_{gate.beacon_id}",
    )
