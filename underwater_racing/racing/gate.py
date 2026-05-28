"""Race gate model."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

from underwater_racing.geometry.transforms import add, as_vector3


@dataclass(frozen=True)
class RaceGate:
    id: int
    center: List[float]
    yaw_deg: float
    opening_size_m: float = 1.5
    frame_thickness_m: float = 0.25
    frame_depth_m: float = 0.30
    beacon_clearance_m: float = 1.0
    beacon_id: int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "center", as_vector3(self.center))
        if self.id <= 0:
            raise ValueError("Gate id must be positive")
        if self.opening_size_m != 1.5:
            raise ValueError("Race gate internal opening must be exactly 1.5 m")
        if self.frame_thickness_m <= 0.0:
            raise ValueError("Frame thickness must be positive")
        if self.frame_depth_m <= 0.0:
            raise ValueError("Frame depth must be positive")
        if self.beacon_clearance_m < 0.0:
            raise ValueError("Beacon clearance cannot be negative")
        if self.beacon_id is None:
            object.__setattr__(self, "beacon_id", self.id)

    @property
    def outer_size_m(self) -> float:
        return self.opening_size_m + 2.0 * self.frame_thickness_m

    @property
    def target_position(self) -> List[float]:
        return list(self.center)

    @property
    def beacon_position(self) -> List[float]:
        vertical_offset = self.opening_size_m / 2.0 + self.frame_thickness_m + self.beacon_clearance_m
        return add(self.center, [0.0, 0.0, vertical_offset])
