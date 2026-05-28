"""Virtual gate beacon guidance measurements."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence

from underwater_racing.geometry.transforms import normalize, normalize_angle_rad, subtract
from underwater_racing.racing.gate import RaceGate


@dataclass(frozen=True)
class BeaconMeasurement:
    active_gate_id: int
    beacon_id: int
    beacon_position: List[float]
    target_position: List[float]
    distance_m: float
    direction_world: List[float]
    bearing_error_rad: float
    vertical_error_m: float
    elevation_error_rad: float

    @property
    def bearing_error_deg(self) -> float:
        return math.degrees(self.bearing_error_rad)

    @property
    def elevation_error_deg(self) -> float:
        return math.degrees(self.elevation_error_rad)


@dataclass(frozen=True)
class VirtualGateBeacon:
    """A software beacon placed above a gate but targeting the gate opening."""

    gate: RaceGate

    @property
    def id(self) -> int:
        return int(self.gate.beacon_id)

    @property
    def position(self) -> List[float]:
        return self.gate.beacon_position

    def get_measurement(
        self,
        vehicle_position: Sequence[float],
        vehicle_yaw_deg: float = 0.0,
    ) -> BeaconMeasurement:
        target_delta = subtract(self.gate.target_position, vehicle_position)
        horizontal_distance = math.hypot(target_delta[0], target_delta[1])
        distance = math.sqrt(
            target_delta[0] ** 2 + target_delta[1] ** 2 + target_delta[2] ** 2
        )
        target_yaw = math.atan2(target_delta[1], target_delta[0])
        yaw_error = normalize_angle_rad(target_yaw - math.radians(vehicle_yaw_deg))
        elevation = math.atan2(target_delta[2], horizontal_distance)

        return BeaconMeasurement(
            active_gate_id=self.gate.id,
            beacon_id=self.id,
            beacon_position=self.position,
            target_position=self.gate.target_position,
            distance_m=distance,
            direction_world=normalize(target_delta),
            bearing_error_rad=yaw_error,
            vertical_error_m=target_delta[2],
            elevation_error_rad=elevation,
        )
