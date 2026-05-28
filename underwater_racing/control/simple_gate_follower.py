"""Simple first-pass controller that follows virtual gate beacon guidance."""

from __future__ import annotations

import math
from dataclasses import dataclass

from underwater_racing.racing.beacon import BeaconMeasurement


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class RoverCommand:
    surge: float = 0.0
    sway: float = 0.0
    heave: float = 0.0
    yaw: float = 0.0


@dataclass
class SimpleGateFollower:
    max_surge: float = 1.0
    max_heave: float = 0.7
    max_yaw: float = 0.8
    surge_gain: float = 0.35
    heave_gain: float = 0.45
    yaw_gain: float = 1.1
    slow_radius_m: float = 0.8

    def compute_command(self, measurement: BeaconMeasurement) -> RoverCommand:
        alignment = max(0.0, math.cos(measurement.bearing_error_rad))
        surge = self.surge_gain * measurement.distance_m * alignment
        if measurement.distance_m < self.slow_radius_m:
            surge *= measurement.distance_m / self.slow_radius_m

        yaw = self.yaw_gain * measurement.bearing_error_rad
        heave = self.heave_gain * measurement.vertical_error_m

        return RoverCommand(
            surge=_clamp(surge, 0.0, self.max_surge),
            sway=0.0,
            heave=_clamp(heave, -self.max_heave, self.max_heave),
            yaw=_clamp(yaw, -self.max_yaw, self.max_yaw),
        )
