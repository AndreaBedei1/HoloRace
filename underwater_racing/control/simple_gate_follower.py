"""Simple first-pass controller that follows virtual gate beacon guidance."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

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
    max_yaw: float = 0.35
    surge_gain: float = 0.35
    heave_gain: float = 0.45
    yaw_gain: float = 0.45
    slow_radius_m: float = 0.8
    min_turning_surge: float = 0.20
    yaw_deadband_deg: float = 5.0
    max_yaw_delta_per_step: float = 0.05
    near_target_radius_m: float = 0.45
    _previous_yaw_command: float = field(default=0.0, init=False, repr=False)

    def compute_command(
        self,
        measurement: BeaconMeasurement,
        keep_forward_near_target: bool = False,
    ) -> RoverCommand:
        alignment = max(0.0, math.cos(measurement.bearing_error_rad))
        surge = self.surge_gain * measurement.distance_m * alignment
        if measurement.distance_m < self.slow_radius_m:
            surge *= measurement.distance_m / self.slow_radius_m
        if abs(measurement.bearing_error_deg) < 90.0 and surge < self.min_turning_surge:
            surge = self.min_turning_surge

        yaw = self._compute_yaw(measurement)
        if measurement.distance_m < self.near_target_radius_m and keep_forward_near_target:
            surge = max(surge, self.min_turning_surge)
        heave = self.heave_gain * measurement.vertical_error_m

        return RoverCommand(
            surge=_clamp(surge, 0.0, self.max_surge),
            sway=0.0,
            heave=_clamp(heave, -self.max_heave, self.max_heave),
            yaw=_clamp(yaw, -self.max_yaw, self.max_yaw),
        )

    def _compute_yaw(self, measurement: BeaconMeasurement) -> float:
        if measurement.distance_m < self.near_target_radius_m:
            self._previous_yaw_command = 0.0
            return 0.0

        if abs(measurement.bearing_error_deg) < self.yaw_deadband_deg:
            self._previous_yaw_command = 0.0
            return 0.0

        target_yaw = _clamp(
            self.yaw_gain * measurement.bearing_error_rad,
            -self.max_yaw,
            self.max_yaw,
        )
        delta = _clamp(
            target_yaw - self._previous_yaw_command,
            -self.max_yaw_delta_per_step,
            self.max_yaw_delta_per_step,
        )
        self._previous_yaw_command += delta
        return self._previous_yaw_command
