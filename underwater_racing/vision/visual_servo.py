"""Visual servoing from image-space gate errors to rover commands."""

from __future__ import annotations

from dataclasses import dataclass

from underwater_racing.control.simple_gate_follower import RoverCommand
from underwater_racing.vision.gate_detection import GateDetection


@dataclass
class VisualGateServo:
    max_surge: float = 0.45
    alignment_surge: float = 0.16
    close_alignment_surge: float = 0.08
    max_sway: float = 0.28
    max_heave: float = 0.35
    max_yaw: float = 0.10
    sway_gain: float = 0.28
    heave_gain: float = 0.35
    yaw_gain: float = 0.09
    x_deadband: float = 0.06
    y_deadband: float = 0.06
    centered_tolerance: float = 0.12
    close_area_fraction: float = 0.25
    min_confidence: float = 0.35

    def command_from_detection(self, detection: GateDetection) -> RoverCommand:
        if not detection.found or detection.confidence < self.min_confidence:
            return RoverCommand()

        x_error = detection.normalized_x_error
        y_error = detection.normalized_y_error
        centered = (
            abs(x_error) <= self.centered_tolerance
            and abs(y_error) <= self.centered_tolerance
        )

        sway = 0.0 if abs(x_error) < self.x_deadband else self.sway_gain * x_error
        yaw = 0.0 if abs(x_error) < self.x_deadband else self.yaw_gain * x_error
        heave = 0.0 if abs(y_error) < self.y_deadband else -self.heave_gain * y_error

        if centered:
            surge = self.max_surge
        elif detection.area_fraction >= self.close_area_fraction:
            surge = self.close_alignment_surge
        else:
            surge = self.alignment_surge

        return RoverCommand(
            surge=_clamp(surge, 0.0, self.max_surge),
            sway=_clamp(sway, -self.max_sway, self.max_sway),
            heave=_clamp(heave, -self.max_heave, self.max_heave),
            yaw=_clamp(yaw, -self.max_yaw, self.max_yaw),
        )


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
