"""Fusion between beacon guidance and optional near-gate visual servoing."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from underwater_racing.control.simple_gate_follower import RoverCommand
from underwater_racing.vision.gate_detection import GateDetection, SimpleGateDetector
from underwater_racing.vision.visual_servo import VisualGateServo

BEACON_SOURCE = "beacon"
VISION_SOURCE = "vision"
FALLBACK_BEACON_SOURCE = "fallback_beacon"


@dataclass(frozen=True)
class VisionGuidanceResult:
    command: RoverCommand
    command_source: str
    detection: GateDetection


@dataclass
class VisionGuidance:
    near_gate_distance_m: float = 4.0
    min_confidence: float = 0.35
    detection_hold_s: float = 0.18
    fallback_max_surge: float = 0.45
    fallback_min_surge: float = 0.08
    fallback_max_reverse: float = 0.10
    fallback_max_sway: float = 0.22
    fallback_max_heave: float = 0.26
    fallback_max_yaw: float = 0.10
    detector: SimpleGateDetector = field(default_factory=SimpleGateDetector)
    servo: VisualGateServo = field(default_factory=VisualGateServo)
    _last_valid_detection: GateDetection | None = field(default=None, init=False, repr=False)
    _last_valid_time_s: float | None = field(default=None, init=False, repr=False)

    def compute_command(
        self,
        distance_to_gate_m: float,
        beacon_command: RoverCommand,
        frame: Any | None,
        time_s: float | None = None,
    ) -> VisionGuidanceResult:
        if distance_to_gate_m > self.near_gate_distance_m:
            self._clear_detection()
            return VisionGuidanceResult(
                command=beacon_command,
                command_source=BEACON_SOURCE,
                detection=GateDetection(),
            )

        detection = self.detector.detect(frame) if frame is not None else GateDetection()
        return self.fuse_detection(
            distance_to_gate_m=distance_to_gate_m,
            beacon_command=beacon_command,
            detection=detection,
            time_s=time_s,
        )

    def fuse_detection(
        self,
        distance_to_gate_m: float,
        beacon_command: RoverCommand,
        detection: GateDetection | None,
        time_s: float | None = None,
    ) -> VisionGuidanceResult:
        detection = detection or GateDetection()
        if distance_to_gate_m > self.near_gate_distance_m:
            self._clear_detection()
            return VisionGuidanceResult(
                command=beacon_command,
                command_source=BEACON_SOURCE,
                detection=detection,
            )

        if self.is_valid_detection(detection):
            self._remember_detection(detection, time_s)
            return VisionGuidanceResult(
                command=self.servo.command_from_detection(detection),
                command_source=VISION_SOURCE,
                detection=detection,
            )

        held_detection = self._recent_detection(time_s)
        if held_detection is not None:
            return VisionGuidanceResult(
                command=self.servo.command_from_detection(held_detection),
                command_source=VISION_SOURCE,
                detection=held_detection,
            )

        return VisionGuidanceResult(
            command=self._cautious_fallback_command(beacon_command, distance_to_gate_m),
            command_source=FALLBACK_BEACON_SOURCE,
            detection=detection,
        )

    def is_valid_detection(self, detection: GateDetection) -> bool:
        return detection.found and detection.confidence >= self.min_confidence

    def _remember_detection(self, detection: GateDetection, time_s: float | None) -> None:
        self._last_valid_detection = detection
        self._last_valid_time_s = time_s

    def _clear_detection(self) -> None:
        self._last_valid_detection = None
        self._last_valid_time_s = None

    def _recent_detection(self, time_s: float | None) -> GateDetection | None:
        if self._last_valid_detection is None:
            return None
        if time_s is None or self._last_valid_time_s is None:
            return None
        if time_s - self._last_valid_time_s > self.detection_hold_s:
            return None
        return self._last_valid_detection

    def _cautious_fallback_command(
        self,
        beacon_command: RoverCommand,
        distance_to_gate_m: float,
    ) -> RoverCommand:
        progress = _clamp(distance_to_gate_m / self.near_gate_distance_m, 0.0, 1.0)
        surge_cap = self.fallback_min_surge + (
            self.fallback_max_surge - self.fallback_min_surge
        ) * progress
        alignment_load = _clamp(
            max(
                abs(beacon_command.sway) / 0.55,
                abs(beacon_command.yaw) / 0.25,
            ),
            0.0,
            1.0,
        )
        surge_cap *= 1.0 - 0.55 * alignment_load

        if beacon_command.surge >= 0.0:
            surge = min(beacon_command.surge, surge_cap)
        else:
            surge = max(beacon_command.surge, -self.fallback_max_reverse)

        return RoverCommand(
            surge=_clamp(surge, -self.fallback_max_reverse, self.fallback_max_surge),
            sway=_clamp(beacon_command.sway, -self.fallback_max_sway, self.fallback_max_sway),
            heave=_clamp(beacon_command.heave, -self.fallback_max_heave, self.fallback_max_heave),
            yaw=_clamp(beacon_command.yaw, -self.fallback_max_yaw, self.fallback_max_yaw),
        )


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
