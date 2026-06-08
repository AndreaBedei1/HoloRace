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
    detector: SimpleGateDetector = field(default_factory=SimpleGateDetector)
    servo: VisualGateServo = field(default_factory=VisualGateServo)

    def compute_command(
        self,
        distance_to_gate_m: float,
        beacon_command: RoverCommand,
        frame: Any | None,
    ) -> VisionGuidanceResult:
        if distance_to_gate_m > self.near_gate_distance_m:
            return VisionGuidanceResult(
                command=beacon_command,
                command_source=BEACON_SOURCE,
                detection=GateDetection(),
            )

        detection = self.detector.detect(frame)
        return self.fuse_detection(
            distance_to_gate_m=distance_to_gate_m,
            beacon_command=beacon_command,
            detection=detection,
        )

    def fuse_detection(
        self,
        distance_to_gate_m: float,
        beacon_command: RoverCommand,
        detection: GateDetection | None,
    ) -> VisionGuidanceResult:
        detection = detection or GateDetection()
        if distance_to_gate_m > self.near_gate_distance_m:
            return VisionGuidanceResult(
                command=beacon_command,
                command_source=BEACON_SOURCE,
                detection=detection,
            )

        if self.is_valid_detection(detection):
            return VisionGuidanceResult(
                command=self.servo.command_from_detection(detection),
                command_source=VISION_SOURCE,
                detection=detection,
            )

        return VisionGuidanceResult(
            command=beacon_command,
            command_source=FALLBACK_BEACON_SOURCE,
            detection=detection,
        )

    def is_valid_detection(self, detection: GateDetection) -> bool:
        return detection.found and detection.confidence >= self.min_confidence
