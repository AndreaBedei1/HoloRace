"""Optional classical-vision helpers for near-gate alignment."""

from underwater_racing.vision.gate_detection import GateDetection, SimpleGateDetector
from underwater_racing.vision.visual_servo import VisualGateServo
from underwater_racing.vision.vision_guidance import VisionGuidance, VisionGuidanceResult

__all__ = [
    "GateDetection",
    "SimpleGateDetector",
    "VisualGateServo",
    "VisionGuidance",
    "VisionGuidanceResult",
]
