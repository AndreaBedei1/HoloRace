import unittest

import numpy as np

from underwater_racing.control.simple_gate_follower import RoverCommand
from underwater_racing.holoocean.vehicle_loader import (
    FRONT_RGB_CAMERA_NAME,
    build_bluerov_config,
)
from underwater_racing.vision.gate_detection import GateDetection, SimpleGateDetector
from underwater_racing.vision.visual_servo import VisualGateServo
from underwater_racing.vision.vision_guidance import (
    BEACON_SOURCE,
    FALLBACK_BEACON_SOURCE,
    VISION_SOURCE,
    VisionGuidance,
)


def synthetic_gate_frame(x_offset=0):
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    frame[:, :] = [20, 60, 120]
    x0 = 45 + x_offset
    x1 = 115 + x_offset
    y0 = 30
    y1 = 90
    thickness = 10
    frame[y0 : y0 + thickness, x0:x1] = [220, 220, 220]
    frame[y1 - thickness : y1, x0:x1] = [220, 220, 220]
    frame[y0:y1, x0 : x0 + thickness] = [220, 220, 220]
    frame[y0:y1, x1 - thickness : x1] = [220, 220, 220]
    return frame


def valid_detection(x_error=0.3, y_error=-0.2):
    return GateDetection(
        found=True,
        normalized_x_error=x_error,
        normalized_y_error=y_error,
        area_fraction=0.12,
        angle_deg=0.0,
        confidence=0.9,
        bbox=(40, 30, 80, 80),
    )


class GateDetectionTests(unittest.TestCase):
    def test_detection_dataclass_defaults_to_not_found(self):
        detection = GateDetection()

        self.assertFalse(detection.found)
        self.assertFalse(detection.is_valid)
        self.assertIsNone(detection.bbox)
        self.assertEqual(detection.confidence, 0.0)

    def test_simple_detector_finds_synthetic_square_gate(self):
        detection = SimpleGateDetector().detect(synthetic_gate_frame())

        self.assertTrue(detection.found)
        self.assertTrue(detection.is_valid)
        self.assertGreater(detection.confidence, 0.35)
        self.assertAlmostEqual(detection.normalized_x_error, 0.0, delta=0.08)
        self.assertAlmostEqual(detection.normalized_y_error, 0.0, delta=0.08)
        self.assertIsNotNone(detection.bbox)


class VisualGateServoTests(unittest.TestCase):
    def test_visual_servo_command_signs_follow_image_errors(self):
        servo = VisualGateServo()
        command = servo.command_from_detection(valid_detection(x_error=0.5, y_error=-0.4))

        self.assertGreater(command.sway, 0.0)
        self.assertGreater(command.yaw, 0.0)
        self.assertGreater(command.heave, 0.0)
        self.assertLessEqual(abs(command.yaw), servo.max_yaw)

    def test_visual_servo_moves_forward_when_centered(self):
        servo = VisualGateServo()
        command = servo.command_from_detection(valid_detection(x_error=0.02, y_error=-0.02))

        self.assertAlmostEqual(command.surge, servo.max_surge)
        self.assertAlmostEqual(command.sway, 0.0)
        self.assertAlmostEqual(command.heave, 0.0)
        self.assertAlmostEqual(command.yaw, 0.0)


class VisionGuidanceFusionTests(unittest.TestCase):
    def test_fusion_uses_beacon_when_far_even_with_valid_detection(self):
        guidance = VisionGuidance()
        beacon = RoverCommand(surge=0.8, yaw=-0.1)

        result = guidance.fuse_detection(
            distance_to_gate_m=4.1,
            beacon_command=beacon,
            detection=valid_detection(),
        )

        self.assertEqual(result.command_source, BEACON_SOURCE)
        self.assertEqual(result.command, beacon)

    def test_fusion_switches_to_vision_when_near_and_detection_valid(self):
        guidance = VisionGuidance()
        beacon = RoverCommand(surge=0.8, yaw=-0.1)

        result = guidance.fuse_detection(
            distance_to_gate_m=3.0,
            beacon_command=beacon,
            detection=valid_detection(x_error=0.4, y_error=0.0),
        )

        self.assertEqual(result.command_source, VISION_SOURCE)
        self.assertNotEqual(result.command, beacon)
        self.assertGreater(result.command.yaw, 0.0)

    def test_fusion_falls_back_to_beacon_when_near_detection_invalid(self):
        guidance = VisionGuidance()
        beacon = RoverCommand(surge=0.8, yaw=-0.1)

        result = guidance.fuse_detection(
            distance_to_gate_m=3.0,
            beacon_command=beacon,
            detection=GateDetection(found=False),
        )

        self.assertEqual(result.command_source, FALLBACK_BEACON_SOURCE)
        self.assertGreater(result.command.surge, 0.0)
        self.assertLess(result.command.surge, beacon.surge)
        self.assertEqual(result.command.yaw, beacon.yaw)

    def test_near_fallback_caps_aggressive_beacon_corrections(self):
        guidance = VisionGuidance()
        beacon = RoverCommand(surge=1.0, sway=0.55, heave=0.4, yaw=0.25)

        result = guidance.fuse_detection(
            distance_to_gate_m=2.5,
            beacon_command=beacon,
            detection=GateDetection(found=False),
        )

        self.assertEqual(result.command_source, FALLBACK_BEACON_SOURCE)
        self.assertLess(result.command.surge, 0.25)
        self.assertLessEqual(abs(result.command.sway), guidance.fallback_max_sway)
        self.assertLessEqual(abs(result.command.heave), guidance.fallback_max_heave)
        self.assertLessEqual(abs(result.command.yaw), guidance.fallback_max_yaw)

    def test_missing_frame_reuses_recent_valid_detection_briefly(self):
        guidance = VisionGuidance()
        beacon = RoverCommand(surge=0.8, yaw=-0.1)
        first = guidance.fuse_detection(
            distance_to_gate_m=3.0,
            beacon_command=beacon,
            detection=valid_detection(x_error=0.4, y_error=0.0),
            time_s=10.0,
        )
        held = guidance.fuse_detection(
            distance_to_gate_m=3.0,
            beacon_command=beacon,
            detection=GateDetection(found=False),
            time_s=10.1,
        )

        self.assertEqual(first.command_source, VISION_SOURCE)
        self.assertEqual(held.command_source, VISION_SOURCE)
        self.assertEqual(held.detection, first.detection)


class VisionCameraConfigTests(unittest.TestCase):
    def test_front_rgb_camera_is_optional(self):
        default_sensors = build_bluerov_config()["sensors"]
        vision_sensors = build_bluerov_config(enable_front_rgb_camera=True)["sensors"]

        self.assertNotIn("RGBCamera", [sensor["sensor_type"] for sensor in default_sensors])
        camera = [sensor for sensor in vision_sensors if sensor["sensor_type"] == "RGBCamera"]
        self.assertEqual(len(camera), 1)
        self.assertEqual(camera[0]["sensor_name"], FRONT_RGB_CAMERA_NAME)
        self.assertIn("CaptureWidth", camera[0]["configuration"])


if __name__ == "__main__":
    unittest.main()
