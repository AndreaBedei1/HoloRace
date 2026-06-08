"""Classical square-gate detection from a front RGB camera frame."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

try:  # pragma: no cover - exercised through normal path when OpenCV is installed.
    import cv2
except ImportError:  # pragma: no cover - fallback is covered by the pure NumPy path.
    cv2 = None


@dataclass(frozen=True)
class GateDetection:
    found: bool = False
    normalized_x_error: float = 0.0
    normalized_y_error: float = 0.0
    area_fraction: float = 0.0
    angle_deg: float = 0.0
    confidence: float = 0.0
    bbox: tuple[int, int, int, int] | None = None

    @property
    def is_valid(self) -> bool:
        return self.found and self.confidence > 0.0 and self.bbox is not None


@dataclass
class SimpleGateDetector:
    """Detect a bright/neutral square frame or strong square contour."""

    min_area_fraction: float = 0.003
    max_area_fraction: float = 0.85
    min_confidence: float = 0.35
    min_brightness: float = 55.0
    max_saturation: float = 0.45
    square_tolerance: float = 0.45

    def detect(self, frame: Any) -> GateDetection:
        rgb = _as_rgb_array(frame)
        if rgb is None:
            return GateDetection()

        height, width = rgb.shape[:2]
        if height <= 0 or width <= 0:
            return GateDetection()

        mask = self._candidate_mask(rgb)
        if cv2 is not None:
            detection = self._detect_with_cv2(mask, width, height)
            if detection.found:
                return detection
            return self._detect_edges_with_cv2(rgb, width, height)

        return self._detect_from_mask_bbox(mask, width, height)

    def _candidate_mask(self, rgb: np.ndarray) -> np.ndarray:
        image = rgb.astype(np.float32, copy=False)
        channel_max = image.max(axis=2)
        channel_min = image.min(axis=2)
        saturation = (channel_max - channel_min) / np.maximum(channel_max, 1.0)
        mask = (channel_max >= self.min_brightness) & (saturation <= self.max_saturation)
        return mask.astype(np.uint8) * 255

    def _detect_with_cv2(self, mask: np.ndarray, width: int, height: int) -> GateDetection:
        kernel = np.ones((5, 5), dtype=np.uint8)
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return self._best_contour_detection(contours, width, height)

    def _detect_edges_with_cv2(self, rgb: np.ndarray, width: int, height: int) -> GateDetection:
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 60, 150)
        kernel = np.ones((3, 3), dtype=np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=1)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return self._best_contour_detection(contours, width, height)

    def _best_contour_detection(
        self,
        contours: list[np.ndarray],
        width: int,
        height: int,
    ) -> GateDetection:
        best: GateDetection | None = None
        best_score = 0.0
        image_area = float(width * height)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue

            bbox_fraction = (w * h) / image_area
            if bbox_fraction < self.min_area_fraction or bbox_fraction > self.max_area_fraction:
                continue

            aspect_score = min(w, h) / float(max(w, h))
            if aspect_score < 1.0 - self.square_tolerance:
                continue

            contour_area = max(float(cv2.contourArea(contour)), 1.0)
            extent = min(1.0, contour_area / float(w * h))
            area_score = min(1.0, bbox_fraction / 0.18)
            confidence = _clamp(0.55 * aspect_score + 0.25 * area_score + 0.20 * extent, 0.0, 1.0)
            if confidence < self.min_confidence:
                continue

            rect = cv2.minAreaRect(contour)
            angle = _normalize_rect_angle(rect)
            detection = self._from_bbox(
                x=x,
                y=y,
                w=w,
                h=h,
                width=width,
                height=height,
                area_fraction=bbox_fraction,
                angle_deg=angle,
                confidence=confidence,
            )
            if confidence > best_score:
                best = detection
                best_score = confidence

        return best if best is not None else GateDetection()

    def _detect_from_mask_bbox(self, mask: np.ndarray, width: int, height: int) -> GateDetection:
        ys, xs = np.nonzero(mask)
        if xs.size == 0 or ys.size == 0:
            return GateDetection()

        x0 = int(xs.min())
        x1 = int(xs.max())
        y0 = int(ys.min())
        y1 = int(ys.max())
        w = x1 - x0 + 1
        h = y1 - y0 + 1
        area_fraction = (w * h) / float(width * height)
        if area_fraction < self.min_area_fraction or area_fraction > self.max_area_fraction:
            return GateDetection()

        aspect_score = min(w, h) / float(max(w, h))
        if aspect_score < 1.0 - self.square_tolerance:
            return GateDetection()

        confidence = _clamp(0.65 * aspect_score + 0.35 * min(1.0, area_fraction / 0.18), 0.0, 1.0)
        if confidence < self.min_confidence:
            return GateDetection()

        return self._from_bbox(
            x=x0,
            y=y0,
            w=w,
            h=h,
            width=width,
            height=height,
            area_fraction=area_fraction,
            angle_deg=0.0,
            confidence=confidence,
        )

    def _from_bbox(
        self,
        x: int,
        y: int,
        w: int,
        h: int,
        width: int,
        height: int,
        area_fraction: float,
        angle_deg: float,
        confidence: float,
    ) -> GateDetection:
        center_x = x + w / 2.0
        center_y = y + h / 2.0
        return GateDetection(
            found=True,
            normalized_x_error=_clamp((center_x - width / 2.0) / (width / 2.0), -1.0, 1.0),
            normalized_y_error=_clamp((center_y - height / 2.0) / (height / 2.0), -1.0, 1.0),
            area_fraction=_clamp(area_fraction, 0.0, 1.0),
            angle_deg=float(angle_deg),
            confidence=_clamp(confidence, 0.0, 1.0),
            bbox=(int(x), int(y), int(w), int(h)),
        )


def _as_rgb_array(frame: Any) -> np.ndarray | None:
    if frame is None:
        return None

    array = np.asarray(frame)
    if array.ndim != 3 or array.shape[2] < 3:
        return None

    rgb = array[:, :, :3]
    if rgb.dtype.kind == "f" and rgb.size and float(np.nanmax(rgb)) <= 1.0:
        rgb = rgb * 255.0
    return np.nan_to_num(rgb, copy=False).astype(np.uint8, copy=False)


def _normalize_rect_angle(rect: tuple[tuple[float, float], tuple[float, float], float]) -> float:
    (_, _), (w, h), angle = rect
    if w < h:
        angle += 90.0
    while angle > 45.0:
        angle -= 90.0
    while angle < -45.0:
        angle += 90.0
    return float(angle)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
