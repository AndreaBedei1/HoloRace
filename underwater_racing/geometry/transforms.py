"""Small vector and yaw transform utilities."""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence

Vector3 = Sequence[float]


def as_vector3(value: Iterable[float]) -> List[float]:
    values = [float(v) for v in value]
    if len(values) != 3:
        raise ValueError(f"Expected a 3D vector, got {len(values)} values")
    return values


def add(a: Vector3, b: Vector3) -> List[float]:
    return [float(a[0]) + float(b[0]), float(a[1]) + float(b[1]), float(a[2]) + float(b[2])]


def subtract(a: Vector3, b: Vector3) -> List[float]:
    return [float(a[0]) - float(b[0]), float(a[1]) - float(b[1]), float(a[2]) - float(b[2])]


def scale(v: Vector3, factor: float) -> List[float]:
    return [float(v[0]) * factor, float(v[1]) * factor, float(v[2]) * factor]


def dot(a: Vector3, b: Vector3) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def norm(v: Vector3) -> float:
    return math.sqrt(dot(v, v))


def normalize(v: Vector3) -> List[float]:
    length = norm(v)
    if length == 0.0:
        return [0.0, 0.0, 0.0]
    return [float(v[0]) / length, float(v[1]) / length, float(v[2]) / length]


def yaw_axes(yaw_deg: float) -> tuple[List[float], List[float], List[float]]:
    """Return gate local forward, right, and up axes in world coordinates."""
    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    forward = [c, s, 0.0]
    right = [-s, c, 0.0]
    up = [0.0, 0.0, 1.0]
    return forward, right, up


def rotate_yaw(local: Vector3, yaw_deg: float) -> List[float]:
    forward, right, up = yaw_axes(yaw_deg)
    return add(add(scale(forward, float(local[0])), scale(right, float(local[1]))), scale(up, float(local[2])))


def normalize_angle_deg(angle_deg: float) -> float:
    """Normalize an angle to [-180, 180)."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def normalize_angle_rad(angle_rad: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def interpolate(a: Vector3, b: Vector3, t: float) -> List[float]:
    return [
        float(a[0]) + (float(b[0]) - float(a[0])) * t,
        float(a[1]) + (float(b[1]) - float(a[1])) * t,
        float(a[2]) + (float(b[2]) - float(a[2])) * t,
    ]
