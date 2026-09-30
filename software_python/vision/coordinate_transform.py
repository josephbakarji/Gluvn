"""Explicit camera-frame to NavEKF/world-frame transformations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class CoordinateTransform:
    """Apply ``p_world = scale * R_world_camera @ p_camera + t_world_camera``.

    ``R_world_camera`` is an active rotation mapping camera coordinates into
    the NavEKF world frame. ``t_world_camera`` is the camera origin in world
    metres. The transform does not infer metric scale from image landmarks.
    """

    R_world_camera: np.ndarray
    t_world_camera: np.ndarray
    scale: float = 1.0

    def __post_init__(self) -> None:
        rotation = np.asarray(self.R_world_camera, dtype=float)
        translation = np.asarray(self.t_world_camera, dtype=float)
        if rotation.shape != (3, 3) or translation.shape != (3,):
            raise ValueError("rotation must be 3x3 and translation must have length 3")
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            raise ValueError("transform parameters must be finite")
        if not np.isfinite(self.scale) or self.scale <= 0:
            raise ValueError("scale must be positive and finite")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
            raise ValueError("R_world_camera must be orthonormal")
        if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
            raise ValueError("R_world_camera must be a proper rotation")

    @classmethod
    def identity(cls) -> "CoordinateTransform":
        return cls(np.eye(3), np.zeros(3), 1.0)

    def transform_points(self, points: Iterable[Iterable[float]]) -> np.ndarray:
        points_array = np.asarray(points, dtype=float)
        if points_array.shape[-1] != 3:
            raise ValueError("points must have a final dimension of length 3")
        return (self.scale * (self.R_world_camera @ points_array.reshape(-1, 3).T)).T.reshape(points_array.shape) + self.t_world_camera

    def transform_point(self, point: Iterable[float]) -> np.ndarray:
        return self.transform_points(np.asarray(point, dtype=float)).reshape(3)

    def inverse(self) -> "CoordinateTransform":
        inverse_rotation = self.R_world_camera.T
        inverse_scale = 1.0 / self.scale
        inverse_translation = -inverse_scale * inverse_rotation @ self.t_world_camera
        return CoordinateTransform(inverse_rotation, inverse_translation, inverse_scale)

    def to_dict(self) -> dict:
        return {
            "R_world_camera": self.R_world_camera.tolist(),
            "t_world_camera_m": self.t_world_camera.tolist(),
            "scale": self.scale,
        }

    @classmethod
    def from_dict(cls, value: dict) -> "CoordinateTransform":
        return cls(value["R_world_camera"], value["t_world_camera_m"], value.get("scale", 1.0))


def quaternion_wxyz_to_rotation(quaternion: Iterable[float]) -> np.ndarray:
    """Return the same world<-sensor DCM convention used by NavEKF.h."""

    w, x, y, z = np.asarray(tuple(quaternion), dtype=float)
    norm = np.linalg.norm([w, x, y, z])
    if norm <= 1e-12:
        raise ValueError("quaternion norm is zero")
    w, x, y, z = np.asarray([w, x, y, z]) / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
