"""Reproducible fixed-camera similarity calibration."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

import numpy as np

from .coordinate_transform import CoordinateTransform


def fit_similarity(camera_points: Iterable[Iterable[float]], world_points: Iterable[Iterable[float]], estimate_scale: bool = True) -> CoordinateTransform:
    """Fit a proper rotation, optional scale, and translation from point pairs.

    Point pairs must describe the same physical landmarks in camera and world
    coordinates. This is an offline calibration helper, not online pose
    estimation and not an EKF update.
    """

    camera = np.asarray(camera_points, dtype=float)
    world = np.asarray(world_points, dtype=float)
    if camera.shape != world.shape or camera.ndim != 2 or camera.shape[1] != 3 or len(camera) < 3:
        raise ValueError("camera/world points must be matching Nx3 arrays with N >= 3")
    camera_center = camera.mean(axis=0)
    world_center = world.mean(axis=0)
    centered_camera = camera - camera_center
    centered_world = world - world_center
    covariance = centered_world.T @ centered_camera
    u, _, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(u @ vt) < 0:
        correction[2, 2] = -1.0
    rotation = u @ correction @ vt
    rotated_camera = (rotation @ centered_camera.T).T
    denominator = float(np.sum(centered_camera ** 2))
    scale = float(np.sum(rotated_camera * centered_world) / denominator) if estimate_scale else 1.0
    if scale <= 0 or not np.isfinite(scale):
        raise ValueError("calibration produced an invalid scale")
    translation = world_center - scale * rotation @ camera_center
    return CoordinateTransform(rotation, translation, scale)


def save_calibration(path: str | Path, transform: CoordinateTransform, metadata: dict | None = None) -> None:
    payload = {"transform": transform.to_dict(), "metadata": metadata or {}}
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_calibration(path: str | Path) -> CoordinateTransform:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return CoordinateTransform.from_dict(payload["transform"])
