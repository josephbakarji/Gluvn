"""Experimental MediaPipe-to-Gluvn world-frame alignment tools.

This package deliberately stops before EKF measurement injection. It consumes
existing firmware telemetry through ``core.port_read.Reader``.
"""

from .coordinate_transform import CoordinateTransform
from .calibration import fit_similarity, load_calibration, save_calibration

__all__ = ["CoordinateTransform", "fit_similarity", "load_calibration", "save_calibration"]
