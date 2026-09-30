"""CSV logging for camera/world alignment experiments."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable

from .mediapipe_tracker import LANDMARK_NAMES, LandmarkFrame
from .vision_ekf_bridge import EkfTelemetry


class AlignmentLogger:
    def __init__(self, path: str | Path):
        self.file = Path(path).open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=[
            "timestamp", "landmark_id", "landmark_name", "camera_x", "camera_y", "camera_z",
            "world_x", "world_y", "world_z", "ekf_position_x", "ekf_position_y", "ekf_position_z",
            "mediapipe_timestamp", "ekf_host_timestamp", "ekf_parser_timestamp",
            "ekf_device_us", "ekf_device_time_s", "mediapipe_minus_device_s",
            "estimated_pc_receipt_minus_device_s",
            "ekf_qw", "ekf_qx", "ekf_qy", "ekf_qz", "ekf_nav_armed", "camera_coordinate_kind",
        ])
        self.writer.writeheader()

    def write(self, frame: LandmarkFrame, world_points: Iterable[Iterable[float]], ekf: EkfTelemetry | None) -> None:
        world = list(world_points)
        for index, (camera, transformed) in enumerate(zip(frame.landmarks, world)):
            position = ekf.position_m if ekf else (None, None, None)
            quaternion = ekf.quaternion_wxyz if ekf else (None, None, None, None)
            self.writer.writerow({
                "timestamp": frame.timestamp, "landmark_id": index,
                "landmark_name": LANDMARK_NAMES[index],
                "camera_x": camera[0], "camera_y": camera[1], "camera_z": camera[2],
                "world_x": transformed[0], "world_y": transformed[1], "world_z": transformed[2],
                "ekf_position_x": position[0], "ekf_position_y": position[1], "ekf_position_z": position[2],
                "mediapipe_timestamp": frame.timestamp,
                "ekf_host_timestamp": ekf.host_timestamp if ekf else None,
                "ekf_parser_timestamp": ekf.parser_timestamp if ekf else None,
                "ekf_device_time_s": ekf.device_time_s if ekf else None,
                "mediapipe_minus_device_s": (
                    frame.timestamp - ekf.device_time_s if ekf else None
                ),
                "estimated_pc_receipt_minus_device_s": (
                    ekf.host_timestamp - ekf.device_time_s if ekf else None
                ),
                "ekf_qw": quaternion[0], "ekf_qx": quaternion[1], "ekf_qy": quaternion[2], "ekf_qz": quaternion[3],
                "ekf_device_us": ekf.device_us if ekf else None,
                "ekf_nav_armed": ekf.nav_armed if ekf else None,
                "camera_coordinate_kind": frame.coordinate_kind,
            })
        self.file.flush()

    def close(self) -> None:
        self.file.close()
