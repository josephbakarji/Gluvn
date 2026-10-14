"""Visualize a saved alignment CSV without camera, glove, or MediaPipe."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from .visualize_alignment import show_alignment


def _read_frames(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    grouped: dict[float, list[dict]] = {}
    for row in rows:
        grouped.setdefault(float(row["mediapipe_timestamp"]), []).append(row)
    return [grouped[timestamp] for timestamp in sorted(grouped)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--frame", type=int, default=-1, help="frame index, default: latest")
    parser.add_argument("--plot-offset", action="store_true", help="plot observed PC/device offset over time")
    args = parser.parse_args()

    frames = _read_frames(args.recording)
    if not frames:
        raise SystemExit("recording contains no landmark rows")
    frame = frames[args.frame]
    camera = np.array([[float(row["camera_x"]), float(row["camera_y"]), float(row["camera_z"])] for row in frame])
    transformed = np.array([[float(row["world_x"]), float(row["world_y"]), float(row["world_z"])] for row in frame])
    position_values = [frame[0][f"ekf_position_{axis}"] for axis in "xyz"]
    quaternion_values = [frame[0][f"ekf_q{component}"] for component in "wxyz"]
    position = np.array([float(value) for value in position_values]) if all(position_values) else None
    quaternion = np.array([float(value) for value in quaternion_values]) if all(quaternion_values) else None
    kind = frame[0].get("camera_coordinate_kind", "unknown")
    show_alignment(camera, transformed, position, quaternion,
                   title=f"Recorded alignment: {kind}, frame {args.frame}",
                   source_label=kind, target_label="EKF/world representation")

    offsets = [
        (float(row["mediapipe_timestamp"]), float(row["mediapipe_minus_device_s"]))
        for group in frames for row in group[:1]
        if row.get("estimated_pc_minus_device_s")
    ]
    if offsets:
        values = np.array([value for _, value in offsets])
        print(f"Observed MediaPipe-PC minus device-clock offset: median={np.median(values):.6f}s "
              f"range=[{values.min():.6f}, {values.max():.6f}]s; this is not synchronization.")
    else:
        print("No EKF/device timestamps were recorded; offset inspection is unavailable.")

    if args.plot_offset and offsets:
        import matplotlib.pyplot as plt
        times, values = zip(*offsets)
        plt.figure("Observed clock offset")
        plt.plot(times, values)
        plt.xlabel("MediaPipe PC timestamp (s since epoch)")
        plt.ylabel("MediaPipe PC time - unwrapped device time (s)")
        plt.title("Paired clock offset diagnostic, not synchronization")
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    main()
