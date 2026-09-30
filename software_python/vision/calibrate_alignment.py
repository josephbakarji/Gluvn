"""Fit and save a fixed-camera similarity transform from paired CSV points."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from .calibration import fit_similarity, save_calibration


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("points", type=Path, help="CSV with camera_x/y/z and world_x/y/z columns")
    parser.add_argument("output", type=Path, help="JSON calibration output")
    parser.add_argument("--rigid", action="store_true", help="force scale=1 instead of fitting scale")
    args = parser.parse_args()

    with args.points.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    camera = np.array([[float(row[f"camera_{axis}"]) for axis in "xyz"] for row in rows])
    world = np.array([[float(row[f"world_{axis}"]) for axis in "xyz"] for row in rows])
    transform = fit_similarity(camera, world, estimate_scale=not args.rigid)
    save_calibration(args.output, transform, {
        "source_csv": str(args.points),
        "point_count": len(rows),
        "scale_fitted": not args.rigid,
    })
    print(f"saved {args.output}: scale={transform.scale:.6g}, translation_m={transform.t_world_camera}")


if __name__ == "__main__":
    main()
