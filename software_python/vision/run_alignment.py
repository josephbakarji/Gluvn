"""Run the first live MediaPipe-to-Gluvn alignment experiment."""

from __future__ import annotations

import argparse
from pathlib import Path

from .calibration import load_calibration
from .coordinate_transform import CoordinateTransform
from .log_alignment import AlignmentLogger
from .mediapipe_tracker import MediaPipeHandsTracker
from .vision_ekf_bridge import VisionEkfBridge


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand", choices=("l", "r"), default="r")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--log", type=Path, default=Path("vision_alignment.csv"))
    parser.add_argument("--world-landmarks", action="store_true")
    parser.add_argument("--no-ekf", action="store_true", help="run camera-only when no glove is connected")
    parser.add_argument("--source", choices=("image", "world"), default=None,
                        help="explicit landmark source; --world-landmarks is retained as an alias")
    parser.add_argument("--show", action="store_true", help="show the latest 3D alignment after capture")
    args = parser.parse_args()

    transform = load_calibration(args.calibration) if args.calibration.exists() else CoordinateTransform.identity()
    if not args.calibration.exists():
        print(f"No calibration file at {args.calibration}; using identity transform.")
    use_world_landmarks = args.world_landmarks or args.source == "world"
    tracker = MediaPipeHandsTracker(args.camera, use_world_landmarks=use_world_landmarks)
    bridge = None if args.no_ekf else VisionEkfBridge(args.hand)
    logger = AlignmentLogger(args.log)
    latest = None
    try:
        if bridge:
            bridge.start()
        print("Fixed-camera experiment: camera pose is stationary; no dynamic camera pose estimation or EKF fusion is performed.")
        if use_world_landmarks:
            print("Source: MediaPipe WorldLandmarks (hand-local metric geometry), not a camera-fixed position.")
        else:
            print("Source: normalized image landmarks (non-metric); transformed values are visualization coordinates only.")
        for image, frame in tracker.frames():
            if len(frame.landmarks) == 0:
                continue
            transformed = transform.transform_points(frame.landmarks)
            ekf = bridge.latest() if bridge else None
            logger.write(frame, transformed, ekf)
            latest = (frame, transformed, ekf)
            print(f"t={frame.timestamp:.3f} kind={frame.coordinate_kind} source={frame.landmarks[0]} transformed={transformed[0]}")
            if args.show:
                import cv2
                cv2.imshow("MediaPipe alignment (press q to stop)", image)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        logger.close()
        if bridge:
            bridge.stop()
    if args.show:
        import cv2
        cv2.destroyAllWindows()
    if args.show and latest:
        from .visualize_alignment import show_alignment
        frame, transformed, ekf = latest
        show_alignment(frame.landmarks, transformed, ekf.position_m if ekf else None,
                   ekf.quaternion_wxyz if ekf else None,
                   source_label=frame.coordinate_kind,
                   target_label="EKF/world representation")


if __name__ == "__main__":
    main()
