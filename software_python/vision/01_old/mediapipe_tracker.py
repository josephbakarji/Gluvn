"""Optional MediaPipe Hands acquisition; import MediaPipe only when used."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Iterator

import numpy as np

LANDMARK_NAMES = (
    "wrist", "thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip",
    "index_mcp", "index_pip", "index_dip", "index_tip",
    "middle_mcp", "middle_pip", "middle_dip", "middle_tip",
    "ring_mcp", "ring_pip", "ring_dip", "ring_tip",
    "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip",
)


@dataclass(frozen=True)
class LandmarkFrame:
    timestamp: float  # PC wall-clock time from time.time(), seconds since epoch
    landmarks: np.ndarray
    coordinate_kind: str
    handedness: str | None = None


class MediaPipeHandsTracker:
    """Capture 21 landmarks from the existing Python camera environment.

    ``image_normalized`` means x/y are normalized image coordinates and z is
    relative/non-metric; these values must never be interpreted as metres.
    ``world_metric_hand_local`` uses MediaPipe WorldLandmarks when available.
    Those coordinates are approximately metric but hand-local, with an origin
    near the hand's geometric centre, not a camera-fixed origin.
    """

    def __init__(self, camera_index: int = 0, use_world_landmarks: bool = False, max_num_hands: int = 1):
        self.camera_index = camera_index
        self.use_world_landmarks = use_world_landmarks
        self.max_num_hands = max_num_hands

    def frames(self) -> Iterator[tuple[object, LandmarkFrame]]:
        import cv2
        import mediapipe as mp

        hands_api = mp.solutions.hands
        capture = cv2.VideoCapture(self.camera_index)
        if not capture.isOpened():
            raise RuntimeError(f"could not open camera {self.camera_index}")
        with hands_api.Hands(static_image_mode=False, max_num_hands=self.max_num_hands,
                             model_complexity=1, min_detection_confidence=0.6,
                             min_tracking_confidence=0.6) as hands:
            try:
                while True:
                    ok, image = capture.read()
                    if not ok:
                        continue
                    result = hands.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                    if not result.multi_hand_landmarks:
                        yield image, LandmarkFrame(time.time(), np.empty((0, 3)), "none")
                        continue
                    hand = result.multi_hand_landmarks[0]
                    use_world = self.use_world_landmarks and bool(result.multi_hand_world_landmarks)
                    source = result.multi_hand_world_landmarks[0] if use_world else hand
                    points = np.array([[p.x, p.y, p.z] for p in source.landmark], dtype=float)
                    handedness = result.multi_handedness[0].classification[0].label if result.multi_handedness else None
                    kind = "world_metric_hand_local" if use_world else "image_normalized"
                    yield image, LandmarkFrame(time.time(), points, kind, handedness)
            finally:
                capture.release()
