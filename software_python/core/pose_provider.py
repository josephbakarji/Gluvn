"""Shared demand-managed pose access for one :class:`core.port_read.Reader`.

The provider reference-counts motion and NavEKF-quaternion requests independently
per hand. It sends each stream's ON command on the first request and OFF on the
last release, so consumers can share one reader without changing one another's
stream requirements.

`request(hand, motion=True, ...)` also yields `lin_accel` on the resulting
`PoseSample` — world-frame, gravity-subtracted linear acceleration. It has no
separate request flag: the firmware bundles it into the same MOTION_STREAM_ON
frame as velocity/position (see m5stick_firmware.ino / port_read.py), since
nothing needs one without the other. `PoseSample.lin_accel` is None whenever
`motion` was not requested, same as `position`/`velocity`.

This supports multiple consumers within one Python process sharing one BLE/USB
connection. It does not support multiple separate processes connecting to the
same glove; BLE peripherals generally allow one central connection. A future
multi-process design should have one owner process rebroadcast decoded samples.
"""

from __future__ import annotations

from dataclasses import dataclass
from threading import Condition, RLock, Thread
from typing import Optional, Tuple
import queue

from core.port_read import Reader


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class PoseSample:
    """Decoded pose data from one firmware sample.

    position/velocity/orientation/lin_accel are None whenever `motion` (and,
    for orientation, `orientation`) wasn't requested.

    nav_armed/nav_zupt_active/nav_zupt_starved/zupt_mahalanobis are None
    whenever `motion` wasn't requested -- port_read.py only sets these keys
    when has_motion was true for a sample:
      - nav_armed: hard-gate. False means position/velocity/lin_accel are
        not yet trustworthy (e.g. the ~0.3s re-arm/settle window). None is
        NOT the same as False -- None means "no reading", not "distrust this".
      - nav_zupt_active: a ZUPT correction ran on this cycle (point-in-time).
      - nav_zupt_starved: stationary-detection gate hasn't corrected in >5s
        -- position/velocity error likely accumulating unchecked.
      - zupt_mahalanobis: continuous soft-scaling companion to
        nav_zupt_starved. Chi-square goodness-of-fit for the last ZUPT
        candidate -- near 0 is a confident stationary fit; NavEKF's own
        accept/reject gate is 16.0, wire-clamped at 64.0. None also covers
        "NavEKF hasn't evaluated a ZUPT candidate yet this session" --
        that case is NOT distinguished from "motion not requested" here;
        check nav_armed's presence/value if the difference matters.
    """

    position: Optional[Vector3]
    velocity: Optional[Vector3]
    orientation: Optional[Quaternion]
    lin_accel: Optional[Vector3]
    nav_armed: Optional[bool]
    nav_zupt_active: Optional[bool]
    nav_zupt_starved: Optional[bool]
    zupt_mahalanobis: Optional[float]
    seq: int
    device_us: int


class _PoseRequest:
    def __init__(self, provider: "PoseProvider", hand: str,
                 motion: bool, orientation: bool):
        self._provider = provider
        self._hand = hand
        self._motion = motion
        self._orientation = orientation
        self._released = False

    def __enter__(self) -> "_PoseRequest":
        self._provider._acquire(self._hand, self._motion, self._orientation)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.release()

    def release(self) -> None:
        if not self._released:
            self._released = True
            self._provider._release(self._hand, self._motion, self._orientation)


class PoseProvider:
    """Reference-counted pose stream manager around one ``Reader`` instance.

    If ``reader`` is omitted, a Reader is created with IMU parsing and
    ``nav_quat_enabled=True`` so a NavEKF quaternion-only frame is decoded as
    ``orientation``. Call :meth:`start` once, then use ``request(...)`` around
    each consumer's lifetime and :meth:`latest` or :meth:`wait_for_sample` to
    obtain decoded pose data.
    """

    def __init__(self, reader: Optional[Reader] = None, hands=("l", "r"),
                 reader_kwargs=None):
        self.reader = reader or self._make_reader(hands, reader_kwargs or {})
        self.hands = tuple(self.reader.hands)
        self._counts = {
            hand: {"motion": 0, "orientation": 0} for hand in self.hands
        }
        self._latest = {hand: None for hand in self.hands}
        self._lock = RLock()
        self._condition = Condition(self._lock)
        self._stop_event = None
        self._dispatchers = {}

    @staticmethod
    def _make_reader(hands, reader_kwargs):
        sensor_config = {
            hand: {
                "flex": False,
                "press": False,
                "imu": True,
                "nav_quat_enabled": True,
            }
            for hand in hands
        }
        sensor_config.update(reader_kwargs.pop("sensor_config", {}))
        return Reader(sensor_config=sensor_config, **reader_kwargs)

    def start(self) -> None:
        """Start the underlying reader and the per-hand sample dispatchers."""
        self.reader.start_readers()
        from threading import Event
        self._stop_event = Event()
        for hand in self.hands:
            thread = Thread(target=self._dispatch, args=(hand,), daemon=True)
            self._dispatchers[hand] = thread
            thread.start()

    def stop(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        self.reader.stop_readers()
        for thread in self._dispatchers.values():
            thread.join(timeout=1.0)
        self._dispatchers.clear()

    def request(self, hand: str, motion=False, orientation=False):
        if hand not in self._counts:
            raise KeyError(f"unknown hand: {hand}")
        if not motion and not orientation:
            raise ValueError("request must select motion and/or orientation")
        return _PoseRequest(self, hand, bool(motion), bool(orientation))

    def latest(self, hand: str) -> Optional[PoseSample]:
        with self._lock:
            return self._latest[hand]

    def wait_for_sample(self, hand: str, timeout=None) -> Optional[PoseSample]:
        with self._condition:
            previous = self._latest[hand]
            self._condition.wait_for(lambda: self._latest[hand] is not previous,
                                     timeout=timeout)
            return self._latest[hand]

    def counts(self, hand: str):
        with self._lock:
            return dict(self._counts[hand])

    def _acquire(self, hand, motion, orientation):
        with self._lock:
            if motion:
                if self._counts[hand]["motion"] == 0:
                    self.reader.send_command(hand, "MOTION_STREAM_ON")
                self._counts[hand]["motion"] += 1
            if orientation:
                if self._counts[hand]["orientation"] == 0:
                    self.reader.send_command(hand, "NAV_QUAT_STREAM_ON")
                self._counts[hand]["orientation"] += 1

    def _release(self, hand, motion, orientation):
        with self._lock:
            if motion:
                self._counts[hand]["motion"] -= 1
                if self._counts[hand]["motion"] == 0:
                    self.reader.send_command(hand, "MOTION_STREAM_OFF")
            if orientation:
                self._counts[hand]["orientation"] -= 1
                if self._counts[hand]["orientation"] == 0:
                    self.reader.send_command(hand, "NAV_QUAT_STREAM_OFF")

    def _dispatch(self, hand):
        parser_q = self.reader.threads[hand]["parser"].getQ()
        while self._stop_event is not None and not self._stop_event.is_set():
            try:
                sample = parser_q.get(timeout=0.1)
            except queue.Empty:
                continue
            pose = PoseSample(
                position=tuple(sample["position"]) if "position" in sample else None,
                velocity=tuple(sample["velocity"]) if "velocity" in sample else None,
                orientation=tuple(sample["nav_quat"]) if "nav_quat" in sample else None,
                lin_accel=tuple(sample["lin_accel"]) if "lin_accel" in sample else None,
                # .get(...) with a None default preserves "not requested/not
                # available" rather than defaulting nav_armed to a misleading False.
                nav_armed=sample.get("nav_armed"),
                nav_zupt_active=sample.get("nav_zupt_active"),
                nav_zupt_starved=sample.get("nav_zupt_starved"),
                zupt_mahalanobis=sample.get("zupt_mahalanobis"),
                seq=sample.get("seq", 0),
                device_us=sample.get("device_us", 0),
            )
            with self._condition:
                self._latest[hand] = pose
                self._condition.notify_all()
