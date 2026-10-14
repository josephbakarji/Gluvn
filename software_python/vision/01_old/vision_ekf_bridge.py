"""External bridge to existing Gluvn telemetry; never performs EKF updates."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import time

from core.port_read import Reader


@dataclass(frozen=True)
class EkfTelemetry:
    host_timestamp: float  # PC receipt time, seconds since epoch
    parser_timestamp: float | None
    device_us: int
    device_time_s: float
    position_m: tuple[float, float, float] | None
    quaternion_wxyz: tuple[float, float, float, float] | None
    nav_armed: bool | None


class VisionEkfBridge:
    """Read NAV_QUAT and MOTION through the project's existing Reader."""

    def __init__(self, hand: str = "r", use_ble: bool = True, force_ble: bool = False):
        self.hand = hand
        self.reader = Reader(sensor_config={hand: {"flex": False, "press": False, "imu": True}},
                             use_ble=use_ble, force_ble=force_ble)
        self.queue = self.reader.threads[hand]["parser"].getQ()
        self._last_device_us: int | None = None
        self._device_wraps = 0
        self.offset_samples: list[float] = []

    def start(self) -> None:
        self.reader.start_readers()
        self.reader.ensure_stream_enabled(self.hand, "NAV_QUAT_STREAM_ON", "nav_quat")
        self.reader.ensure_stream_enabled(self.hand, "MOTION_STREAM_ON", "position")

    def stop(self) -> None:
        self.reader.stop_readers()

    def latest(self) -> EkfTelemetry | None:
        sample: dict[str, Any] | None = None
        while True:
            try:
                sample = self.queue.get_nowait()
            except Exception:
                break
        if sample is None:
            return None
        received_at = time.time()
        device_us = int(sample["device_us"])
        if self._last_device_us is not None and device_us < self._last_device_us:
            self._device_wraps += 1
        self._last_device_us = device_us
        device_time_s = (device_us + self._device_wraps * 2**32) / 1e6
        self.offset_samples.append(received_at - device_time_s)
        return EkfTelemetry(
            host_timestamp=received_at,
            parser_timestamp=sample.get("time"),
            device_us=device_us,
            device_time_s=device_time_s,
            position_m=sample.get("position"),
            quaternion_wxyz=sample.get("nav_quat"),
            nav_armed=sample.get("nav_armed"),
        )

    def estimated_pc_minus_device_offset_s(self) -> float | None:
        """Return the observed PC/device offset, not a synchronization claim."""
        if not self.offset_samples:
            return None
        return self.offset_samples[-1]
