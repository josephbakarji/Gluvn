"""Opt-in research framing, bounded queues and readiness; no hardware imports."""
from collections import Counter, deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import queue
import struct
import threading
import time

SPEC_PATH = Path(__file__).resolve().parents[1] / "aaai" / "capture_contract_v2.json"
SPEC = json.loads(SPEC_PATH.read_text(encoding="utf-8"))
FIELDS = SPEC["sample_fields"]
SAMPLE = struct.Struct(">" + "".join(str(f["count"]) + f["code"] for f in FIELDS))
CONTROL = struct.Struct(">BBBIHHI")
RAW_HEADER = struct.Struct(">QQQI")
STATUS = {k: 1 << v for k, v in SPEC["status_bits"].items()}
SYNC = b"\xa6\x5a"


class CaptureError(RuntimeError):
    pass


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xffff if crc & 0x8000 else (crc << 1) & 0xffff
    return crc


def envelope(payload, sync=SYNC):
    if not 1 <= len(payload) <= 255:
        raise ValueError("payload must contain 1..255 bytes")
    return sync + bytes([len(payload)]) + payload + struct.pack(">H", crc16(payload))


def encode_sample(frame):
    values = []
    for f in FIELDS:
        value = frame[f["name"]]
        values.extend([value] if f["count"] == 1 else value)
    return envelope(SAMPLE.pack(*values))


def control_packets(kind, hand, message_id, device_us, data):
    raw = json.dumps(data, separators=(",", ":"), sort_keys=True, allow_nan=False, ensure_ascii=False).encode("utf-8")
    if not 1 <= len(raw) <= 65536:
        raise ValueError("control JSON exceeds 64 KiB")
    parts = (len(raw) + 239) // 240
    return [envelope(CONTROL.pack(2, kind, ord(hand), message_id, i, parts, device_us) + raw[i*240:(i+1)*240])
            for i in range(parts)]


@dataclass(frozen=True)
class InputChunk:
    data: bytes
    chunk_id: int
    transport_rx_ns: int
    transport_enqueue_ns: int


class Ingress:
    """Observe original bytes before parser-queue loss; latch failures outside that queue."""
    def __init__(self, target, observer=None):
        self.target, self.observer = target, observer
        self.counts, self.error, self.next_id = Counter(), None, 0

    def fail(self, reason):
        if self.error is None:
            self.error = str(reason)

    def receive(self, data, rx_ns=None):
        rx_ns = time.perf_counter_ns() if rx_ns is None else rx_ns
        data = bytes(data)
        for offset in range(0, len(data), 4096):
            chunk = InputChunk(data[offset:offset+4096], self.next_id, rx_ns, time.perf_counter_ns())
            self.next_id += 1
            self.counts["attempted"] += 1
            self.counts["bytes"] += len(chunk.data)
            self.counts["splits"] += int(offset > 0)
            try:
                if self.observer is not None:
                    self.observer(chunk)
            except Exception as exc:
                self.fail(f"raw logger: {exc}")
                self.counts["raw_rejected"] += 1
            try:
                self.target.put_nowait(chunk)
                self.counts["enqueued"] += 1
                self.counts["high_chunks"] = max(self.counts["high_chunks"],self.target.qsize())
            except queue.Full:
                self.counts["rejected"] += 1
                self.counts["rejected_bytes"] += len(chunk.data)
                self.fail("raw parser queue overflow")


class ByteQueue:
    """Bound serialized payload bytes as well as record count; reject rather than evict."""
    def __init__(self, max_records=1024, max_bytes=2*1024**2):
        self.max_records, self.max_bytes = max_records, max_bytes
        self._q, self._bytes = deque(), 0
        self._cv = threading.Condition()
        self.counts = Counter(attempted=0, enqueued=0, rejected=0, consumed=0, high_records=0, high_bytes=0)

    def put_nowait(self, item):
        item = bytes(item)
        with self._cv:
            self.counts["attempted"] += 1
            if len(self._q) >= self.max_records or self._bytes + len(item) > self.max_bytes:
                self.counts["rejected"] += 1
                raise queue.Full("capture byte/record cap exceeded")
            self._q.append(item)
            self._bytes += len(item)
            self.counts["enqueued"] += 1
            self.counts["high_records"] = max(self.counts["high_records"], len(self._q))
            self.counts["high_bytes"] = max(self.counts["high_bytes"], self._bytes)
            self._cv.notify()

    def get(self, timeout=.1):
        with self._cv:
            if not self._cv.wait_for(lambda: bool(self._q), timeout):
                raise queue.Empty
            item = self._q.popleft()
            self._bytes -= len(item)
            self.counts["consumed"] += 1
            return item

    def snapshot(self):
        with self._cv:
            return dict(self.counts, pending=len(self._q), pending_bytes=self._bytes)


class Decoder:
    """Bounded mixed v1/v2 framing; preserve receipt provenance across split/coalesced chunks."""
    def __init__(self, hand, clock=time.monotonic):
        self.hand, self.clock = hand, clock
        self.buf, self.spans, self.pending = bytearray(), deque(), {}
        self.counts = Counter(bytes_in=0, accepted_bytes=0, discarded_bytes=0, crc_errors=0, packets=0)
        self.last_crc_error = None

    def _consume(self, n, discarded=False):
        del self.buf[:n]
        self.counts["discarded_bytes" if discarded else "accepted_bytes"] += n
        while n:
            length, chunk = self.spans[0]
            if length <= n:
                self.spans.popleft()
                n -= length
            else:
                self.spans[0] = (length - n, chunk)
                n = 0

    def expire(self):
        if any(self.clock() - entry["started"] > 3 for entry in self.pending.values()):
            self.counts["control_timeouts"] += 1
            self.pending.clear()
            raise CaptureError("control fragment reassembly timed out")

    def feed(self, chunk, parser_dequeue_ns=None):
        self.expire()
        if len(chunk.data) > 4096 or len(self.buf) + len(chunk.data) > 8192:
            raise CaptureError("parser input/buffer cap exceeded")
        if not chunk.data:
            return []
        self.buf.extend(chunk.data)
        self.spans.append((len(chunk.data), chunk))
        self.counts["bytes_in"] += len(chunk.data)
        out = []
        while len(self.buf) >= 2:
            positions = [i for sync in (SYNC, b"\xa5\x5a") if (i := self.buf.find(sync)) >= 0]
            if not positions:
                self._consume(len(self.buf)-1, True)
                break
            if min(positions):
                self._consume(min(positions), True)
            if len(self.buf) < 3:
                break
            total = self.buf[2] + 5
            if len(self.buf) < total:
                break
            packet = bytes(self.buf[:total])
            payload = packet[3:-2]
            if len(payload) < 2 or crc16(payload) != int.from_bytes(packet[-2:], "big"):
                self.counts["crc_errors"] += 1
                self.last_crc_error = dict(
                    stream_offset=self.counts["accepted_bytes"] + self.counts["discarded_bytes"],
                    first_chunk_id=self.spans[0][1].chunk_id,
                    expected_frame_bytes=total,
                    received_crc=int.from_bytes(packet[-2:], "big"),
                    calculated_crc=crc16(payload))
                self._consume(1, True)
                continue
            used, last = 0, None
            for length, part in self.spans:
                used += length
                last = part
                if used >= total:
                    break
            first = self.spans[0][1]
            timing = dict(first_chunk_id=first.chunk_id, last_chunk_id=last.chunk_id,
                          transport_rx_first_ns=first.transport_rx_ns, transport_rx_last_ns=last.transport_rx_ns,
                          parser_dequeue_ns=parser_dequeue_ns or time.perf_counter_ns(), decode_ns=time.perf_counter_ns())
            self._consume(total)
            self.counts["packets"] += 1
            if packet[:2] != SYNC:
                if payload[0] != ord(self.hand):
                    raise CaptureError("legacy hand mismatch during research startup")
                out.append(dict(record_kind="legacy", packet=packet, **timing))
                continue
            if len(payload) < 3 or payload[0] != 2 or payload[2] != ord(self.hand):
                raise CaptureError("research version or hand mismatch")
            if payload[1] == 1:
                if len(payload) != SAMPLE.size:
                    raise CaptureError(f"v2 sample payload length {len(payload)} != {SAMPLE.size}")
                values, cursor, frame = SAMPLE.unpack(payload), 0, {}
                for f in FIELDS:
                    n = f["count"]
                    frame[f["name"]] = values[cursor] if n == 1 else tuple(values[cursor:cursor+n])
                    cursor += n
                if frame["status"] & ~0xfff:
                    raise CaptureError("unknown v2 status bits")
                if any(x > 4095 for name in ("flex_raw", "press_raw") for x in frame[name]):
                    raise CaptureError("finger ADC counts outside 0..4095")
                frame.update(record_kind="sample", **timing)
                self.sample_count = getattr(self, "sample_count", 0) + 1
                out.append(frame)
            elif payload[1] in (2, 3, 4):
                record = self._control(payload)
                if record is not None:
                    out.append(dict(record, **timing))
            else:
                raise CaptureError("unknown v2 record kind")
        return out

    def _control(self, payload):
        if len(payload) < CONTROL.size:
            raise CaptureError("short control header")
        version, kind, hand, ident, index, count, device_us = CONTROL.unpack_from(payload)
        data = payload[CONTROL.size:]
        if not 1 <= count <= 274 or index >= count or not 1 <= len(data) <= 240 or (index != count-1 and len(data) != 240):
            raise CaptureError("invalid control fragment dimensions")
        key = kind, hand, ident
        if key not in self.pending:
            if sum(k[1] == hand for k in self.pending) >= 4:
                raise CaptureError("control pending-message cap exceeded")
            self.pending[key] = dict(started=self.clock(), count=count, device_us=device_us, parts={})
        entry = self.pending[key]
        if entry["count"] != count or entry["device_us"] != device_us or index in entry["parts"]:
            raise CaptureError("conflicting/duplicate control fragment")
        entry["parts"][index] = data
        if sum(map(len, entry["parts"].values())) > 65536:
            raise CaptureError("control JSON exceeds 64 KiB")
        if len(entry["parts"]) != count:
            return None
        del self.pending[key]
        try:
            def unique(pairs):
                obj = {}
                for k, v in pairs:
                    if k in obj:
                        raise ValueError("duplicate JSON key")
                    obj[k] = v
                return obj
            raw_json = b"".join(entry["parts"][i] for i in range(count)).decode("utf-8")
            obj = json.loads(raw_json,
                             object_pairs_hook=unique, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            if not isinstance(obj, dict):
                raise ValueError("control JSON must be an object")
            if kind == 2 and obj.get("metadata_schema") == 1:
                from .capture_metadata import verify_configuration_hash
                verify_configuration_hash(raw_json)
        except (UnicodeError, ValueError) as exc:
            raise CaptureError(f"invalid control JSON: {exc}") from exc
        return dict(record_kind={2:"metadata", 3:"event", 4:"metrics"}[kind], message_id=ident,
                    device_event_us=device_us, data=obj)

    def snapshot(self):
        result = dict(self.counts, buffered_bytes=len(self.buf), pending_messages=len(self.pending))
        if self.last_crc_error is not None:
            result["last_crc_error"] = dict(self.last_crc_error)
        return result


def metadata_errors(meta, main=True):
    required = ("boot_id", "firmware_elf_sha256", "firmware_source_sha256", "contract_sha256", "config_sha256",
                "calibration_result", "accel_bias_g", "active_gyro_bias_dps", "finger_calibration",
                "sensor", "estimator_config", "capabilities")
    errors = [f"missing metadata {k}" for k in required if k not in meta or meta[k] is None]
    if meta.get("contract_version") != SPEC["contract_version"]:
        errors.append("contract version mismatch")
    if meta.get("calibration_result") != "success":
        errors.append("calibration success not confirmed")
    for k in ("accel_bias_g", "active_gyro_bias_dps"):
        values = meta.get(k, [])
        if not isinstance(values, list) or len(values) != 3 or not all(isinstance(v, (float,int)) and math.isfinite(v) for v in values):
            errors.append(f"invalid {k}")
    if main:
        from .capture_metadata import metadata_detail_errors
        errors.extend(metadata_detail_errors(meta))
        for section in ("sensor", "estimator_config"):
            value = meta.get(section, {})
            if not isinstance(value, dict) or not value.get("complete"):
                errors.append(f"{section} metadata incomplete; main capture blocked")
    for cap in ("six_axis", "reset_events", "calibration_outcome"):
        capabilities=meta.get("capabilities")
        if not isinstance(capabilities,dict) or capabilities.get(cap) is not True:
            errors.append(f"missing capability {cap}")
    for key in ("firmware_elf_sha256", "firmware_source_sha256", "contract_sha256", "config_sha256"):
        value=meta.get(key)
        if not isinstance(value,str) or len(value)!=64 or any(c not in "0123456789abcdef" for c in value.lower()):
            errors.append(f"invalid {key}")
    calibration=meta.get("finger_calibration",{})
    for key in ("min_flex","max_flex","min_press","max_press"):
        value=calibration.get(key) if isinstance(calibration,dict) else None
        if not isinstance(value,list) or len(value)!=5 or any(type(x) is not int or not 0<=x<=4095 for x in value):
            errors.append(f"invalid finger calibration {key}")
    if main and not any(k.startswith("invalid finger calibration") for k in errors):
        # Flex MIN/MAX are straight/bent ADC endpoints, not sorted numerical bounds.
        zero_flex=[i for i,(a,b) in enumerate(zip(calibration['min_flex'],calibration['max_flex'])) if a==b]
        if zero_flex:
            errors.append(f"zero-width flex calibration at channels {zero_flex}; main capture blocked")
        if any(a>=b for a,b in zip(calibration['min_press'],calibration['max_press'])):
            errors.append("zero-width/reversed press calibration; main capture blocked")
    return errors


class Readiness:
    """Fixed liveness checks on device µs; no labels or clustering metrics."""
    def __init__(self, main=True, sensor_only=False):
        if main and sensor_only:
            raise ValueError("sensor-only readiness is diagnostic-only")
        self.sensor_only = sensor_only
        self.main, self.meta, self.last, self.good_start, self.fresh_at = main, None, None, None, None
        self.ready, self.frame_time_us, self.invalid_reason = False, 0, "waiting for metadata"
        # Opt-in (10 Nov, live glove sessions): a freshly powered glove reports calibration_result "unknown" on its
        # first INFO, before the host has sent RECALIBRATE_GYRO. Accept that one condition in metadata, but refuse
        # every sample until metadata with calibration_result "success" has been received. Default unchanged.
        self.allow_uncalibrated = False
        # Opt-in (10 Nov, glove_T02_sequence): over BLE one whole notification can vanish between the glove (tx_sent,
        # 0 rejected/failed) and the host (0 CRC/discarded bytes), so frame_seq jumps by 2. Default 0 refuses that as
        # before. With max_sequence_gap = n, a forward jump of at most n lost frames is counted in sequence_losses
        # (the decoder marks sequence_gap/break_before on the next row) and the capture continues. Restarts, clock
        # errors and larger holes still refuse.
        self.max_sequence_gap = 0
        self.sequence_losses = []

    def metadata(self, meta):
        errors = metadata_errors(meta, self.main)
        if self.allow_uncalibrated:
            errors = [e for e in errors if e != "calibration success not confirmed"]
        if errors:
            raise CaptureError("; ".join(errors))
        if self.meta is not None and meta.get("boot_id") != self.meta.get("boot_id"):
            raise CaptureError("boot/configuration changed during capture")
        if self.meta is not None and meta.get("config_sha256") != self.meta.get("config_sha256"):
            # The active gyro bias is part of config_sha256, so a host-commanded recalibration changes it
            # (glove_T01_two_tuts, 10 Nov: same boot_id, epoch 0 -> 1, hash changed, session aborted). Opt-in:
            # accept exactly that change before the first sample; any other change still refuses.
            recalibrated = (self.allow_uncalibrated and self.last is None and meta.get("calibration_result") == "success"
                            and meta.get("calibration_origin") == "command"
                            and isinstance(meta.get("calibration_epoch"), int)
                            and isinstance(self.meta.get("calibration_epoch"), int)
                            and meta["calibration_epoch"] > self.meta["calibration_epoch"])
            if not recalibrated:
                raise CaptureError("boot/configuration changed during capture")
        self.meta = meta

    def sample(self, frame):
        if self.meta is None:
            raise CaptureError("v2 sample before complete metadata")
        if self.allow_uncalibrated and self.meta.get("calibration_result") != "success":
            raise CaptureError("v2 sample before confirmed gyro calibration")
        s = frame["status"]
        fresh = bool(s & STATUS["imu_fresh"])
        if self.last is None and frame["frame_seq"]!=self.meta.get("frame_seq",0):
            raise CaptureError("first v2 output sequence differs from pre-start metadata")
        if self.last is not None:
            previous = self.last
            delta = (frame["frame_us"] - previous["frame_us"]) & 0xffffffff
            step = (frame["frame_seq"] - previous["frame_seq"]) & 0xffffffff
            # A tolerated hole of n lost frames spans about n+1 frame periods: allow 100 ms per step there.
            limit = 100000 * step if 2 <= step <= self.max_sequence_gap + 1 else 100000
            if delta == 0 or delta >= 2**31 or delta > limit:
                raise CaptureError("invalid frame clock/gap; valid interval closed")
            if step != 1:
                expected = (previous["frame_seq"] + 1) & 0xffffffff
                if not 2 <= step <= self.max_sequence_gap + 1:
                    raise CaptureError(f"output sequence loss/restart: expected {expected}, received {frame['frame_seq']}")
                self.sequence_losses.append(dict(expected=expected, received=int(frame["frame_seq"]),
                                                 lost=int(step - 1), frame_us=int(frame["frame_us"])))
            epochs = ("calibration_epoch",) if self.sensor_only else ("nav_epoch", "calibration_epoch")
            if any(frame[k] != previous[k] for k in epochs):
                if self.ready:
                    raise CaptureError("reset/calibration epoch changed")
                self.good_start = None
            imu_delta = (frame["imu_seq"] - previous["imu_seq"]) & 0xffffffff
            if step != 1:                               # lost frames may have carried IMU reads: 0..step allowed
                if imu_delta > step or (fresh and imu_delta == 0):
                    raise CaptureError("IMU freshness/sequence mismatch")
            elif imu_delta != (1 if fresh else 0):
                raise CaptureError("IMU freshness/sequence mismatch")
            if step == 1 and not fresh and frame["imu_read_us"] != previous["imu_read_us"]:
                raise CaptureError("stale IMU read timestamp changed")
            self.frame_time_us += delta
        self.last = dict(frame)
        if not fresh and (frame["imu_dt_us"] != 0 or s & (STATUS["imu_dt_valid"] | STATUS["mahony_stepped"] | STATUS["nav_update_called"])):
            raise CaptureError("stale frame claims an estimator update")
        if fresh:
            need = ("imu_raw_finite", "imu_dt_valid")
            if not self.sensor_only:
                need += ("mahony_stepped", "nav_update_called")
            if not all(s & STATUS[k] for k in need):
                self.good_start = None
                if self.ready:
                    raise CaptureError("invalid fresh IMU/update during valid interval")
                return False
            floats = [x for name in ("accel_pre_g", "gyro_pre_dps", "accel_input_g", "gyro_input_dps") for x in frame[name]]
            if not all(math.isfinite(x) for x in floats) or not 0 < frame["imu_dt_us"] <= 500000:
                raise CaptureError("finite/dt flags disagree with six-axis values")
            self.fresh_at = self.frame_time_us
        if self.fresh_at is None and self.frame_time_us <= 100000:
            return False
        if self.fresh_at is None or self.frame_time_us - self.fresh_at > 100000:
            raise CaptureError("fresh IMU gap exceeds 100 ms")
        valid = True
        if not self.sensor_only:
            need = ("nav_armed", "mahony_output_finite", "nav_output_finite")
            valid = all(s & STATUS[k] for k in need)
            for name in ("q_mahony_wxyz", "q_nav_wxyz"):
                valid &= all(math.isfinite(x) for x in frame[name]) and sum(x*x for x in frame[name]) > 0
            valid &= all(math.isfinite(x) for k in ("velocity_nav_mps", "position_nav_m", "lin_accel_nav_mps2") for x in frame[k])
        if not valid:
            self.invalid_reason = "waiting for armed, finite estimator outputs"
            self.good_start = None
            if self.ready:
                raise CaptureError("invalid/unarmed estimator output")
            return False
        if self.good_start is None:
            self.good_start = self.frame_time_us
        self.ready = self.frame_time_us - self.good_start >= 1000000
        self.invalid_reason = "" if self.ready else "collecting 1 s of valid observations"
        return self.ready
