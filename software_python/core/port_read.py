"""
Gluvn-M5 port reader — USB or BLE, auto-selected.

ReadSerial/ReadBLE put raw bytes onto a shared queue; ParseSerial extracts
frames from that queue via sync+len+CRC, independent of which transport
is feeding it.

Offline analysis (CSV save/load, plotting, packet-loss stats) lives in
data_analysis.py, not here — this module only depends on it lazily, inside
Reader.make_reader_threads(), and only when parse_file is actually used.
"""

from __future__ import division
from core.__init__ import (
    BLE_NAME_L, BLE_NAME_R, NUS_TX_CHAR_UUID, NUS_RX_CHAR_UUID,
    portL, portR, baud, EXPDIR
)
from bleak import BleakClient, BleakScanner
from threading import Thread, Event
from struct import unpack, calcsize
import asyncio
import threading
import serial
from serial.tools import list_ports
import queue
import time
import mido
import os


class Reader:
    def __init__(self,
                 directory=EXPDIR,
                 parse_file=None,
                 sensor_config=None,
                 use_ble=True,
                 force_ble=False,
                 ble_timeout=10.0):

        self.directory = directory
        self.baud = baud
        self.sensor_config = sensor_config or {'l': {'flex': True, 'press': True, 'imu': True}}
        self.hands = list(self.sensor_config.keys())
        self.parse_qsize = 30
        self.parse_file = None if parse_file is None else os.path.join(directory, parse_file)
        self.use_ble = use_ble
        self.force_ble = force_ble
        self.ble_timeout = ble_timeout

        self.threads = self.make_reader_threads()
        self.printers = self.make_printer_threads()

    def make_reader_threads(self):
        time0 = time.time()
        threads = {hand: {} for hand in self.hands}
        failed = []

        available_ports = [p.device for p in list_ports.comports()]

        for hand in self.hands:
            if self.parse_file is None:
                shared_q = queue.Queue(maxsize=48)
                port = portR if hand == 'r' else portL

                if port in available_ports and not self.force_ble:
                    print(f"[{hand.upper()}] USB found on {port}, using it.")
                    threads[hand]['port'] = port
                    try:
                        threads[hand]['serial'] = ReadSerial(port, self.baud, sensq=shared_q)
                        threads[hand]['is_ble'] = False
                    except serial.SerialException as e:
                        print(f"Warning: could not open {port}: {e}")
                        failed.append(hand)
                        continue
                else:
                    if self.use_ble:
                        print(f"[{hand.upper()}] No USB port — connecting via BLE.")
                        ble_name = BLE_NAME_R if hand == 'r' else BLE_NAME_L
                        threads[hand]['serial'] = ReadBLE(ble_name, sensq=shared_q)
                        threads[hand]['is_ble'] = True
                    else:
                        print(f"Warning: no USB port for {hand} and BLE is disabled — skipping.")
                        failed.append(hand)
                        continue

                threads[hand]['parser'] = ParseSerial(
                    shared_q, time0,
                    add_timestamp=True,
                    **self.sensor_config[hand],
                    qsize=self.parse_qsize,
                    expected_hand=hand,
                )
            else:
                from core.data_analysis import ParseFile
                threads[hand]['parser'] = ParseFile(self.directory, self.parse_file, hand)

        for hand in failed:
            del threads[hand]
            self.hands.remove(hand)

        if not self.hands:
            raise Exception("No active data streams acquired.")

        return threads

    def ensure_fresh_gyro_calibration(self, hand, timeout=10.0):
        """Block until a fresh gyro-bias measurement is confirmed for `hand`.
        BLE-only -- see start_readers for why USB doesn't need this.

        gyro_calib_toggle (imu1's LSB) flips exactly once when a calibration
        completes, regardless of trigger (boot, RECALIBRATE_GYRO, or the
        on-device button). We wait for it to CHANGE relative to a baseline
        read before sending the command -- not for a stream pause, since a
        pause isn't unique to calibration completing.
        """
        parser_q = self.threads[hand]['parser'].getQ()
        baseline = None
        try:
            baseline = parser_q.get(timeout=1.0)['gyro_calib_toggle']
        except queue.Empty:
            pass  # no frame yet -- first post-command sample becomes the baseline instead

        print(f"[{hand.upper()}] Recalibrating gyro -- keep glove still...")
        self.send_command(hand, "RECALIBRATE_GYRO")

        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                sample = parser_q.get(timeout=0.5)
            except queue.Empty:
                continue   # expected: firmware sends nothing while calibrating
            if baseline is None:
                baseline = sample['gyro_calib_toggle']
                continue
            if sample['gyro_calib_toggle'] != baseline:
                print(f"[{hand.upper()}] Gyro recalibrated.")
                return True

        print(f"Warning: [{hand.upper()}] gyro recalibration not confirmed within {timeout}s -- continuing anyway.")
        return False

    def ensure_stream_enabled(self, hand, cmd, data_key, timeout=3.0, retries=3):
        """Send a stream-enable command (NAV_QUAT_STREAM_ON, MOTION_STREAM_ON)
        and confirm it actually took effect by watching for data_key to
        appear on this hand's decoded queue, resending on a timeout.

        send_command has no ACK -- a dropped BLE write (more likely on a
        weaker/lossier link than the other hand's) previously left that
        hand's stream silently never enabled, with no visible error: the
        command "succeeds" locally (no exception), the firmware just never
        receives or acts on it. This mirrors ensure_fresh_gyro_calibration's
        confirm-before-proceeding approach instead of trusting a one-shot send.
        """
        parser_q = self.threads[hand]['parser'].getQ()
        for attempt in range(1, retries + 1):
            self.send_command(hand, cmd)
            deadline = time.time() + timeout
            while time.time() < deadline:
                try:
                    sample = parser_q.get(timeout=0.5)
                except queue.Empty:
                    continue
                if data_key in sample:
                    return True
            print(f"Warning: [{hand.upper()}] {cmd} not confirmed within {timeout}s "
                  f"(attempt {attempt}/{retries}){' -- retrying' if attempt < retries else ''}.")
        print(f"Warning: [{hand.upper()}] {cmd} never confirmed after {retries} attempts -- "
              f"this hand's corresponding data will be unavailable.")
        return False

    def start_readers(self):
        for hand in self.hands:
            if self.parse_file is not None:
                self.threads[hand]['parser'].start()
                continue

            self.threads[hand]['serial'].start()

            if self.threads[hand].get('is_ble', False):
                print(f"[{hand.upper()}] Connecting via BLE...")
                if self.threads[hand]['serial'].wait_connected(timeout=self.ble_timeout):
                    print(f"[{hand.upper()}] Connected.")
                    # Parser must already be running so gyro_calib_toggle is
                    # visible on the decoded queue -- see ensure_fresh_gyro_calibration.
                    self.threads[hand]['parser'].start()
                    self.ensure_fresh_gyro_calibration(hand)
                else:
                    print(f"Warning: [{hand.upper()}] BLE connection timed out.")
                    self.threads[hand]['parser'].start()
            else:
                self.threads[hand]['serial'].send_handshake()
                print(f"[{hand.upper()}] Connected.")
                self.threads[hand]['parser'].start()

    def stop_readers(self):
        for hand in self.hands:
            if self.parse_file is None:
                try:
                    self.threads[hand]['serial'].stop()
                except Exception:
                    pass
        self.report_hand_integrity()

    def report_hand_integrity(self):
        """Surface each parser's hand_byte_seen (see ParseSerial.parse) --
        more than one distinct value on a single hand's parser means frames
        from the other glove (or garbage) landed on this queue during the
        session, which is otherwise invisible."""
        for hand in self.hands:
            parser = self.threads[hand].get('parser')
            seen = getattr(parser, 'hand_byte_seen', None)
            if seen and len(seen) > 1:
                print(f"Warning: [{hand.upper()}] saw {len(seen)} distinct hand-byte "
                      f"values on its own queue this session: {sorted(seen)} -- "
                      f"this indicates cross-hand contamination upstream of the parser.")

    def make_printer_threads(self):
        return {hand: printSens(self.threads[hand]['parser'].getQ()) for hand in self.hands}

    def start_printers(self):
        for hand in self.hands:
            self.printers[hand].start()

    def stop_printers(self):
        for hand in self.hands:
            self.printers[hand].join(timeout=1)

    def run_sensors(self):
        self.start_readers()
        input('Running — press Enter to stop.\n')
        print('Stopping...')
        self.stop_readers()

    def send_command(self, hand, cmd):
        try:
            self.threads[hand]['serial'].send(f'{cmd}\n')
        except Exception as e:
            print(f"Warning: failed to send command to {hand}: {e}")


class ReadBLE(Thread):
    def __init__(self, device_name, sensq=None, qsize=48):
        Thread.__init__(self)
        self.daemon = True
        self.device_name = device_name
        self.sensq = sensq if sensq is not None else queue.Queue(maxsize=qsize)  # own queue if none given
        self._stop_flag = threading.Event()
        self._connected_event = threading.Event()
        self._loop = asyncio.new_event_loop()
        self._client = None
        self._write_q = asyncio.Queue()

    def run(self):
        self._loop.run_until_complete(self._ble_loop())

    def stop(self):
        self._stop_flag.set()

    def getQ(self):
        return self.sensq

    def wait_connected(self, timeout=20.0):
        return self._connected_event.wait(timeout)

    def send(self, text):
        asyncio.run_coroutine_threadsafe(self._write_q.put(text), self._loop)

    def _notification_handler(self, sender, data: bytearray):
        try:
            self.sensq.put_nowait(bytes(data))
        except queue.Full:
            try:
                self.sensq.get_nowait()
            except queue.Empty:
                pass
            try:
                self.sensq.put_nowait(bytes(data))
            except queue.Full:
                pass

    async def _ble_loop(self):
        while not self._stop_flag.is_set():
            try:
                device = await BleakScanner.find_device_by_name(self.device_name, timeout=5.0)
                if device is None:
                    print(f"BLE: no advertisement seen for '{self.device_name}' in 5s, retrying...")
                    await asyncio.sleep(1)
                    continue

                async with BleakClient(device) as client:
                    self._client = client
                    self._connected_event.set()

                    await client.start_notify(NUS_TX_CHAR_UUID, self._notification_handler)

                    while not self._stop_flag.is_set() and client.is_connected:
                        try:
                            cmd = await asyncio.wait_for(self._write_q.get(), timeout=0.1)
                            await client.write_gatt_char(
                                NUS_RX_CHAR_UUID,
                                cmd.encode() if isinstance(cmd, str) else cmd,
                                response=False
                            )
                        except asyncio.TimeoutError:
                            pass

                    self._connected_event.clear()

            except Exception as e:
                print(f"BLE: connection attempt failed ({type(e).__name__}: {e}), retrying in 2s...")
                self._connected_event.clear()
                await asyncio.sleep(2)

        self._client = None


class ReadSerial(Thread):
    def __init__(self, port, baud, sensq=None, qsize=48):
        Thread.__init__(self)
        self.daemon = True
        self._stop_event = Event()
        self.port = port
        self.baud = baud
        self.sensq = sensq if sensq is not None else queue.Queue(maxsize=qsize)

        self.serial_port = serial.Serial()
        self.serial_port.port = self.port
        self.serial_port.baudrate = self.baud
        self.serial_port.timeout = 2
        self.serial_port.dtr = False
        self.serial_port.rts = False
        self.serial_port.open()
        self._hardware_reset()

    def _hardware_reset(self):
        """Toggle DTR/RTS to force an ESP32 hardware reset, equivalent to unplug/replug."""
        try:
            self.serial_port.dtr = False
            self.serial_port.rts = True
            time.sleep(0.12)  # ESP32 bootloader requires >100ms pulse
            self.serial_port.rts = False
            time.sleep(0.5)  # allow boot + BLE stack init to complete
        except Exception as e:
            print(f"Warning: hardware reset failed: {e}")

    def send_handshake(self, delay=1.0):
        """Send NORMAL_MODE after the read thread is already running."""
        time.sleep(delay)
        try:
            self.serial_port.write(b'NORMAL_MODE\n')
            time.sleep(0.3)
            self.serial_port.reset_input_buffer()
            print(f"[{self.port}] Firmware streaming state initialized.")
        except Exception as e:
            print(f"Warning: Handshake failed on {self.port}: {e}")

    def run(self):
        while not self._stop_event.is_set():
            self.read()

    def read(self):
        try:
            n = self.serial_port.in_waiting or 1
            temp = self.serial_port.read(n)
            if temp:
                try:
                    self.sensq.put(temp, timeout=0.5)
                except queue.Full:
                    pass
        except Exception as e:
            print(f"Warning: serial read failed on {self.port}: {e}")
            time.sleep(0.12)

    def stop(self):
        self._stop_event.set()
        try:
            self.serial_port.cancel_read()
        except Exception:
            pass
        try:
            self.serial_port.close()
        except Exception:
            pass

    def send(self, text):
        try:
            self.serial_port.write(text.encode())
        except Exception as e:
            print(f"Warning: failed to write to {self.port}: {e}")

    def getQ(self):
        return self.sensq

    def getQBlock(self):
        return self.sensq.get(block=True)


class ParseSerial(Thread):
    """
    Parses the firmware's [0xA5][0x5A][len][payload][crc16_hi][crc16_lo] frames.

    Dispatch is FLAGS-FIRST, not length-first. payload[1] (right after hand)
    is an always-present stream-flags byte: bit0=calibration_mode,
    bit1=send_quat, bit2=send_nav_quat, bit3=send_motion. Length alone can't
    identify which optional blocks are present -- quat and nav_quat are both
    8 bytes, and combinations of (normal/cal) x (quat/nav_quat/motion) can
    collide on total length. Decode order: read flags -> derive which blocks
    are present -> build the exact struct format -> unpack. Length is still
    checked against what flags imply, as an integrity cross-check only.

    len is a multiple of one of: 29 (normal, 8-bit calibrated sensors) or 39
    (calibration, raw 16-bit sensors) -- both include the flags byte. A
    multiple >1x means firmware-side bundling (BUNDLE_FRAMES>1): several
    sample payloads concatenated under one sync/len/CRC, trading per-sample
    latency for fewer notifications. lcm(29,39)=1131>255, so a single uint8
    length byte can never match both unit sizes -- flags settles the rest.
    Both modes unpack to the same 20-field base tuple (hand, flags, seq,
    device_us, yaw, pitch, roll, imu0-2, flex0-4, fsr0-4), so the index
    slices below are mode-independent regardless of bundle depth.

    sensor_type lives in imu0's LSB (0=accel,1=gyro), not a standalone byte
    -- decoded as `unp[7] & 1`, costs ~1 LSB of imu0 resolution (well under
    the noise floor). Same technique, imu1's LSB: gyro_calib_toggle flips
    once per completed gyro calibration (boot or RECALIBRATE_GYRO). It is
    NOT "calibration in progress" -- the firmware sends nothing at all while
    calibrating, so a host confirms completion by watching this bit CHANGE
    relative to a baseline read before requesting RECALIBRATE_GYRO, rather
    than inferring completion from stream silence.

    device_us (unp[3]) is the firmware's micros() at that sample's deadline
    -- independent of USB/BLE arrival timing, so diff(device_us) gives the
    true device-side sample interval. Each sample in a bundle carries its
    own device_us/seq.

    NavEKF health (flags bits 4-6, plus zupt_mahalanobis in the motion
    block) is decoded ONLY when has_motion is true -- NavEKF may not even be
    running otherwise. Check for the KEY's presence ('nav_armed' in sample),
    not a default value.
      - nav_armed: hard-gate. False means velocity/position/lin_accel are
        not yet trustworthy (e.g. the ~0.3s re-arm window after
        MOTION_STREAM_ON activates). Treat False as "no data", not
        "degraded data".
      - nav_zupt_active: a ZUPT correction ran this cycle (point-in-time).
      - nav_zupt_starved: no successful ZUPT in >5s (NavEKF.h's
        zuptStarvationThresholdSec_) -- position/velocity error likely
        accumulating unchecked.
      - zupt_mahalanobis: continuous companion to nav_zupt_starved. NavEKF's
        chi-square (3-DoF) goodness-of-fit for the last ZUPT candidate --
        near 0 = confident stationary fit. NavEKF's own gate is 16.0; wire
        encoding clamps at 64.0 (ZUPT_MAHALANOBIS_CAP). None if NavEKF
        hasn't evaluated a ZUPT candidate yet this session.

    format / length_checksum are accepted for backward-compatible call
    signatures but are no longer used — the frame is now self-describing.
    """
    FRAME_SYNC = b'\xA5\x5A'
    QUAT_SUFFIX     = 'hhhh'     # qw,qx,qy,qz
    NAV_QUAT_SUFFIX = 'hhhh'     # nqw,nqx,nqy,nqz
    MOTION_SUFFIX   = 'hhhhhhhhhB'  # velX,velY,velZ,posX,posY,posZ,laX,laY,laZ,zupt_quality

    # flags bit positions (must match m5stick_firmware.ino's buildPayload)
    FLAG_CALIBRATION_MODE = 0x01
    FLAG_SEND_QUAT        = 0x02
    FLAG_SEND_NAV_QUAT    = 0x04
    FLAG_SEND_MOTION      = 0x08
    # Bits 4-6: NavEKF health -- meaningful ONLY when FLAG_SEND_MOTION is
    # also set. Decoded into is_armed/is_zupt_active/is_zupt_starved, but
    # only when has_motion.
    FLAG_ARMED         = 0x10
    FLAG_ZUPT_ACTIVE   = 0x20
    FLAG_ZUPT_STARVED  = 0x40
    FLAG_LOW_BATTERY   = 0x80

    ZUPT_QUALITY_SENTINEL = 255   # "never computed yet" -- NavEKF's own -1.0f
    ZUPT_MAHALANOBIS_CAP = 64.0   # must match firmware (4x NavEKF.h's zuptGateChiSq_ of 16.0)

    # Base struct includes the flags byte (2nd field, right after hand).
    _BASE_SAMPLE = '>BBBIHHHHHHBBBBBBBBBB'
    _BASE_CAL    = '>BBBIHHHHHHHHHHHHHHHH'
    SAMPLE_UNIT_LEN = calcsize(_BASE_SAMPLE)   # 29
    CAL_UNIT_LEN    = calcsize(_BASE_CAL)      # 39

    QUAT_SCALE = 32767.0
    MOTION_VEL_RANGE_MPS = 8.0
    MOTION_POS_RANGE_M   = 2.0        # must match firmware's MOTION_POS_RANGE_M
    LIN_ACCEL_RANGE_MPS2 = 4.0 * 9.80665  # must match firmware's LIN_ACCEL_RANGE_MPS2 (4g)

    # unp[0] ('hand')'s wire encoding isn't documented anywhere -- see
    # hand_byte_seen below for why it's tracked as a diagnostic instead of enforced.

    def __init__(self, sensq, time0, format=None,
                 length_checksum=None, add_timestamp=False,
                 flex=False, press=True, imu=False, qsize=30,
                 nav_quat_enabled=False, expected_hand=None):
        Thread.__init__(self)
        self.daemon = True
        self.sensq = sensq
        self.time0 = time0
        self.dataq = queue.Queue(maxsize=qsize)
        self.read_configs = {'flex': flex, 'press': press, 'imu': imu}
        self.nav_quat_enabled = nav_quat_enabled   # retained for API compatibility; flags
                                                    # now disambiguates quat vs nav_quat directly

        self.add_timestamp = add_timestamp

        # Which hand this parser instance is meant to decode ('l'/'r'), if
        # known -- purely descriptive today (see hand_byte_seen), kept as a
        # constructor arg for a future active cross-check.
        self.expected_hand = expected_hand
        # Every distinct payload[0] value observed on this parser's queue.
        # Each hand has its own connection/parser, so this should only ever
        # contain ONE value for the run's lifetime -- more than one is hard
        # proof frames from the other glove (or garbage) landed here.
        self.hand_byte_seen = set()

        self.slices = {
            # unp indices (base fields, before any optional suffix):
            # 0=hand,1=flags,2=seq,3=device_us,4=yaw,5=pitch,6=roll,
            # 7=imu0,8=imu1,9=imu2,10-14=flex,15-19=press.
            'imu': slice(4, 10),
            'flex': slice(10, 15),
            'press': slice(15, 20),
        }
        self._buf = bytearray()

        # Cheap running counters for ad-hoc inspection (e.g. parser.crc_errors
        # in a REPL) — not tied to any firmware-side counter.
        self.crc_errors = 0
        self.frames_ok = 0
        self.flags_mismatch_errors = 0   # length didn't match what flags implied

    def _format_for_flags(self, flags):
        """Build the exact struct format + expected unit length for one
        sample's flags byte. Returns (format_str, unit_len, has_motion)."""
        is_cal = bool(flags & self.FLAG_CALIBRATION_MODE)
        has_quat = bool(flags & self.FLAG_SEND_QUAT)
        has_nav_quat = bool(flags & self.FLAG_SEND_NAV_QUAT)
        has_motion = bool(flags & self.FLAG_SEND_MOTION)

        fmt = self._BASE_CAL if is_cal else self._BASE_SAMPLE
        if has_quat:
            fmt += self.QUAT_SUFFIX
        if has_nav_quat:
            fmt += self.NAV_QUAT_SUFFIX
        if has_motion:
            fmt += self.MOTION_SUFFIX
        return fmt, calcsize(fmt), has_quat, has_nav_quat, has_motion

    def run(self):
        while True:
            self.parse()

    @staticmethod
    def _crc16_ccitt(data, crc=0xFFFF):
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
        return crc

    def parse(self):
        chunk = self.sensq.get(block=True)
        self._buf += chunk

        while True:
            sync_idx = self._buf.find(self.FRAME_SYNC)

            if sync_idx < 0:
                # No sync in buffer. Keep the last byte only, in case it's
                # the first half of a sync word split across chunk reads.
                if len(self._buf) > 1:
                    del self._buf[:-1]
                return

            if sync_idx > 0:
                del self._buf[:sync_idx]   # drop leading noise (ASCII acks, boot banner, etc.)

            if len(self._buf) < 3:
                return   # need the length byte yet

            length = self._buf[2]
            frame_total = 3 + length + 2   # header + payload + crc16

            if len(self._buf) < frame_total:
                return   # wait for the rest of the frame to arrive

            payload = bytes(self._buf[3:3 + length])
            crc_rx = (self._buf[3 + length] << 8) | self._buf[3 + length + 1]

            if self._crc16_ccitt(payload) != crc_rx or length < 2:
                self.crc_errors += 1
                del self._buf[0]
                continue

            # CRC only proves the bytes weren't corrupted in transit, not
            # that they arrived on the RIGHT queue. payload[0]'s wire
            # encoding (which integer means 'l' vs 'r') isn't documented,
            # so it can't be cross-checked and dropped here without risking
            # silent loss on a wrong guess. Tracked as a diagnostic instead:
            # more than one distinct value on a single hand's parser is hard
            # evidence of cross-hand contamination upstream (BLE routing,
            # stale client reference on reconnect, etc). Check
            # parser.hand_byte_seen after a two-glove session.
            self.hand_byte_seen.add(payload[0])

            # Peek the flags byte (payload[1]) to determine this bundle's
            # unit format without relying on length -- every sample in one
            # bundle shares the same stream state, so one peek covers the frame.
            flags = payload[1]
            fmt, unit_len, has_quat, has_nav_quat, has_motion = self._format_for_flags(flags)

            if unit_len == 0 or length % unit_len != 0:
                # Length doesn't divide evenly by what flags imply -- frame
                # is corrupt or flags/length disagree. Don't guess; drop it.
                self.flags_mismatch_errors += 1
                del self._buf[0]
                continue

            n_samples = length // unit_len
            for i in range(n_samples):
                sub = payload[i * unit_len:(i + 1) * unit_len]
                unp = unpack(fmt, sub)
                data_to_put = {
                    'seq': unp[2],
                    'device_us': unp[3],
                    'sensor_type': unp[7] & 1,
                    'gyro_calib_toggle': unp[8] & 1,
                    'calibration_mode': bool(flags & self.FLAG_CALIBRATION_MODE),
                    'low_battery': bool(flags & self.FLAG_LOW_BATTERY),
                }

                # Optional blocks always appear in this fixed order on the
                # wire: quat, then nav_quat, then motion (vel+pos+lin_accel).
                # Since presence is known from flags (not guessed from
                # length), start offsets are unambiguous.
                cursor = 20   # first index after the 20-field base tuple
                if has_quat:
                    data_to_put['quat'] = tuple(v / self.QUAT_SCALE for v in unp[cursor:cursor + 4])
                    cursor += 4
                if has_nav_quat:
                    data_to_put['nav_quat'] = tuple(v / self.QUAT_SCALE for v in unp[cursor:cursor + 4])
                    cursor += 4
                if has_motion:
                    vx, vy, vz, px, py, pz, lax, lay, laz, zupt_quality_raw = unp[cursor:cursor + 10]
                    data_to_put['velocity'] = (
                        vx / self.QUAT_SCALE * self.MOTION_VEL_RANGE_MPS,
                        vy / self.QUAT_SCALE * self.MOTION_VEL_RANGE_MPS,
                        vz / self.QUAT_SCALE * self.MOTION_VEL_RANGE_MPS,
                    )
                    data_to_put['position'] = (
                        px / self.QUAT_SCALE * self.MOTION_POS_RANGE_M,
                        py / self.QUAT_SCALE * self.MOTION_POS_RANGE_M,
                        pz / self.QUAT_SCALE * self.MOTION_POS_RANGE_M,
                    )
                    data_to_put['lin_accel'] = (
                        lax / self.QUAT_SCALE * self.LIN_ACCEL_RANGE_MPS2,
                        lay / self.QUAT_SCALE * self.LIN_ACCEL_RANGE_MPS2,
                        laz / self.QUAT_SCALE * self.LIN_ACCEL_RANGE_MPS2,
                    )
                    # None when NavEKF hasn't evaluated a ZUPT candidate yet
                    # (firmware's -1.0f sentinel, wire value 255). Otherwise
                    # a chi-square goodness-of-fit distance, near 0 =
                    # confident stationary fit; NavEKF's own gate is 16.0,
                    # wire clamps at ZUPT_MAHALANOBIS_CAP (64.0).
                    if zupt_quality_raw == self.ZUPT_QUALITY_SENTINEL:
                        data_to_put['zupt_mahalanobis'] = None
                    else:
                        data_to_put['zupt_mahalanobis'] = (
                            zupt_quality_raw / 254.0 * self.ZUPT_MAHALANOBIS_CAP
                        )
                    # NavEKF health bits (flags bits 4-6) -- meaningful only
                    # here; NavEKF may not even be running when has_motion is False.
                    data_to_put['nav_armed'] = bool(flags & self.FLAG_ARMED)
                    data_to_put['nav_zupt_active'] = bool(flags & self.FLAG_ZUPT_ACTIVE)
                    data_to_put['nav_zupt_starved'] = bool(flags & self.FLAG_ZUPT_STARVED)
                    cursor += 10

                for key, active in self.read_configs.items():
                    if active:
                        data_to_put[key] = unp[self.slices[key]]

                if self.add_timestamp:
                    data_to_put['time'] = time.time() - self.time0

                try:
                    self.dataq.put(data_to_put, timeout=0.1)
                except queue.Full:
                    pass

                self.frames_ok += 1

            del self._buf[:frame_total]

    def getQ(self):
        return self.dataq


class ReadKeyboard(Thread):
    def __init__(self, portname, time0):
        Thread.__init__(self)
        self.portname = portname
        self.dataq = queue.Queue(maxsize=0)
        self.daemon = True
        self.time0 = time0

    def run(self):
        inport = mido.open_input(self.portname)
        while True:
            msg = inport.receive(block=True)
            self.dataq.put([time.time() - self.time0, msg])

    def getQ(self):
        return self.dataq


class printSens(Thread):
    def __init__(self, sensorq, info=''):
        Thread.__init__(self)
        self.sensorq = sensorq
        self.info = info
        self.daemon = True

    def run(self):
        while True:
            print(self.info, self.sensorq.get(block=True))