"""
app_jacob_choir.py — three-note choir instrument.

Trigger topology ( multi-sensor via MultiSensorProcess):
  - lin_accel burst  -> retrigger up to 3 notes (yaw-sector selected, per hand)
  - pitch tilt       -> step the sounding note up/down within the scale
  - press 5         -> all-notes-off
  - press 4         -> volume delta from pitch rate-of-change (aftertouch)
"""

import sys
import time
import asyncio
import numpy as np
from collections import deque
from bleak import BleakScanner

from core.__init__ import BLE_NAME_L, BLE_NAME_R
from core.note_mapper import NoteMapper
from core.port_read import Reader
from core.midi_writer import MidiWriter
from threading import Thread
import queue

from core.base_config import TWO_BYTE, BYTE, ZERO_ACCEL   # single source of truth for hardware calibration
from ui.digital_twin import build_app_and_window
import threading


class LiveControls:
    """Thread-safe bridge for settings the digital twin UI can change while
    ChoirMovingWindow.run() is already looping in its own thread. One
    instance is shared between both; the choir thread calls the getters
    once per loop iteration (never caches a value across iterations), the
    Qt thread calls the setters from its widget callbacks."""

    def __init__(self, vibrato_enabled=True, thresholds=None):
        self._lock = threading.Lock()
        self._vibrato_enabled = vibrato_enabled
        self._thresholds = dict(thresholds or {})
        self._vibrato_reset_pending = False

    def get_vibrato_enabled(self):
        with self._lock:
            return self._vibrato_enabled

    def set_vibrato_enabled(self, enabled):
        with self._lock:
            if self._vibrato_enabled and not enabled:
                self._vibrato_reset_pending = True
            self._vibrato_enabled = bool(enabled)

    def pop_vibrato_reset_pending(self):
        with self._lock:
            pending = self._vibrato_reset_pending
            self._vibrato_reset_pending = False
            return pending

    def get_threshold(self, key, default=None):
        with self._lock:
            return self._thresholds.get(key, default)

    def set_threshold(self, key, value):
        with self._lock:
            self._thresholds[key] = value

    def get_all_thresholds(self):
        with self._lock:
            return dict(self._thresholds)


def nav_quat_to_euler_deg(qw, qx, qy, qz):
    """Yaw/pitch/roll (degrees) from a unit quaternion, using the SAME
    rotation-matrix convention as MahonyAHRS.h's computeEuler() -- verified
    element-for-element identical to NavEKF.h's quatToDCM(), so this applies
    unchanged to nav_quat. Returns (yaw, pitch, roll) to match the existing
    imu[0:3] ordering used elsewhere in this file.

    yaw = atan2f(R10, R00): +/-180 deg, true wraparound.
    pitch = -asinf(R20): +/-90 deg, REFLECTS at the endpoint (no true wrap).
    roll = atan2f(R21, R22): +/-180 deg, true wraparound.
    (Naming matches MahonyAHRS.h exactly.)
    """
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r10 = 2.0 * (qx * qy + qw * qz)
    r20 = 2.0 * (qx * qz - qw * qy)
    r21 = 2.0 * (qy * qz + qw * qx)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

    s = min(max(r20, -1.0), 1.0)
    yaw = np.degrees(np.arctan2(r10, r00))
    pitch = -np.degrees(np.arcsin(s))
    roll = np.degrees(np.arctan2(r21, r22))
    return yaw, pitch, roll


def scan_available_ble(timeout=5.0):
    async def _scan():
        found = set()
        devices = await BleakScanner.discover(timeout=timeout)
        for d in devices:
            if d.name == BLE_NAME_R: found.add('r')
            elif d.name == BLE_NAME_L: found.add('l')
        return found
    return asyncio.run(_scan())


class MultiSensorProcess(Thread):
    """
    Per-hand trigger thread supporting >=1 simultaneous trigger sensor types
    (e.g. ['press','flex']), each with independent threshold/hysteresis and
    persistent turn_state — required by ChoirMovingWindow's simultaneous
    flex+press event consumption.
    """
    def __init__(self, hand, sensor_q, collect_q,
                 trigger_sensor=('flex',),
                 trigger_thresh=None,
                 trigger_hysteresis=None,
                 mod_sensors=None, mod_idx=None, extra_q=None,
                 send_all_data=True):
        super().__init__()
        self.daemon = True
        self.hand = hand
        self.sensor_q = sensor_q
        self.collect_q = collect_q
        self.trigger_sensors = list(trigger_sensor)
        self.mod_sensors = mod_sensors
        self.mod_idx = mod_idx
        self.extra_q = extra_q
        self.send_all_data = send_all_data

        trigger_thresh = trigger_thresh or {}
        trigger_hysteresis = trigger_hysteresis or {}
        self.trigger_thresh, self.trigger_hysteresis = {}, {}
        self.turn_state, self.trig_on, self.trig_off, self.switch = {}, {}, {}, {}
        for s in self.trigger_sensors:
            self.trigger_thresh[s] = trigger_thresh[s]
            self.trigger_hysteresis[s] = trigger_hysteresis[s]
            self.turn_state[s] = np.zeros(5, dtype=int)
            self.trig_on[s] = np.zeros(5, dtype=bool)
            self.trig_off[s] = np.zeros(5, dtype=bool)
            self.switch[s] = np.zeros(5, dtype=int)

    def trigger_logic(self, sensor_dict, trig_sens):
        sensarr = np.asarray(sensor_dict[trig_sens])
        sdiff = sensarr - self.trigger_thresh[trig_sens]
        trigon = sdiff - self.trigger_hysteresis[trig_sens] > 0
        trigoff = sdiff + self.trigger_hysteresis[trig_sens] < 0
        turnon = trigon & ~self.trig_on[trig_sens] & ~self.turn_state[trig_sens].astype(bool)
        turnoff = trigoff & ~self.trig_off[trig_sens] & self.turn_state[trig_sens].astype(bool)
        n_switch = turnon.astype(int) - turnoff.astype(int)
        self.turn_state[trig_sens] = n_switch + self.turn_state[trig_sens]
        self.trig_on[trig_sens], self.trig_off[trig_sens] = trigon, trigoff
        self.switch[trig_sens] = n_switch
        return n_switch

    def run(self):
        # NOTE: only send_all_data=True path is implemented — this file's
        # __main__ always sets it; non-aggregated (deltas-only) forwarding
        # for multi-sensor trigger lists is unimplemented, not merely buggy.
        while True:
            sensor_dict = self.sensor_q.get(block=True)
            for trig_sens in self.trigger_sensors:
                self.trigger_logic(sensor_dict, trig_sens)
            sensor_dict['hand'] = self.hand
            try:
                self.collect_q.put_nowait(sensor_dict)
            except queue.Full:
                # collect_q is shared between both hands' threads. A
                # blocking put() here, if the consumer is ever slower than
                # this thread's frame rate, would stall THIS thread
                # indefinitely -- which in turn stops draining sensor_q,
                # which backs up this hand's BLE receive path. Since the
                # other hand's thread shares the same collect_q, that
                # stall can starve the other hand's frames too, even
                # though its own BLE link is fine. Drop the oldest queued
                # frame and retry once instead of ever blocking here.
                try:
                    self.collect_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self.collect_q.put_nowait(sensor_dict)
                except queue.Full:
                    pass
            if self.extra_q is not None:
                try:
                    self.extra_q.put_nowait(sensor_dict)
                except queue.Full:
                    pass


class ChoirBaseApp(Thread):
    # Fallback-only now: used by is_motion_armed() before the first
    # motion-tagged frame for a hand arrives (real nav_armed from the wire
    # is preferred once available -- see port_read.py / m5stick_firmware.ino
    # flags bits 4-6). ~0.3s re-arm/settle window after MOTION_STREAM_ON
    # transitions inactive->active, per the hardware brief.
    MOTION_ARM_SETTLE_SEC = 0.3

    # Must match NavEKF.h's zuptGateChiSq_ (accept/reject threshold) and
    # m5stick_firmware.ino / port_read.py's ZUPT_MAHALANOBIS_CAP (wire
    # encoding ceiling) respectively. See zupt_confidence_scale().
    ZUPT_GATE_CHISQ = 16.0
    ZUPT_MAHALANOBIS_CAP = 64.0

    # Roll-driven vibrato (continuous, always-live, no held-finger gate --
    # independent of pitch-tilt's note-step/volume routing). Both hands
    # SUM into one depth value: pitch_bend has no per-hand channel (see
    # midi_writer.py -- trig_note/pitch_bend are both hardcoded to MIDI
    # channel 1), so there is exactly one vibrato lane for the whole
    # instrument regardless of which hand rolls. Units are scaled-byte
    # (0-127 scale, same as roll_trigger_thresh) offset from each hand's
    # calibrated roll0 -- see vibrato_depth_for_hand.
    VIBRATO_DEADZONE = 9        # +/- this many scaled-byte units around roll0: zero depth
    VIBRATO_SATURATION = 45     # scaled-byte units past the deadzone for ONE hand to reach max depth
    VIBRATO_MAX_PITCH_BEND = 2000   # per-hand cap on pitch_bend units (14-bit range is +/-8192;
                                     # summed across 2 hands this can reach 4000, comfortably inside range)
    VIBRATO_LFO_HZ = 4.5        # oscillation rate -- slow, wide, operatic feel

    REVOLUTION_CLAMP = 8   # max +/- full wraps accumulated for yaw/roll revolution counting

    def __init__(self, root_note='D', scale='minor',
                 sensor_config=None, thresholds=None, trigger_sensors=None,
                 mod_sensors=None, mod_idx=None, send_all_data=False,
                 hands=None, hysteresis=None, use_ble=True,
                 enable_motion_stream=False):
        super().__init__()
        self.daemon = True
        self.hands = hands if hands is not None else ['r']
        # thresholds/hysteresis are now required, not defaulted here.
        # Previously this class silently defaulted to {'flex':130,'press':20} /
        # {'flex':5,'press':5} while __main__ passed {'press':130,'flex':20} /
        # {'press':5,'flex':5} — flex/press SWAPPED between the two. __main__'s
        # values always won at runtime (the default was dead code), but the
        # mismatch made it unclear which was actually intended. Single source
        # of truth now: whatever the caller passes in.
        if thresholds is None or hysteresis is None:
            raise ValueError(
                "thresholds and hysteresis must be passed explicitly — "
                "see __main__ for the values currently in use."
            )
        self.hysteresis = hysteresis
        self.thresholds = thresholds
        self.trigger_sensors = trigger_sensors or {h: ['press'] for h in self.hands}
        self.mod_sensors = mod_sensors or {h: [None] for h in self.hands}
        self.mod_idx = mod_idx or {h: [None] for h in self.hands}
        self.send_all_data = send_all_data
        self.sensor_config = self._get_sensor_config(sensor_config)
        self.collect_q = queue.Queue(maxsize=20)
        self.viz_q = {h: queue.Queue(maxsize=5) for h in self.hands}
        self.effect_q = queue.Queue(maxsize=5)
        self.use_ble = use_ble
        self.enable_motion_stream = enable_motion_stream
        self.motion_stream_armed_at = {}   # hand -> time.monotonic() at last MOTION_STREAM_ON; see is_motion_armed()
        self.last_nav_armed = {}   # hand -> most recent real nav_armed bit from the wire; see is_motion_armed()
        self.reader = Reader(sensor_config=self.sensor_config, use_ble=self.use_ble)
        self.mapper = NoteMapper(root_note=root_note, scale=scale)
        self.midi_writer = MidiWriter()

    def _get_sensor_config(self, sensor_config):
        if self.send_all_data:
            return {h: {'flex': True, 'press': True, 'imu': True} for h in self.hands}
        if sensor_config is not None:
            return sensor_config

        config = {h: {'flex': False, 'press': False, 'imu': False} for h in self.hands}
        for hand in self.hands:
            for sensor in config[hand]:
                if self.mod_sensors.get(hand) and sensor in self.mod_sensors[hand]:
                    config[hand][sensor] = True
                if sensor in self.trigger_sensors.get(hand, []):   # fixed: 'in' not '=='
                    config[hand][sensor] = True
        return config

    def _force_accel_streaming(self):
        """imu[3:6]'s accel/gyro mode no longer affects this app's own
        trigger logic -- accel-burst detection now reads NavEKF's lin_accel
        (world-frame, gravity-subtracted) instead of the raw body-frame
        imu[3:6] norm. Kept here anyway: send_all_data=True still requests
        the imu stream (e.g. for viz_q / digital_twin.py's raw-sensor
        display), and USE_GYRO_OFF keeps that stream's accel/gyro mode
        pinned to a known state for whatever else reads it, same as before.
        use_gyro is a RAM-only firmware flag that a calibration session can
        leave set to True (confirmed possible on an aborted gyro-bias
        attempt) — force it off here rather than relying on the calibration
        tool's cleanup path."""
        for hand in self.hands:
            if hand in self.reader.threads:
                try:
                    self.reader.send_command(hand, "USE_GYRO_OFF")
                except Exception as e:
                    print(f"Warning: could not force USE_GYRO_OFF on {hand}: {e}")

    def _enable_motion_streaming(self):
        """MOTION_STREAM_ON -- velocity/position/lin_accel/zupt_quality
        (all ride along, no separate opt-in; see m5stick_firmware.ino /
        port_read.py). Stamps motion_stream_armed_at[hand] as the fallback
        timer for is_motion_armed's brief pre-first-frame window, and clears
        any stale last_nav_armed reading from a previous stream session --
        a hand that's re-enabling motion streaming should fall back to the
        timer heuristic until a fresh nav_armed arrives, not keep trusting
        whatever it last read before the stream was torn down."""
        for hand in self.hands:
            if hand in self.reader.threads:
                try:
                    self.reader.ensure_stream_enabled(hand, "MOTION_STREAM_ON", "lin_accel")
                    self.motion_stream_armed_at[hand] = time.monotonic()
                    self.last_nav_armed.pop(hand, None)
                except Exception as e:
                    print(f"Warning: could not enable MOTION_STREAM_ON on {hand}: {e}")

    def _enable_nav_quat_streaming(self):
        """NAV_QUAT_STREAM_ON -- gravity-corrected roll/pitch (yaw remains
        Mahony-equivalent quality: no magnetometer, unobservable, drifts
        freely either way). Required now that roll/pitch trigger logic reads
        nav_quat instead of the legacy Mahony imu[0:2] byte fields."""
        for hand in self.hands:
            if hand in self.reader.threads:
                try:
                    self.reader.ensure_stream_enabled(hand, "NAV_QUAT_STREAM_ON", "nav_quat")
                except Exception as e:
                    print(f"Warning: could not enable NAV_QUAT_STREAM_ON on {hand}: {e}")

    def is_motion_armed(self, hand):
        """'NavEKF's velocity/position/lin_accel are trustworthy for this
        hand right now'. Prefers the real nav_armed bit decoded from the
        wire (see port_read.py / m5stick_firmware.ino's flags bits 4-6);
        falls back to the MOTION_ARM_SETTLE_SEC timer heuristic only if a
        motion-tagged frame for this hand hasn't arrived yet at all (e.g.
        immediately after MOTION_STREAM_ON, before the first reply). Once
        real frames are flowing this fallback should never actually engage
        -- nav_armed will be present on every one of them, since this app
        always requests motion (see run())."""
        if hand in self.last_nav_armed:
            return self.last_nav_armed[hand]
        armed_at = self.motion_stream_armed_at.get(hand)
        if armed_at is None:
            return False
        return (time.monotonic() - armed_at) >= self.MOTION_ARM_SETTLE_SEC

    def zupt_confidence_scale(self, zupt_mahalanobis):
        """Soft-scaling multiplier in [0,1] for accel_norm, from NavEKF's
        ZUPT goodness-of-fit distance. Full sensitivity (1.0) at or below
        NavEKF.h's own accept/reject gate (ZUPT_GATE_CHISQ=16.0) -- inside
        its trusted zone, this app doesn't second-guess the estimator.
        Past the gate, linear falloff to 0.0 at ZUPT_MAHALANOBIS_CAP (64.0,
        the wire encoding's clamp ceiling -- see port_read.py). None
        (motion not requested, or NavEKF hasn't evaluated a ZUPT candidate
        yet this session) is treated as full sensitivity: there's no
        evidence of a problem, so this doesn't stack with the separate
        nav_armed hard-gate, which already covers the "no data yet" case."""
        if zupt_mahalanobis is None:
            return 1.0
        if zupt_mahalanobis <= self.ZUPT_GATE_CHISQ:
            return 1.0
        span = self.ZUPT_MAHALANOBIS_CAP - self.ZUPT_GATE_CHISQ
        return max(0.0, 1.0 - (zupt_mahalanobis - self.ZUPT_GATE_CHISQ) / span)

    def vibrato_depth_for_hand(self, scaled_roll_value, roll0_value):
        """Vibrato depth contribution (0.0-1.0) from one hand's current
        roll, relative to that hand's calibrated rest position (roll0).
        Deadzone (VIBRATO_DEADZONE) around rest gives zero depth -- natural
        hand wobble at 'rest' shouldn't trigger any vibrato, per the
        hardware brief. Past the deadzone, depth ramps up LINEARLY (not a
        hard on/off) so the wobble eases in with roll angle rather than
        snapping on, saturating at 1.0 once the hand has rolled
        VIBRATO_SATURATION units past the deadzone. Sign of the offset
        doesn't matter -- rolling either direction from rest adds depth the
        same way; only the LFO oscillation itself is signed."""
        offset = abs(scaled_roll_value - roll0_value)
        past_deadzone = max(0.0, offset - self.VIBRATO_DEADZONE)
        return min(1.0, past_deadzone / self.VIBRATO_SATURATION)

    def initialize_triggers(self):
        triggers = {}
        for hand in self.hands:
            if hand not in self.reader.threads:
                print(f"Warning: {hand} hand not connected — skipping trigger init")
                continue
            triggers[hand] = MultiSensorProcess(
                hand, self.reader.threads[hand]['parser'].getQ(), self.collect_q,
                trigger_sensor=self.trigger_sensors[hand],
                trigger_thresh=self.thresholds,
                trigger_hysteresis=self.hysteresis,
                mod_sensors=self.mod_sensors[hand], mod_idx=self.mod_idx[hand], extra_q=self.viz_q[hand],
                send_all_data=self.send_all_data,
            )
            triggers[hand].start()
        self.triggers = triggers

    def sync_press_flex_thresholds_from_live_controls(self):
        press_thresh = self.live_controls.get_threshold('press_thresh')
        flex_thresh = self.live_controls.get_threshold('flex_thresh')
        press_hyst = self.live_controls.get_threshold('press_hysteresis')
        flex_hyst = self.live_controls.get_threshold('flex_hysteresis')
        for hand in self.hands:
            if hand not in self.triggers:
                continue
            trig = self.triggers[hand]
            if press_thresh is not None and 'press' in trig.trigger_thresh:
                trig.trigger_thresh['press'] = press_thresh
            if flex_thresh is not None and 'flex' in trig.trigger_thresh:
                trig.trigger_thresh['flex'] = flex_thresh
            if press_hyst is not None and 'press' in trig.trigger_hysteresis:
                trig.trigger_hysteresis['press'] = press_hyst
            if flex_hyst is not None and 'flex' in trig.trigger_hysteresis:
                trig.trigger_hysteresis['flex'] = flex_hyst

    def run(self):
        self.reader.start_readers()
        self._force_accel_streaming()
        if self.enable_motion_stream:
            self._enable_motion_streaming()
            self._enable_nav_quat_streaming()
        notemaps = self.mapper.basic_map_2hands(hands=self.hands)
        self.initialize_triggers()
        while True:
            reading_dict = self.collect_q.get(block=True)
            if 'switch' in reading_dict:
                self.midi_writer.trig_note_array(reading_dict['switch'],
                                                  notemaps[reading_dict['hand']])


class ChoirMovingWindow(ChoirBaseApp):
    def __init__(self, *args, volume_controller=None, pitch_bender=None,
                 num_lh_fingers=5, num_rh_fingers=5, avg_window_size=10,
                 base_volume=20, use_yaw=True,
                 accel_trigger_thresh=110, accel_trigger_hysteresis=10,
                 accel_norm_max=15000,
                 roll_trigger_thresh_range=32, roll_trigger_hysteresis=5,
                 pitch_trigger_thresh_range=10, pitch_trigger_hysteresis=5,
                 yaw_window=10, vibrato_enabled=True, live_controls=None,
                 **kwargs):
        super().__init__(*args, **kwargs)
        self.num_lh_fingers = num_lh_fingers
        self.num_rh_fingers = num_rh_fingers
        self.playing_notes = [None] * num_rh_fingers
        self.volume_controller = volume_controller
        self.pitch_bender = pitch_bender
        # fixed: build per active hand, prefilled to avoid nan on first mean()
        self.averaging_queue = {h: deque([0.0] * avg_window_size, maxlen=avg_window_size)
                                 for h in self.hands}
        self.base_volume = base_volume
        self.pitch_bend_limit = 8192
        self.global_volume = base_volume
        self.note_array = None
        self.use_yaw = use_yaw

        # IMU-derived trigger tunables — all live here, not hardcoded in run().
        # accel_norm_max is the scaling denominator for the accel-burst
        # detector: it maps a raw accel_norm reading onto [0,127] before
        # accel_trigger_thresh is applied against it. If bursts are hard to
        # trigger, LOWER this first (it directly rescales sensitivity) —
        # accel_trigger_thresh alone is a secondary knob on the same
        # underlying degree of freedom.
        self.accel_trigger_thresh = accel_trigger_thresh
        self.accel_trigger_hysteresis = accel_trigger_hysteresis
        self.accel_norm_max = accel_norm_max
        self.roll_trigger_thresh_range = roll_trigger_thresh_range
        self.roll_trigger_hysteresis = roll_trigger_hysteresis
        # pitch_trigger_thresh_range: previously absent — pitch_trigger_thresh
        # was pinned exactly to the neutral reference pose (pitch0) with zero
        # margin, unlike roll which offsets by +/-roll_trigger_thresh_range.
        # That's a real asymmetry, not an intentional design choice as far as
        # I can tell from the surrounding code — it made pitch_switch fire on
        # almost any tilt at all, bounded only by the much smaller hysteresis.
        # Added here so pitch has the same kind of deadzone-from-neutral that
        # roll already had.
        self.pitch_trigger_thresh_range = pitch_trigger_thresh_range
        self.pitch_trigger_hysteresis = pitch_trigger_hysteresis
        self.yaw_window = yaw_window

        self.live_controls = live_controls if live_controls is not None else LiveControls(
            vibrato_enabled=vibrato_enabled,
            thresholds={
                'accel_trigger_thresh': accel_trigger_thresh,
                'accel_trigger_hysteresis': accel_trigger_hysteresis,
                'roll_trigger_thresh_range': roll_trigger_thresh_range,
                'roll_trigger_hysteresis': roll_trigger_hysteresis,
                'pitch_trigger_thresh_range': pitch_trigger_thresh_range,
                'pitch_trigger_hysteresis': pitch_trigger_hysteresis,
                'yaw_window': yaw_window,
                'press_thresh': self.thresholds.get('press'),
                'flex_thresh': self.thresholds.get('flex'),
                'press_hysteresis': self.hysteresis.get('press'),
                'flex_hysteresis': self.hysteresis.get('flex'),
            },
        )

    def pitch_bend(self, reading_dict):
        if self.pitch_bender in reading_dict:
            return self.scaling_modulo(reading_dict[self.pitch_bender],
                                        min_output=-self.pitch_bend_limit,
                                        max_output=self.pitch_bend_limit,
                                        min_input=0, max_input=TWO_BYTE)
        return 0

    def volume_control(self, hand, reading_dict):
        # fixed: dict/deque type mismatch (was self.averaging_queue.append(...))
        # NOTE: superseded by inline accel-norm handling in run(); kept for API parity
        if self.volume_controller == 'imu1' and 'imu1' in reading_dict:
            return self.input_scaling(reading_dict['imu1'], max_output=127,
                                       min_input=0, max_input=TWO_BYTE)
        if self.volume_controller == 'accel_mag' and all(k in reading_dict for k in ('imu3','imu4','imu5')):
            norm = np.sqrt(sum((reading_dict[f'imu{i}'] - TWO_BYTE/2.0)**2 for i in (3,4,5)))
            volume = max(norm - ZERO_ACCEL, 0)
            self.averaging_queue[hand].append(volume)
            return self.base_volume + self.input_scaling(
                np.mean(self.averaging_queue[hand]),
                max_output=127 - self.base_volume, min_input=0, max_input=self.accel_norm_max)
        return self.base_volume

    def single_trigger_logic(self, sensor_value, trig_on, trig_off, turn_state,
                              trigger_thresh, trigger_hysteresis):
        sdiff = sensor_value - trigger_thresh
        trigon = sdiff - trigger_hysteresis > 0
        trigoff = sdiff + trigger_hysteresis < 0
        turnon = trigon and (not trig_on) and (not turn_state)
        turnoff = trigoff and (not trig_off) and turn_state
        n_switch = int(turnon) - int(turnoff)
        turn_state = n_switch + turn_state
        return trigon, trigoff, turn_state, n_switch

    def scale_bounded90_deg(self, angle_deg):
        """Pitch (asinf-derived, +/-90 deg, reflects rather than wraps) onto
        [0,127]. Separate from scale_wrapped180_deg because the two angle
        families have different native ranges (+/-90 vs +/-180) -- using one
        shared scale factor for both would silently compress or clip
        whichever one it wasn't tuned for."""
        return self.input_scaling(angle_deg, max_output=127, min_input=-90.0, max_input=90.0)

    def scale_wrapped180_deg(self, angle_deg):
        """Yaw or roll (atan2f-derived, +/-180 deg, true wraparound) onto
        [0,127]. See scale_bounded90_deg for why pitch uses a different mapping."""
        return self.input_scaling(angle_deg, max_output=127, min_input=-180.0, max_input=180.0)

    def wrap_around_count(self, prev, curr, threshold=100):
        """
        Discrete phase-unwrap step on a modulo-127 signal:
        n_k = n_{k-1} + sign(theta_{k-1} - theta_k) * 1[|theta_k-theta_{k-1}| > thresh]
        """
        if np.abs(curr - prev) > threshold:
            return np.sign(prev - curr)
        return 0

    def circular_mean_scaled(self, values, scale=127):
        """Circular mean for a scaled [0,scale] value that wraps (yaw/roll —
        true atan2-based +/-180 deg wraparound in the underlying angle). A
        plain arithmetic mean is wrong near the wrap boundary: e.g. samples
        dithering between ~2 and ~125 average to ~63.5, the diametric
        OPPOSITE of the true angle. NOT valid for pitch, which is asin-bounded
        ([-90,90], reflects rather than wraps at the endpoint) — use a plain
        arithmetic mean for pitch instead.
        """
        angles = np.asarray(values, dtype=float) * (2 * np.pi / scale)
        mean_angle = np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))
        return int(round(mean_angle * scale / (2 * np.pi))) % scale

    # --------------------------------------------------------
    # Scaling — clamped both bounds (MIDI range is hard-constrained [0,127]).
    # Required by calibration + run() below; previously pulled in via
    # MovingWindow inheritance which was never actually wired up.
    # --------------------------------------------------------
    def input_scaling(self, input, min_output=0, max_output=127, shift=0,
                       min_input=0, max_input=BYTE):
        output = int(min_output + (max_output - min_output) *
                     ((input + shift) - min_input) / (max_input - min_input))
        return min(max(output, min_output), max_output)

    def scaling_modulo(self, input, min_output=0, max_output=127,
                        min_input=0, max_input=BYTE):
        return int(((max_output + (max_output - min_output) *
                     (input - min_input) / (max_input - min_input))
                    % (max_output - min_output)) - max_output)

    def run(self):
        self.reader.start_readers()
        self._force_accel_streaming()
        # Unconditional here (unlike ChoirBaseApp, where it's still gated by
        # enable_motion_stream for API parity with other subclasses): this
        # app's accel-burst trigger reads lin_accel and its roll/pitch
        # triggers read nav_quat, both requiring these streams. They are no
        # longer optional telemetry for THIS class.
        self._enable_motion_streaming()
        self._enable_nav_quat_streaming()
        self.initialize_triggers()
        notemaps = self.mapper.basic_map_2hands(hands=self.hands)

        # accel_trigger_thresh, roll/pitch trigger ranges, hysteresis, and
        # yaw_window are read live from self.live_controls inside the loop
        # below (see LiveControls) rather than frozen here, so the digital
        # twin's threshold sliders take effect without restarting the app.

        accel_trig_on = {h: 0 for h in self.hands}
        accel_trig_off = {h: 0 for h in self.hands}
        accel_turn_state = {h: 0 for h in self.hands}
        scaled_accel = {h: 0 for h in self.hands}
        accel_switch = {h: 0 for h in self.hands}

        scaled_yaw = {h: 0 for h in self.hands}
        scaled_yaw_reading = {h: 0 for h in self.hands}
        scaled_yaw_prev = {h: 0 for h in self.hands}
        num_revolution_yaw = {h: 0 for h in self.hands}

        pitch_trig_on = {h: 0 for h in self.hands}
        pitch_trig_off = {h: 0 for h in self.hands}
        pitch_turn_state = {h: 0 for h in self.hands}
        pitch_trigger_thresh = {h: 0 for h in self.hands}
        # scaled_pitch is a direct scaled reading now (see scale_bounded90_deg)
        # -- no scaled_pitch_prev/num_revolution_pitch, since pitch
        # (asin-bounded, reflects) doesn't wrap and revolution-counting
        # doesn't apply to it. See module docstring / nav_quat_to_euler_deg.
        scaled_pitch = {h: 0 for h in self.hands}
        pitch_switch = {h: 0 for h in self.hands}
        scaled_pitch_prev_reading = {h: 0 for h in self.hands}
        scaled_pitch_change = {h: 0 for h in self.hands}

        roll_trig_on = {h: 0 for h in self.hands}
        roll_trig_off = {h: 0 for h in self.hands}
        roll_turn_state = {h: 0 for h in self.hands}
        scaled_roll = {h: 0 for h in self.hands}
        scaled_roll_reading = {h: 0 for h in self.hands}
        roll_trigger_thresh = {h: 0 for h in self.hands}
        roll_switch = {h: 0 for h in self.hands}
        scaled_roll_prev = {h: 0 for h in self.hands}
        num_revolution_roll = {h: 0 for h in self.hands}

        yaw0 = {h: 0 for h in self.hands}
        pitch0 = {h: 0 for h in self.hands}
        roll0 = {h: 0 for h in self.hands}
        yaw0_list = {h: [] for h in self.hands}
        pitch0_list = {h: [] for h in self.hands}
        roll0_list = {h: [] for h in self.hands}
        yaw_trigger_thresh = {h: [] for h in self.hands}

        # --- Per-hand independent reference calibration (fixed: was one shared counter) ---
        print('Initializing reference roll/pitch/yaw per hand')
        sample_counts = {h: 0 for h in self.hands}
        SAMPLES_PER_HAND = 150
        CALIBRATION_TIMEOUT_SEC = 15.0   # generous margin over the ~0.3s re-arm window
                                          # and typical BLE sample rates -- if a hand hasn't
                                          # finished by then, something is actually wrong
                                          # (glove not connected, BLE stall), not just slow.
        calibration_start = time.monotonic()
        last_progress_print = calibration_start

        while any(sample_counts[h] < SAMPLES_PER_HAND for h in self.hands):
            elapsed = time.monotonic() - calibration_start
            if elapsed >= CALIBRATION_TIMEOUT_SEC:
                incomplete = {h: sample_counts[h] for h in self.hands if sample_counts[h] < SAMPLES_PER_HAND}
                print(f"WARNING: calibration timed out after {CALIBRATION_TIMEOUT_SEC}s. "
                      f"Proceeding with partial samples for: {incomplete} "
                      f"(target was {SAMPLES_PER_HAND} each). Check that hand's BLE "
                      f"connection and NAV_QUAT_STREAM_ON if this recurs.")
                break

            if time.monotonic() - last_progress_print >= 1.0:
                progress = ', '.join(f"{h}: {sample_counts[h]}/{SAMPLES_PER_HAND}" for h in self.hands)
                print(f"  calibrating... {progress}")
                last_progress_print = time.monotonic()

            try:
                reading_dict = self.collect_q.get(block=True, timeout=0.5)
            except queue.Empty:
                continue   # no frame arrived this half-second -- loop back to re-check the deadline/progress print

            hand = reading_dict['hand']
            if sample_counts[hand] >= SAMPLES_PER_HAND:
                continue
            if 'nav_quat' not in reading_dict:
                # NAV_QUAT_STREAM_ON was just sent; give the BLE round-trip
                # a moment rather than silently skewing the calibration
                # window with fewer real samples than SAMPLES_PER_HAND implies.
                continue
            qw, qx, qy, qz = reading_dict['nav_quat']
            yaw_deg, pitch_deg, roll_deg = nav_quat_to_euler_deg(qw, qx, qy, qz)
            yaw0_list[hand].append(self.scale_wrapped180_deg(yaw_deg))
            pitch0_list[hand].append(self.scale_bounded90_deg(pitch_deg))
            roll0_list[hand].append(self.scale_wrapped180_deg(roll_deg))
            sample_counts[hand] += 1

        REST_SCALED_DEFAULT = 63.5   # scale midpoint (0-127) -- "no tilt/no roll" fallback,
                                      # same convention used throughout this file (see
                                      # scale_bounded90_deg/scale_wrapped180_deg's dithering note)
        for hand in self.hands:
            if len(yaw0_list[hand]) == 0:
                # CALIBRATION_TIMEOUT_SEC hit with literally zero samples for
                # this hand (glove never connected / never streamed nav_quat
                # at all this session) -- sum([])/len([]) would otherwise be
                # a ZeroDivisionError here. Fall back to the scale midpoint
                # rather than crash; this hand's triggers will be miscalibrated
                # until it actually connects and the app is restarted.
                print(f"WARNING: hand '{hand}' produced zero calibration samples -- "
                      f"using midpoint defaults. This hand's roll/pitch/yaw triggers "
                      f"will be inaccurate until it's reconnected and recalibrated.")
                yaw0[hand] = REST_SCALED_DEFAULT
                pitch0[hand] = REST_SCALED_DEFAULT
                roll0[hand] = REST_SCALED_DEFAULT
            else:
                # pitch: asin-bounded, reflects (no true wrap) -- plain mean is correct.
                pitch0[hand] = int(sum(pitch0_list[hand]) / len(pitch0_list[hand]))
                # yaw/roll: atan2-bounded, true +/-180 wraparound -- need circular mean.
                yaw0[hand] = self.circular_mean_scaled(yaw0_list[hand])
                roll0[hand] = self.circular_mean_scaled(roll0_list[hand])
            yaw_trigger_thresh[hand] = [yaw0[hand] - self.yaw_window, yaw0[hand] + self.yaw_window]
            pitch_trigger_thresh[hand] = pitch0[hand] + self.pitch_trigger_thresh_range

        if 'r' in self.hands: roll_trigger_thresh['r'] = roll0['r'] + self.roll_trigger_thresh_range
        if 'l' in self.hands: roll_trigger_thresh['l'] = roll0['l'] - self.roll_trigger_thresh_range

        # --- Main loop ---
        last_frame_time = {h: time.monotonic() for h in self.hands}
        last_stale_warning = {h: 0.0 for h in self.hands}
        STALE_WARN_SEC = 1.0
        STALE_WARN_REPEAT_SEC = 5.0
        while True:
            try:
                reading_dict = self.collect_q.get(block=True, timeout=STALE_WARN_SEC)
            except queue.Empty:
                now_mono = time.monotonic()
                for h in self.hands:
                    gap = now_mono - last_frame_time[h]
                    if gap > STALE_WARN_SEC and now_mono - last_stale_warning[h] > STALE_WARN_REPEAT_SEC:
                        last_stale_warning[h] = now_mono
                        print(f"WARNING: hand '{h}' has produced no frame in {gap:.1f}s -- "
                              f"its roll/pitch/yaw trigger state is frozen at its last known reading.")
                continue
            hand = reading_dict['hand']
            now_mono = time.monotonic()
            last_frame_time[hand] = now_mono
            for h in self.hands:
                if h == hand:
                    continue
                gap = now_mono - last_frame_time[h]
                if gap > STALE_WARN_SEC and now_mono - last_stale_warning[h] > STALE_WARN_REPEAT_SEC:
                    last_stale_warning[h] = now_mono
                    print(f"WARNING: hand '{h}' has produced no frame in {gap:.1f}s -- "
                          f"its roll/pitch/yaw trigger state is frozen at its last known reading.")

            accel_trigger_thresh = self.live_controls.get_threshold('accel_trigger_thresh', self.accel_trigger_thresh)
            accel_trigger_hysteresis = self.live_controls.get_threshold('accel_trigger_hysteresis', self.accel_trigger_hysteresis)
            roll_trigger_thresh_range = self.live_controls.get_threshold('roll_trigger_thresh_range', self.roll_trigger_thresh_range)
            roll_trigger_hysteresis = self.live_controls.get_threshold('roll_trigger_hysteresis', self.roll_trigger_hysteresis)
            pitch_trigger_thresh_range = self.live_controls.get_threshold('pitch_trigger_thresh_range', self.pitch_trigger_thresh_range)
            pitch_trigger_hysteresis = self.live_controls.get_threshold('pitch_trigger_hysteresis', self.pitch_trigger_hysteresis)
            yaw_window = self.live_controls.get_threshold('yaw_window', self.yaw_window)
            self.sync_press_flex_thresholds_from_live_controls()

            # Derived thresholds recomputed from the live range every
            # iteration -- a stale roll_trigger_thresh/pitch_trigger_thresh
            # left over from calibration-time (or the last yaw re-center)
            # would silently ignore a threshold-range change made from the
            # digital twin UI.
            if 'r' in self.hands: roll_trigger_thresh['r'] = roll0['r'] + roll_trigger_thresh_range
            if 'l' in self.hands: roll_trigger_thresh['l'] = roll0['l'] - roll_trigger_thresh_range
            for h in self.hands:
                pitch_trigger_thresh[h] = pitch0[h] + pitch_trigger_thresh_range
                yaw_trigger_thresh[h] = [yaw0[h] - yaw_window, yaw0[h] + yaw_window]

            if 'nav_quat' not in reading_dict or 'lin_accel' not in reading_dict:
                # This hand's NAV_QUAT_STREAM_ON/MOTION_STREAM_ON request is
                # still settling (BLE round-trip, or the ~0.3s re-arm window
                # itself) -- skip this frame's trigger logic rather than
                # process it against stale/absent orientation or accel data.
                # flex/press-only frames (no imu/motion keys requested) never
                # reach this branch in practice since send_all_data=True
                # requests both streams unconditionally in run() above.
                continue

            qw, qx, qy, qz = reading_dict['nav_quat']
            yaw_deg, pitch_deg, roll_deg = nav_quat_to_euler_deg(qw, qx, qy, qz)

            # accel-burst: NavEKF's world-frame, gravity-subtracted lin_accel
            # replaces the old raw body-frame imu[3:6] norm. Already
            # zero-centered by the estimator -- no TWO_BYTE/2.0 offset or
            # ZERO_ACCEL deadband needed, those were compensating for
            # body-frame/raw-count artifacts that don't exist here.
            #
            # Two independent confidence mechanisms stack here:
            #   1. Hard-gate on is_motion_armed (real nav_armed bit, see
            #      below) -- during the re-arm/settle window lin_accel is
            #      not yet trustworthy at all. Rather than trigger notes
            #      off transient garbage, treat the burst detector as
            #      reading a silent zero until armed.
            #   2. Soft-scale on zupt_confidence_scale -- once armed, a
            #      degrading (but not yet starved) ZUPT fit continuously
            #      attenuates sensitivity rather than an abrupt cutoff. See
            #      zupt_confidence_scale's docstring for why the curve is
            #      flat at full sensitivity below NavEKF's own gate.
            # 'nav_armed' in reading_dict is always true now that this app
            # unconditionally requests MOTION_STREAM_ON (see run()) -- the
            # only time it'd be absent is a frame that predates the very
            # first motion-tagged reply, hence the .get(...) + cache pattern
            # feeding is_motion_armed's fallback timer.
            if 'nav_armed' in reading_dict:
                self.last_nav_armed[hand] = reading_dict['nav_armed']
            lax, lay, laz = reading_dict['lin_accel']
            if self.is_motion_armed(hand):
                raw_norm = float(np.sqrt(lax**2 + lay**2 + laz**2))
                confidence = self.zupt_confidence_scale(reading_dict.get('zupt_mahalanobis'))
                accel_norm = raw_norm * confidence
            else:
                accel_norm = 0.0
            self.averaging_queue[hand].append(accel_norm)
            scaled_accel[hand] = self.input_scaling(np.mean(self.averaging_queue[hand]),
                                                      max_output=127, min_input=0, max_input=self.accel_norm_max)

            # pitch: asin-bounded, REFLECTS at +/-90 (no true wrap) -- direct
            # scaled reading, no revolution counting. Applying
            # wrap_around_count here (as an earlier revision of this file
            # did) was measuring a discontinuity that can't structurally
            # occur for an asin-derived angle; it happened to mostly work
            # only because a live hand rarely lingers exactly at +/-90.
            scaled_pitch_prev_reading[hand] = scaled_pitch[hand]
            scaled_pitch[hand] = self.scale_bounded90_deg(pitch_deg)
            scaled_pitch_change[hand] = scaled_pitch[hand] - scaled_pitch_prev_reading[hand]

            # yaw/roll: atan2-bounded, TRUE +/-180 wraparound -- these are
            # the ones that legitimately need revolution counting to recover
            # continuous rotation across the wrap boundary.
            scaled_yaw_reading[hand] = self.scale_wrapped180_deg(yaw_deg)
            scaled_roll_reading[hand] = self.scale_wrapped180_deg(roll_deg)

            num_revolution_yaw[hand] += self.wrap_around_count(scaled_yaw_prev[hand], scaled_yaw_reading[hand])
            num_revolution_roll[hand] += self.wrap_around_count(scaled_roll_prev[hand], scaled_roll_reading[hand])
            # A forearm-mounted glove has no legitimate reason to accumulate
            # more than a few full rotations in a row -- clamp so a run of
            # spurious BLE-glitch unwraps can't push scaled_yaw/scaled_roll
            # far from where roll_trigger_thresh/yaw_trigger_thresh expect
            # them, which would look exactly like "the trigger stopped
            # responding" from the player's side.
            num_revolution_yaw[hand] = int(np.clip(num_revolution_yaw[hand], -self.REVOLUTION_CLAMP, self.REVOLUTION_CLAMP))
            num_revolution_roll[hand] = int(np.clip(num_revolution_roll[hand], -self.REVOLUTION_CLAMP, self.REVOLUTION_CLAMP))

            scaled_yaw[hand] = scaled_yaw_reading[hand] + num_revolution_yaw[hand] * 127
            scaled_roll[hand] = scaled_roll_reading[hand] + num_revolution_roll[hand] * 127

            # Roll-driven vibrato: continuous, always-live, independent of
            # pitch-tilt's held-finger gating (per design -- roll and pitch
            # can both be active at once, controlling different things).
            # Recomputed every iteration using each hand's MOST RECENTLY
            # KNOWN scaled_roll -- BLE delivers one hand's reading_dict at a
            # time (see 'hand = reading_dict['hand']' above), so the other
            # hand's contribution here is whatever it last reported, not
            # necessarily this same instant. That's fine for a continuous
            # wobble; it would NOT be fine for a discrete trigger.
            if self.live_controls.get_vibrato_enabled():
                total_vibrato_depth = sum(
                    self.vibrato_depth_for_hand(scaled_roll[h], roll0[h]) for h in self.hands
                )
                if total_vibrato_depth > 0.0:
                    lfo = np.sin(2 * np.pi * self.VIBRATO_LFO_HZ * time.monotonic())
                    pitch_bend_value = lfo * total_vibrato_depth * self.VIBRATO_MAX_PITCH_BEND
                else:
                    pitch_bend_value = 0.0
                self.midi_writer.pitch_bend(pitch_bend_value)
            elif self.live_controls.pop_vibrato_reset_pending():
                self.midi_writer.pitch_bend(0)

            scaled_yaw_prev[hand] = scaled_yaw_reading[hand]
            scaled_roll_prev[hand] = scaled_roll_reading[hand]

            accel_trig_on[hand], accel_trig_off[hand], accel_turn_state[hand], accel_switch[hand] = \
                self.single_trigger_logic(scaled_accel[hand], accel_trig_on[hand], accel_trig_off[hand],
                                           accel_turn_state[hand], accel_trigger_thresh, accel_trigger_hysteresis)
            roll_trig_on[hand], roll_trig_off[hand], roll_turn_state[hand], roll_switch[hand] = \
                self.single_trigger_logic(scaled_roll[hand], roll_trig_on[hand], roll_trig_off[hand],
                                           roll_turn_state[hand], roll_trigger_thresh[hand], roll_trigger_hysteresis)
            pitch_trig_on[hand], pitch_trig_off[hand], pitch_turn_state[hand], pitch_switch[hand] = \
                self.single_trigger_logic(scaled_pitch[hand], pitch_trig_on[hand], pitch_trig_off[hand],
                                           pitch_turn_state[hand], pitch_trigger_thresh[hand], pitch_trigger_hysteresis)

            active = [n for n in self.playing_notes if n is not None]

            if accel_switch[hand] == 1:
                print('Triggering notes...')
                for h in self.hands:                  
                    yaw0[h] = scaled_yaw[h]
                    yaw_trigger_thresh[h] = [yaw0[h] - yaw_window, yaw0[h] + yaw_window]

                self.playing_notes = self.midi_writer.turn_off_all_playing(self.playing_notes)

                def trig_notes():
                    playing_note_count = 0
                    for hnd in self.hands[::-1]:
                        for i, tsf in enumerate(self.triggers[hnd].turn_state['flex']):
                            if tsf:
                                self.midi_writer.trig_note(notemaps[hnd][i], vel=self.global_volume)
                                self.playing_notes[playing_note_count] = notemaps[hnd][i]
                                playing_note_count += 1
                            if playing_note_count >= 3:
                                return
                trig_notes()
                try:
                    self.effect_q.put_nowait({'hand': hand, 'text': f'NOTE TRIGGER (accel burst): {active}'})
                except queue.Full:
                    pass

            if self.triggers[hand].switch['press'][4] == 1:
                self.playing_notes = self.midi_writer.turn_off_all_playing(self.playing_notes)
                try:
                    self.effect_q.put_nowait({'hand': hand, 'text': f'ALL NOTES OFF: {active}'})
                except queue.Full:
                    pass

            if pitch_switch[hand] != 0:
                if self.use_yaw:
                    note_idx = -1
                    for i in range(len(yaw_trigger_thresh[hand])):
                        if scaled_yaw[hand] < yaw_trigger_thresh[hand][i]:
                            note_idx = i
                            break
                else:
                    # Yaw disabled — always target the first sounding note.
                    note_idx = 0
                    
                switch_condition = (0 <= note_idx < len(self.playing_notes)
                                     and self.triggers[hand].turn_state['press'][2]
                                     and self.playing_notes[note_idx] is not None)
                if switch_condition:
                    self.midi_writer.trig_note(self.playing_notes[note_idx], 0)
                    new_note_idx = self.mapper.notes_in_scale.index(self.playing_notes[note_idx]) + pitch_switch[hand]
                    new_note = self.mapper.notes_in_scale[new_note_idx]
                    self.midi_writer.trig_note(new_note, vel=self.global_volume)
                    self.playing_notes[note_idx] = new_note
                    try:
                        self.effect_q.put_nowait({'hand': hand, 'text': f'NOTE STEP {pitch_switch[hand]:+d} (pitch tilt): {active}'})
                    except queue.Full:
                        pass

            if self.triggers[hand].turn_state['press'][3] == 1:
                self.global_volume = min(max(self.global_volume + int(scaled_pitch_change[hand]), 0), 127)
                self.midi_writer.aftertouch(self.global_volume)
                try:
                    self.effect_q.put_nowait({'hand': hand, 'text': f'VOLUME DELTA (pitch tilt): {active}'})
                except queue.Full:
                    pass


if __name__ == "__main__":
    active_hands = scan_available_ble(timeout=5.0)
    if not active_hands:
        print("[Error] No active Gluvn hardware found.")
        sys.exit(1)
    print(f"Discovered gloves: {[h.upper() for h in sorted(active_hands)]}")

    root_note, scale = 'C', 'major'
    num_rh_fingers = 3
    trigger_sensors = {h: ['press', 'flex'] for h in active_hands}
    thresholds = {'press': 20, 'flex': 100}
    hysteresis = {'press': 5, 'flex': 5}
    ACCEL_TRIGGER_THRESH = 110
    ACCEL_TRIGGER_HYSTERESIS = 10
    ACCEL_NORM_MAX = 15.0   # m/s^2 (~1.5g) -- see note above; was 15000 raw counts pre-migration
    ROLL_TRIGGER_THRESH_RANGE = 20     # ~28.3 deg, roll's mapping (+/-180deg) didn't shift
    ROLL_TRIGGER_HYSTERESIS = 5
    PITCH_TRIGGER_THRESH_RANGE = 12    # ~8.5 deg under pitch's +/-90deg mapping
    PITCH_TRIGGER_HYSTERESIS = 5
    YAW_WINDOW = 10                    # ~14.2 deg under yaw's +/-180deg mapping
    VIBRATO_ENABLED = True

    app = ChoirMovingWindow(
        root_note=root_note, scale=scale,
        volume_controller='press_pitch',
        pitch_bender=None,
        num_rh_fingers=num_rh_fingers,
        avg_window_size=10,
        send_all_data=True,
        trigger_sensors=trigger_sensors,
        thresholds=thresholds,
        hysteresis=hysteresis,
        hands=list(active_hands),
        use_ble=True,
        use_yaw=True,   # if yaw drifting is high, disable this
        accel_trigger_thresh=ACCEL_TRIGGER_THRESH,
        accel_trigger_hysteresis=ACCEL_TRIGGER_HYSTERESIS,
        accel_norm_max=ACCEL_NORM_MAX,
        roll_trigger_thresh_range=ROLL_TRIGGER_THRESH_RANGE,
        roll_trigger_hysteresis=ROLL_TRIGGER_HYSTERESIS,
        pitch_trigger_thresh_range=PITCH_TRIGGER_THRESH_RANGE,
        pitch_trigger_hysteresis=PITCH_TRIGGER_HYSTERESIS,
        yaw_window=YAW_WINDOW,
        vibrato_enabled=VIBRATO_ENABLED,
        enable_motion_stream=True,
    )
    app.start()

    qt_app, twin_window = build_app_and_window(app.viz_q, effect_q=app.effect_q, hands=app.hands,
                                                live_controls=app.live_controls)

    def _shutdown():
        print('Shutting down...')
        app.reader.stop_readers()
        for hand in app.hands:
            if hand in app.triggers:
                app.triggers[hand].join(timeout=0.5)
        app.join(timeout=0.5)

    qt_app.aboutToQuit.connect(_shutdown)
    sys.exit(qt_app.exec())