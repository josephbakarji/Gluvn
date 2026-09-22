"""Single source of truth for pose and orientation.

One :class:`PoseProvider` per :class:`core.port_read.Reader` is the ONLY
consumer of each hand's parser queue. For every decoded frame it:

  1. computes arm-frame yaw/pitch/roll and the render DCM ONCE, using the
     orientation functions defined in this same file (the mounting
     correction lives nowhere else), and
  2. republishes the *enriched* frame to every subscriber, so the choir's
     trigger logic, the digital twin and any ad-hoc consumer see identical
     data and can never disagree about which angle is "pitch".

Why a sole drain: ``queue.Queue.get`` hands each item to exactly one caller.
Two threads reading the same parser queue split its frames between them
(measured: ~50/50 with two consumers), so a second reader would halve the
frame rate seen by the first. Every consumer therefore attaches through
:meth:`subscribe` (a private queue per consumer) instead of touching
``reader.threads[hand]['parser'].getQ()`` directly. The provider must be the
one to ``start()``/``stop()`` the reader's dispatchers; do not also start a
second consumer on the parser queue.

Enriched frame (a shallow copy of the parser dict, original left untouched):
    everything the parser produced (flex, press, imu, lin_accel, nav_armed,
    nav_quat, position, velocity, ...) plus, when ``nav_quat`` is present:
        arm_ypr   (yaw, pitch, roll) degrees, arm frame
                  yaw CCW+ from above, +/-180 | pitch +up, +/-90 | roll
                  supination+, +/-180
        R_render  3x3 world<-sensor DCM, for drawing the sensor-frame mesh
    and always:
        hand      'l' | 'r'   (so a fan-in queue needs no side channel)

Stream demand is reference-counted per hand, independently for motion and
NavEKF-quaternion requests: ON is sent on the first request, OFF on the
last release. ``request(hand, motion=True, ...)`` also yields ``lin_accel``
(world-frame, gravity-subtracted); the firmware bundles it into the same
MOTION_STREAM_ON frame as velocity/position, so there is no separate flag.
``PoseSample.lin_accel`` is None whenever ``motion`` was not requested.

This supports multiple consumers within ONE Python process sharing one
BLE/USB connection. It does not support multiple processes connecting to
the same glove (BLE peripherals generally allow one central). A future
multi-process design should have one owner process rebroadcast decoded
samples.

ORIENTATION AND POSITION SOURCE (firmware is the truth; this file only relays it)
------------------------------------------------------------------------------
orientation  Mahony's Euler output, imu[0:3], decoded exactly (fw_ypr). Valid from
             the first frame, matches the stick's display, honours RESET_YAW.
             The NavEKF quaternion is NOT used: it is the identity until the EKF
             arms and after every navEkf.reset() (incl. enabling the streams).
position     NavEKF position/velocity/lin_accel, as sent, rotated about Z into the
velocity     display frame (see MotionConfig.align_nav_to_display); originals
lin_accel    kept as position_nav / velocity_nav / lin_accel_nav. Only meaningful
             while nav_armed.
arm_ypr      the same attitude named for how the stick is worn
             (MotionConfig.sensor_mounting); 'firmware' = no renaming.

MOTION PIPELINE -- what a music app gets, and what it must not do
------------------------------------------------------------------
Per frame the provider attaches ``frame['motion']`` (a MotionFrame): arm-frame
angles, 0-127 scaled angles with yaw/roll unwrapped, calibrated references,
windowed accel, roll depth, yaw sector, latched finger levels, and EDGE EVENTS
(accel_burst, pitch_step, press/flex on/off). All of it is computed once, here,
by HandMotion/FingerTriggers, from thresholds in ``provider.tuning`` (the same
LiveControls the twin's sliders drive). A music app therefore only decides what
to DO with those numbers and edges: which note, which controller, which sound.
It should never compute an angle, a threshold crossing, a latch, a calibration,
or read a parser queue. Lifecycle is bring_up() -> calibrate() -> consume.

SENSOR -> ARM ORIENTATION (defined below, in the "Orientation" section)
-----------------------------------------------------------------------
The M5StickC IMU is worn with its long axis on sensor +Y (fingers point +Y,
screen normal +Z). Firmware (MahonyAHRS.computeEuler / NavEKF quatToDCM) and
every downstream consumer name angles with the aerospace convention
R = Rz(yaw) Ry(pitch) Rx(roll), i.e. they implicitly assume the long axis is
body +X. Consequence without correction:

    raise forearm      -> firmware ROLL   changes   (should be pitch)
    supinate hand      -> firmware PITCH  changes   (should be roll)

Every consumer must see the SAME correction, so it is applied once, here,
per frame, and nowhere else.

The correction: conjugate the attitude by the fixed axis map C
(v_arm = C v_sensor):

        [0 1 0]
    C = [1 0 0]     det(C) = +1  (proper rotation: 180 deg about (1,1,0)/sqrt2)
        [0 0 -1]

Unique proper signed permutation satisfying: fingers (+Y_s) -> +X_arm,
raise -> +pitch, supinate -> +roll (verified by exhaustive search over the 24
proper signed permutations).

    R_arm = C R_sensor C^T
    quaternion vector part:  (x, y, z) -> (y, x, -z),  w unchanged
    Z_arm points DOWN in the world frame after conjugation, so Euler yaw comes
    out negated; it is re-negated in euler_arm_deg() so yaw stays
    counter-clockwise-positive seen from above (same sign as legacy).

Hand-independence: applied identically to both hands. Mirror handedness of a
left glove is a rendering concern (twin finger spread), not an attitude one --
a mirrored glove rotates the same way as a non-mirrored one.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field, replace
from threading import Condition, Event, Lock, RLock, Thread
from typing import Optional, Tuple
import queue
import threading
import time

import numpy as np

from core.port_read import Reader
from core.hand_diag import HandDiag


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


# ===========================================================================
# Orientation: sensor frame -> arm frame
# (rationale and derivation of C are in the module docstring above)
# ===========================================================================

# v_arm = C @ v_sensor
C_SENSOR_TO_ARM = np.array([[0.0, 1.0, 0.0],
                            [1.0, 0.0, 0.0],
                            [0.0, 0.0, -1.0]])


def quat_sensor_to_arm(qw, qx, qy, qz):
    """Sensor-frame attitude quaternion -> arm-frame attitude quaternion.
    Conjugation by C on the vector part; w is invariant."""
    return qw, qy, qx, -qz


def rotation_matrix_from_quat(qw, qx, qy, qz):
    """Body-to-world DCM, same convention as NavEKF quatToDCM /
    MahonyAHRS computeEuler."""
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qw * qz)
    r02 = 2.0 * (qx * qz + qw * qy)
    r10 = 2.0 * (qx * qy + qw * qz)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qw * qx)
    r20 = 2.0 * (qx * qz - qw * qy)
    r21 = 2.0 * (qy * qz + qw * qx)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    return np.array([[r00, r01, r02],
                     [r10, r11, r12],
                     [r20, r21, r22]])


MOUNTINGS = ('firmware', 'firmware_rot180', 'fingers_along_x', 'fingers_along_y')
DEFAULT_MOUNTING = 'firmware'


def per_hand(value, hand, default=None):
    """Resolve a scalar config value or a hand-keyed mapping."""
    if isinstance(value, dict):
        return value.get(hand, default)
    return value


def euler_arm_deg(qw, qx, qy, qz, mounting=DEFAULT_MOUNTING):
    """Sensor-frame quaternion -> (yaw, pitch, roll) in DEGREES, arm frame.

    yaw   about world up, CCW-positive from above, +/-180 (true wrap)
    pitch forearm elevation, +up, +/-90 (reflects, no wrap)
    roll  supination about the forearm long axis, +/-180 (true wrap)

    Same formulas as MahonyAHRS::computeEuler on the corrected quaternion,
    then yaw re-negated (see module docstring).
    """
    if mounting == 'fingers_along_x':
        # Sensor frame IS the arm frame, so no axis remap and yaw/roll keep the
        # firmware's sign. Pitch does NOT: the firmware's world is Z-up (the
        # accelerometer reads +1 g on Z at rest), which makes its own pitch
        # positive when the long axis points DOWN. Flip it so both mountings
        # deliver the same contract: +pitch = forearm raised.
        aw, ax, ay, az, yaw_sign, pitch_sign = qw, qx, qy, qz, 1.0, 1.0
    elif mounting == 'firmware_rot180':
        # The stick's screen normal is unchanged, but its X/Y sensor axes are
        # reversed. Conjugating by Rz(pi) restores the shared physical frame.
        aw, ax, ay, az, yaw_sign, pitch_sign = qw, -qx, -qy, qz, 1.0, -1.0
    elif mounting == 'firmware':
        # Pure pass-through: the firmware's own formulas (MahonyAHRS::computeEuler)
        # on the unmodified attitude -- no remap, no sign flips.
        aw, ax, ay, az, yaw_sign, pitch_sign = qw, qx, qy, qz, 1.0, -1.0
    else:
        aw, ax, ay, az = quat_sensor_to_arm(qw, qx, qy, qz)
        yaw_sign, pitch_sign = -1.0, -1.0   # Z_arm points down after conjugation
    r00 = 1.0 - 2.0 * (ay * ay + az * az)
    r10 = 2.0 * (ax * ay + aw * az)
    r20 = 2.0 * (ax * az - aw * ay)
    r21 = 2.0 * (ay * az + aw * ax)
    r22 = 1.0 - 2.0 * (ax * ax + ay * ay)

    s = min(max(r20, -1.0), 1.0)
    yaw = yaw_sign * np.degrees(np.arctan2(r10, r00))
    pitch = pitch_sign * np.degrees(np.arcsin(s))
    roll = np.degrees(np.arctan2(r21, r22))
    return float(yaw), float(pitch), float(roll)


def rotation_render_from_quat(qw, qx, qy, qz):
    """DCM to draw the sensor-frame mesh. This is intentionally the RAW
    firmware attitude, with NO mounting remap.

    Verified numerically (2000 random attitudes, max error 1.2e-14): the
    twin's existing Euler->scipy 'ZYX' render reproduces the firmware
    quaternion's rotation exactly, so the drawn hand's physical pose was
    never wrong. The mesh is authored in the sensor frame (fingers +Y) and
    the firmware quaternion is world<-sensor, so no correction belongs here.

    The mounting correction is an ANGLE-NAMING fix only: it changes which
    Euler angle is called pitch vs roll for the trigger logic and the
    telemetry readout. Applying C to the render matrix would rotate the
    drawn hand away from the real one.
    """
    return rotation_matrix_from_quat(qw, qx, qy, qz)


# ---------------------------------------------------------------------------
# Frame-level entry point: the ONE function that turns a decoded firmware
# sample into pose. pose_provider.PoseProvider calls this once per frame and
# attaches the result, so no consumer ever re-derives angles or rotations.
# ---------------------------------------------------------------------------

def pose_from_quat(qw, qx, qy, qz, mounting=DEFAULT_MOUNTING):
    """Everything a consumer needs from one nav_quat, computed once.

    Returns a dict:
        ypr_deg : (yaw, pitch, roll) in DEGREES, arm frame -- what trigger
                  logic and telemetry read (see euler_arm_deg).
        R       : 3x3 world<-sensor DCM (raw attitude) -- what the renderer
                  applies to the sensor-frame mesh (see
                  rotation_render_from_quat for why this is NOT remapped).
    """
    return {
        "ypr_deg": euler_arm_deg(qw, qx, qy, qz, mounting),
        "R": rotation_render_from_quat(qw, qx, qy, qz),
    }



# ---------------------------------------------------------------------------
# Firmware truth: decode what the stick actually sends
# ---------------------------------------------------------------------------
#
# ORIENTATION comes from Mahony's Euler output (imu[0:3] on the wire), NOT from
# the NavEKF quaternion. Reasons, all from the firmware sources:
#   * NavEKF's quaternion is the identity until the EKF arms (NavEKF::reset()
#     sets it to 1,0,0,0) and is reset again by RESET_POSITION, the stick's
#     button, a gyro recalibration, AND by enabling MOTION/NAV_QUAT streams
#     (buildAndSendFrame's navEkfNeeded rising edge). The firmware sends it
#     regardless, so it is garbage exactly when a session starts.
#   * imu[0:3] is what the stick's own display shows, is valid from the first
#     frame, and honours RESET_YAW (ahrs.resetYaw()) -- the legacy choir used it.
# NavEKF is used only for what only it provides: position, velocity and
# gravity-free linear acceleration, gated on nav_armed.

def decode_mahony_euler(yaw_u16, pitch_u16, roll_u16):
    """Exact inverse of the firmware's quantiser (m5stick_firmware.ino):
    yaw/roll (angle+180)*32767/180, pitch (angle+90)*32767/180.
    Returns (yaw, pitch, roll) in degrees, the firmware's own numbers."""
    return (yaw_u16 * 180.0 / 32767.0 - 180.0,
            pitch_u16 * 180.0 / 32767.0 - 90.0,
            roll_u16 * 180.0 / 32767.0 - 180.0)


def quat_from_fw_euler(yaw_deg, pitch_deg, roll_deg):
    """Quaternion (w, x, y, z) of R = Rz(yaw) Ry(pitch) Rx(roll), the exact
    inverse of MahonyAHRS::computeEuler (yaw=atan2(R10,R00),
    pitch=-asin(R20), roll=atan2(R21,R22)). Verified round-trip to <1e-12 deg."""
    cy, sy = np.cos(np.radians(yaw_deg) / 2), np.sin(np.radians(yaw_deg) / 2)
    cp, sp = np.cos(np.radians(pitch_deg) / 2), np.sin(np.radians(pitch_deg) / 2)
    cr, sr = np.cos(np.radians(roll_deg) / 2), np.sin(np.radians(roll_deg) / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def firmware_pose(imu, mounting=DEFAULT_MOUNTING, pitch_sign=1):
    """The single decode of a frame's orientation.

    ``imu`` is the parser's six-value imu list. Returns None if unusable, else
      fw_ypr   the firmware's own (yaw, pitch, roll) degrees, exactly as sent
      arm_ypr  the same attitude named per ``mounting`` (equal to fw_ypr for
               'firmware'); this is what music apps read
      R        world<-sensor DCM of the firmware attitude, for rendering
    """
    if imu is None or len(imu) < 3:
        return None
    fw = decode_mahony_euler(imu[0], imu[1], imu[2])
    pose = pose_from_quat(*quat_from_fw_euler(*fw), mounting=mounting)
    arm_ypr = pose["ypr_deg"]
    arm_ypr = (arm_ypr[0], pitch_sign * arm_ypr[1], arm_ypr[2])
    return {"fw_ypr": fw, "arm_ypr": arm_ypr, "R": pose["R"]}


def wrap180(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def quat_yaw_deg(quat):
    """Heading (deg) of a (w, x, y, z) attitude, MahonyAHRS's formula."""
    w, x, y, z = quat
    return float(np.degrees(np.arctan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z))))


def rotate_z(vec, deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return (c * vec[0] - s * vec[1], s * vec[0] + c * vec[1], vec[2])


# ===========================================================================
# Motion processing: everything between a decoded frame and a music app
# ===========================================================================
#
# A music app never computes an angle, a threshold crossing, a finger latch or
# a calibration itself. The PoseProvider runs one HandMotion per hand inside
# its dispatcher thread and attaches the result to every frame as
# ``frame['motion']`` (a MotionFrame). Every subscriber -- the choir, the
# digital twin, an app that does not exist yet -- therefore sees identical
# values and identical edge events, and a threshold fixed here is fixed for all
# of them. Nothing below does I/O; HandMotion is a plain state machine that can
# be driven from a test with hand-made frames.

REST_SCALED_DEFAULT = 63.5   # midpoint of the 0-127 scale: "no tilt / no roll"


def input_scaling(value, min_output=0, max_output=127, shift=0,
                  min_input=0, max_input=255):
    """Linear map of ``value`` onto [min_output, max_output], clamped on both
    ends (MIDI's range is hard-limited to 0-127). Every call site passes its
    own input range; 255 is only the historical default."""
    out = int(min_output + (max_output - min_output) *
              ((value + shift) - min_input) / (max_input - min_input))
    return min(max(out, min_output), max_output)


def scale_bounded90_deg(angle_deg):
    """Pitch (asin-derived, +/-90 deg, reflects rather than wraps) onto [0,127].
    Separate from scale_wrapped180_deg: the two angle families have different
    native ranges, and one shared factor would compress or clip one of them.
    1 unit = 1.417 deg."""
    return input_scaling(angle_deg, max_output=127, min_input=-90.0, max_input=90.0)


def scale_wrapped180_deg(angle_deg):
    """Yaw or roll (atan2-derived, +/-180 deg, true wraparound) onto [0,127].
    1 unit = 2.835 deg."""
    return input_scaling(angle_deg, max_output=127, min_input=-180.0, max_input=180.0)


def wrap_around_count(prev, curr, threshold=100):
    """One phase-unwrap step on a modulo-127 signal:
    n_k = n_{k-1} + sign(theta_{k-1} - theta_k) * 1[|theta_k - theta_{k-1}| > thresh]."""
    if np.abs(curr - prev) > threshold:
        return np.sign(prev - curr)
    return 0


def circular_mean_scaled(values, scale=127):
    """Circular mean of a scaled [0,scale] value that wraps (yaw/roll). A plain
    mean is wrong near the seam: samples dithering between ~2 and ~125 average
    to ~63.5, the diametric opposite of the true angle. NOT valid for pitch,
    which reflects at +/-90 instead of wrapping -- use a plain mean there."""
    angles = np.asarray(values, dtype=float) * (2 * np.pi / scale)
    mean_angle = np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))
    return int(round(mean_angle * scale / (2 * np.pi))) % scale


def zupt_confidence_scale(zupt_mahalanobis, gate_chisq=16.0, cap=64.0):
    """Soft-scaling multiplier in [0,1] for accel_norm from NavEKF's ZUPT
    goodness-of-fit. Full sensitivity at or below NavEKF.h's own accept/reject
    gate (16.0); linear falloff to 0 at the wire clamp (64.0). None (no
    evidence yet) is treated as full sensitivity -- the separate nav_armed
    hard gate already covers "no data yet"."""
    if zupt_mahalanobis is None or zupt_mahalanobis <= gate_chisq:
        return 1.0
    return max(0.0, 1.0 - (zupt_mahalanobis - gate_chisq) / (cap - gate_chisq))


def roll_depth(scaled_roll, roll0, deadzone, saturation):
    """Roll deflection from calibrated rest as a 0.0-1.0 depth. Zero inside the
    dead zone (rest wobble must do nothing), then linear (eases in rather than
    snapping on), saturating ``saturation`` units past it. The sign of the
    offset is ignored: rolling either way from rest adds depth. Apps map this
    to vibrato, tremolo, filter -- whatever they like."""
    return min(1.0, max(0.0, abs(scaled_roll - roll0) - deadzone) / saturation)


def schmitt_latch(value, trig_on, trig_off, turn_state, thresh, hyst):
    """Hysteresis latch on a scalar. Returns (trig_on, trig_off, turn_state,
    switch) with switch in {-1, 0, +1}: +1 on the rising edge above
    thresh+hyst, -1 on the falling edge below thresh-hyst (only after a +1)."""
    sdiff = value - thresh
    trigon = sdiff - hyst > 0
    trigoff = sdiff + hyst < 0
    turnon = trigon and (not trig_on) and (not turn_state)
    turnoff = trigoff and (not trig_off) and turn_state
    n_switch = int(turnon) - int(turnoff)
    return trigon, trigoff, turn_state + n_switch, n_switch


def initial_pitch_state(mode='directional'):
    return (False, False, 0) if mode == 'legacy_latch' else 0


def pitch_step(offset, state, thresh_range, hysteresis, mode='directional'):
    """Bidirectional note-step gesture from forearm pitch (tilt up / down).

    ``offset`` is scaled pitch minus the calibrated rest pitch. Returns
    (new_state, step) with step in {-1, 0, +1}.

    'directional' (default) is a Schmitt trigger in each direction, state in
    {-1, 0, +1}: armed at neutral, fires +1 above +(range+hyst) and -1 below
    -(range+hyst), re-arms once |offset| < max(range, hyst/2). Fire and re-arm
    edges stay apart for any range >= 0, so a reading dithering at either edge
    cannot chatter, and one tilt is exactly one step. (The old re-arm test
    |off| < max(range-hyst, 0) was unsatisfiable whenever range <= hyst, so the
    latch died after its first step.)

    'legacy_latch' reproduces the legacy edge-toggle for A/B: +1 on crossing
    above, -1 on crossing below only after a prior +1; tilting down from
    neutral is silent.
    """
    if mode == 'legacy_latch':
        on_prev, off_prev, ts = state if isinstance(state, tuple) else (False, False, 0)
        tron = (offset - hysteresis) > 0
        troff = (offset + hysteresis) < 0
        turnon = tron and not on_prev and not ts
        turnoff = troff and not off_prev and ts
        n = int(turnon) - int(turnoff)
        return (tron, troff, ts + n), n
    fire = thresh_range + hysteresis
    if state == 0:
        if offset > fire:
            return 1, 1
        if offset < -fire:
            return -1, -1
        return 0, 0
    if abs(offset) < max(thresh_range, 0.5 * hysteresis):
        return 0, 0
    return state, 0


class LiveControls:
    """Thread-safe bridge for settings a UI (the digital twin) can change
    while the motion pipeline is already running in other threads. One
    instance is shared: the pipeline reads the getters on EVERY frame (never
    caches a value), the Qt thread calls the setters from its widget
    callbacks. ``vibrato_enabled`` is the twin's roll-depth modulation switch;
    the pipeline always computes roll_depth, apps decide whether to use it."""

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


@dataclass
class MotionConfig:
    """Static parameters for the motion pipeline. The live-tunable subset is
    seeded into LiveControls (see :meth:`tuning_seed`) so a UI can change it
    without a restart; everything else is fixed for the session."""

    accel_norm_max: float = 15.0      # m/s^2 that maps to 127 in the burst detector.
                                      # If bursts are hard to trigger LOWER this first.
    avg_window_size: int = 10         # frames averaged before the burst threshold
    accel_trigger_thresh: int = 110
    accel_trigger_hysteresis: int = 10
    roll_trigger_thresh_range: int = 20
    roll_trigger_hysteresis: int = 5
    # Pitch step fires at (range+hyst) = 10 units = 14.2 deg from rest, legacy's
    # 14.3 deg, and re-arms at `range` = 5 units = 7.1 deg.
    pitch_trigger_thresh_range: int = 5
    pitch_trigger_hysteresis: int = 5
    yaw_window: int = 10              # +/-10 units = +/-28.3 deg (yaw is 2.835 deg/unit)
    roll_depth_deadzone: int = 4      # units (~11 deg): zero depth inside
    roll_depth_saturation: int = 15   # units (~42 deg) past the dead zone for full depth
    pitch_step_mode: str = 'directional'
    # Physical mounting fact. 'firmware' is the documented pass-through and is
    # the default: no angle is renamed or sign-flipped. Other values are opt-in
    # hypotheses produced by tools/mounting_check.py.
    sensor_mounting: object = DEFAULT_MOUNTING
    # +1 preserves legacy firmware parity; -1 reverses musical pitch stepping.
    pitch_sign: object = 1
    # Position/velocity/lin_accel come from NavEKF's world frame; orientation
    # from Mahony's display frame. The firmware documents that RESET_YAW does
    # NOT touch NavEKF's frame, so after a reset they differ by a yaw. Rotate
    # the NavEKF vectors into the display frame so position and orientation
    # agree. Estimated from the two yaws while armed, low-passed; a step
    # larger than align_snap_deg (a reset) is adopted at once.
    align_nav_to_display: bool = True
    align_tau_sec: float = 1.0
    align_snap_deg: float = 30.0
    motion_arm_settle_sec: float = 0.3
    zupt_gate_chisq: float = 16.0     # NavEKF.h zuptGateChiSq_
    zupt_mahalanobis_cap: float = 64.0  # port_read.py wire clamp
    revolution_clamp: int = 8         # max +/- full wraps accumulated for yaw/roll
    calibration_window_sec: float = 2.0
    stale_warn_sec: float = 1.0
    stale_warn_repeat_sec: float = 5.0
    trigger_sensors: Optional[dict] = None   # hand -> ['press', 'flex']; None = both
    finger_thresholds: dict = field(default_factory=lambda: {'press': 20, 'flex': 100})
    finger_hysteresis: dict = field(default_factory=lambda: {'press': 5, 'flex': 5})
    vibrato_enabled: bool = True      # the twin's roll-depth toggle (LiveControls)

    def __post_init__(self):
        mountings = self.sensor_mounting.values() if isinstance(self.sensor_mounting, dict) else (self.sensor_mounting,)
        if any(m not in MOUNTINGS for m in mountings):
            raise ValueError(f"sensor_mounting must use values from {MOUNTINGS}, got {self.sensor_mounting!r}")
        signs = self.pitch_sign.values() if isinstance(self.pitch_sign, dict) else (self.pitch_sign,)
        if any(s not in (-1, 1) for s in signs):
            raise ValueError(f"pitch_sign must use -1 or 1, got {self.pitch_sign!r}")
        if self.pitch_step_mode not in ('directional', 'legacy_latch'):
            raise ValueError("pitch_step_mode must be 'directional' or 'legacy_latch', "
                             f"got {self.pitch_step_mode!r}")

    def sensors_for(self, hand):
        if self.trigger_sensors and hand in self.trigger_sensors:
            return list(self.trigger_sensors[hand])
        return ['press', 'flex']

    def tuning_seed(self):
        return {
            'accel_trigger_thresh': self.accel_trigger_thresh,
            'accel_trigger_hysteresis': self.accel_trigger_hysteresis,
            'roll_trigger_thresh_range': self.roll_trigger_thresh_range,
            'roll_trigger_hysteresis': self.roll_trigger_hysteresis,
            'pitch_trigger_thresh_range': self.pitch_trigger_thresh_range,
            'pitch_trigger_hysteresis': self.pitch_trigger_hysteresis,
            'yaw_window': self.yaw_window,
            'press_thresh': self.finger_thresholds.get('press'),
            'flex_thresh': self.finger_thresholds.get('flex'),
            'press_hysteresis': self.finger_hysteresis.get('press'),
            'flex_hysteresis': self.finger_hysteresis.get('flex'),
        }


@dataclass(frozen=True)
class MotionEvent:
    """One edge. ``hand`` is the hand that produced it (a merged queue can
    carry events from both hands, so never assume the frame's hand).

    kind        value       index
    'accel_burst'  +1 rise / -1 fall   -1
    'pitch_step'   +1 up / -1 down     -1
    'press'|'flex' +1 on / -1 off      finger 0-4
    yaw_sector is the heading bin at the moment of the event (0 = left of the
    window, 1 = inside it, -1 = right of it / unknown).
    """
    hand: str
    kind: str
    value: int
    index: int = -1
    yaw_sector: int = -1


@dataclass(frozen=True)
class FingerState:
    """Latched (hysteresis) finger levels, 1 = held. Not raw sensor values."""
    flex: Tuple[int, ...] = (0, 0, 0, 0, 0)
    press: Tuple[int, ...] = (0, 0, 0, 0, 0)


@dataclass(frozen=True)
class MotionFrame:
    """Everything a music app needs to know about one hand at one instant.

    valid        False when the frame lacked orientation or motion data (a stream
                 still settling). Feature fields then HOLD their last valid value;
                 do not act on them -- check ``valid`` first.
    calibrating  the reference pose is being collected; no events are emitted.
    armed        NavEKF's lin_accel is trustworthy (nav_armed bit, else a settle timer).
    ypr_deg      arm-frame (yaw, pitch, roll) in degrees.
    scaled_*     0-127 scale; yaw and roll are UNWRAPPED (they can leave 0-127 by
                 whole revolutions), pitch is not.
    pitch_delta  change in scaled_pitch since the previous frame.
    accel_norm   armed, ZUPT-confidence-weighted |lin_accel| in m/s^2.
    accel_scaled the windowed mean of accel_norm on 0-127 (what the burst
                 threshold sees).
    roll_depth   0.0-1.0 roll deflection from rest (see roll_depth()).
    yaw_sector   see MotionEvent.
    reference    calibrated (yaw0, pitch0, roll0) in scaled units.
    fingers      latched finger levels as of this frame.
    events       edges since the previous frame delivered to this consumer.
                 A slow consumer loses state frames but never an edge: see
                 EdgeSafeQueue (edges can also arrive on a later frame than the
                 one they were detected on, and possibly from the other hand).
    """
    hand: str
    t: float
    valid: bool
    calibrating: bool = False
    calibrated: bool = False
    armed: bool = False
    ypr_deg: Optional[Vector3] = None
    scaled_yaw: int = 0
    scaled_pitch: int = 0
    scaled_roll: int = 0
    pitch_delta: int = 0
    accel_norm: float = 0.0
    accel_scaled: int = 0
    roll_depth: float = 0.0
    yaw_sector: int = -1
    reference: Tuple[float, float, float] = (REST_SCALED_DEFAULT,) * 3
    fingers: FingerState = FingerState()
    events: Tuple[MotionEvent, ...] = ()


class FingerTriggers:
    """Per-hand hysteresis latches on the raw flex/press arrays. Thresholds are
    read live from ``tuning`` (keys press_thresh, flex_hysteresis, ...) with the
    static values as fallback, so a slider change applies on the next frame."""

    def __init__(self, hand, sensors, thresholds, hysteresis, tuning=None):
        self.hand = hand
        self.sensors = list(sensors)
        self._thresh = dict(thresholds)
        self._hyst = dict(hysteresis)
        self.tuning = tuning
        self.turn, self.on, self.off = {}, {}, {}

    def _live(self, key, default):
        if self.tuning is not None:
            v = self.tuning.get_threshold(key, default)
            if v is not None:
                return v
        return default

    def state(self):
        def lvl(s):
            arr = self.turn.get(s)
            return tuple(int(v) for v in arr) if arr is not None else (0, 0, 0, 0, 0)
        return FingerState(flex=lvl('flex'), press=lvl('press'))

    def update(self, frame):
        events = []
        for s in self.sensors:
            raw = frame.get(s)
            if raw is None:
                continue
            arr = np.asarray(raw)
            if s not in self.turn or len(self.turn[s]) != len(arr):
                self.turn[s] = np.zeros(len(arr), dtype=int)
                self.on[s] = np.zeros(len(arr), dtype=bool)
                self.off[s] = np.zeros(len(arr), dtype=bool)
            t = self._live(f'{s}_thresh', self._thresh.get(s, 0))
            h = self._live(f'{s}_hysteresis', self._hyst.get(s, 0))
            sdiff = arr - t
            trigon = sdiff - h > 0
            trigoff = sdiff + h < 0
            turnon = trigon & ~self.on[s] & ~self.turn[s].astype(bool)
            turnoff = trigoff & ~self.off[s] & self.turn[s].astype(bool)
            n = turnon.astype(int) - turnoff.astype(int)
            self.turn[s] = n + self.turn[s]
            self.on[s], self.off[s] = trigon, trigoff
            for i in np.nonzero(n)[0]:
                events.append(MotionEvent(self.hand, s, int(n[i]), int(i)))
        return self.state(), events


class HandMotion:
    """One hand's motion state machine: calibration, scaling, unwrapping, accel
    features, gesture edges and finger latches. Pure -- no threads, no I/O.
    ``update`` is called from one thread (the provider's dispatcher for this
    hand); ``recenter_yaw`` may be called from an app thread, hence the lock."""

    def __init__(self, hand, config, tuning=None, fingers=None):
        self.hand = hand
        self.cfg = config
        self.tuning = tuning
        self.fingers = fingers
        self._lock = threading.Lock()
        self._calibrating = False
        self._calibrated = False
        self._yaw0 = self._pitch0 = self._roll0 = REST_SCALED_DEFAULT
        self._cal_y, self._cal_p, self._cal_r = [], [], []
        self._armed_at = None
        self._last_nav_armed = None
        self._have_pose = False
        self._v = dict(ypr=None, sy=0, sp=0, sr=0, dp=0, an=0.0, asc=0, rd=0.0, sec=-1)
        self._reset_runtime()

    # -- helpers -----------------------------------------------------------

    def _t(self, key, default):
        if self.tuning is not None:
            v = self.tuning.get_threshold(key, default)
            if v is not None:
                return v
        return default

    def _reset_runtime(self):
        n = self.cfg.avg_window_size
        self._avg = deque([0.0] * n, maxlen=n)
        self._acc_on = self._acc_off = False
        self._acc_ts = 0
        self._pitch_state = initial_pitch_state(self.cfg.pitch_step_mode)
        self._prev_yaw = self._prev_roll = self._prev_pitch = None
        self._rev_yaw = self._rev_roll = 0
        self._have_pose = False

    def _is_armed(self, now):
        """NavEKF's lin_accel is trustworthy right now. Prefers the real
        nav_armed bit from the wire; the settle timer is only the fallback for
        the moment before the first motion-tagged frame has arrived."""
        if self._last_nav_armed is not None:
            return bool(self._last_nav_armed)
        if self._armed_at is None:
            return False
        return (now - self._armed_at) >= self.cfg.motion_arm_settle_sec

    def mark_motion_stream_on(self, now=None):
        with self._lock:
            self._armed_at = time.monotonic() if now is None else now
            self._last_nav_armed = None

    @property
    def reference(self):
        return (self._yaw0, self._pitch0, self._roll0)

    # -- calibration -------------------------------------------------------

    def begin_calibration(self):
        with self._lock:
            self._calibrating = True
            self._cal_y, self._cal_p, self._cal_r = [], [], []

    def end_calibration(self):
        """Finish the window; returns the number of samples used. With zero
        samples (glove never streamed orientation) every reference falls back
        to the scale midpoint instead of crashing."""
        with self._lock:
            self._calibrating = False
            n = len(self._cal_y)
            if n == 0:
                self._yaw0 = self._pitch0 = self._roll0 = REST_SCALED_DEFAULT
            else:
                self._pitch0 = int(sum(self._cal_p) / n)        # pitch reflects: plain mean
                self._yaw0 = circular_mean_scaled(self._cal_y)   # yaw/roll wrap: circular mean
                self._roll0 = circular_mean_scaled(self._cal_r)
            self._calibrated = True
            self._reset_runtime()
            return n

    def recenter_yaw(self):
        """Make the current heading the yaw reference (yaw has no absolute
        source without a magnetometer, so apps re-center on a musical cue)."""
        with self._lock:
            if self._have_pose:
                self._yaw0 = self._v['sy']

    # -- per-frame ---------------------------------------------------------

    def _emit(self, now, valid, calibrating, fstate, events):
        v = self._v
        return MotionFrame(
            hand=self.hand, t=now, valid=valid, calibrating=calibrating,
            calibrated=self._calibrated, armed=self._is_armed(now),
            ypr_deg=v['ypr'], scaled_yaw=v['sy'], scaled_pitch=v['sp'],
            scaled_roll=v['sr'], pitch_delta=v['dp'], accel_norm=v['an'],
            accel_scaled=v['asc'], roll_depth=v['rd'], yaw_sector=v['sec'],
            reference=self.reference, fingers=fstate, events=tuple(events))

    def update(self, frame, now=None):
        now = time.monotonic() if now is None else now
        cfg = self.cfg
        with self._lock:
            if self.fingers is not None:
                fstate, fevents = self.fingers.update(frame)
            else:
                fstate, fevents = FingerState(), []
            if 'nav_armed' in frame:
                self._last_nav_armed = frame['nav_armed']

            if self._calibrating:
                if 'arm_ypr' in frame:
                    y, p, r = frame['arm_ypr']
                    self._cal_y.append(scale_wrapped180_deg(y))
                    self._cal_p.append(scale_bounded90_deg(p))
                    self._cal_r.append(scale_wrapped180_deg(r))
                return self._emit(now, False, True, fstate, ())

            if 'arm_ypr' not in frame:
                # Orientation is independent of NavEKF motion streams.
                return self._emit(now, False, False, fstate, fevents)

            yaw_deg, pitch_deg, roll_deg = frame['arm_ypr']
            armed = self._is_armed(now)

            # Accel: NavEKF's world-frame, gravity-subtracted lin_accel. Two
            # stacked confidence mechanisms: a hard gate on nav_armed (the
            # burst detector reads a silent zero until armed) and a soft
            # ZUPT-confidence scale once armed.
            lin_accel = frame.get('lin_accel')
            if armed and lin_accel is not None:
                lax, lay, laz = lin_accel
                raw = float(np.sqrt(lax ** 2 + lay ** 2 + laz ** 2))
                accel_norm = raw * zupt_confidence_scale(
                    frame.get('zupt_mahalanobis'), cfg.zupt_gate_chisq,
                    cfg.zupt_mahalanobis_cap)
            else:
                accel_norm = 0.0
            self._avg.append(accel_norm)
            accel_scaled = input_scaling(np.mean(self._avg), max_output=127,
                                         min_input=0, max_input=cfg.accel_norm_max)

            # Pitch reflects at +/-90: direct reading, no revolution counting.
            scaled_pitch = scale_bounded90_deg(pitch_deg)
            pitch_delta = 0 if self._prev_pitch is None else scaled_pitch - self._prev_pitch
            self._prev_pitch = scaled_pitch

            # Yaw/roll wrap: unwrap, seeding the previous reading from the first
            # frame so a heading near the seam does not count a phantom
            # revolution against an assumed starting value of 0.
            ry = scale_wrapped180_deg(yaw_deg)
            rr = scale_wrapped180_deg(roll_deg)
            if self._prev_yaw is None:
                self._prev_yaw, self._prev_roll = ry, rr
            self._rev_yaw += wrap_around_count(self._prev_yaw, ry)
            self._rev_roll += wrap_around_count(self._prev_roll, rr)
            # A forearm has no business accumulating more than a few turns;
            # clamp so a run of BLE-glitch unwraps cannot drag scaled_yaw far
            # from where the sector window expects it.
            self._rev_yaw = int(np.clip(self._rev_yaw, -cfg.revolution_clamp, cfg.revolution_clamp))
            self._rev_roll = int(np.clip(self._rev_roll, -cfg.revolution_clamp, cfg.revolution_clamp))
            self._prev_yaw, self._prev_roll = ry, rr
            scaled_yaw = ry + self._rev_yaw * 127
            scaled_roll = rr + self._rev_roll * 127

            w = self._t('yaw_window', cfg.yaw_window)
            sector = -1
            for i, thr in enumerate((self._yaw0 - w, self._yaw0 + w)):
                if scaled_yaw < thr:
                    sector = i
                    break
            depth = roll_depth(scaled_roll, self._roll0, cfg.roll_depth_deadzone,
                               cfg.roll_depth_saturation)

            self._have_pose = True
            self._v = dict(ypr=(yaw_deg, pitch_deg, roll_deg), sy=scaled_yaw,
                           sp=scaled_pitch, sr=scaled_roll, dp=pitch_delta,
                           an=accel_norm, asc=int(accel_scaled), rd=depth, sec=sector)

            events = []
            self._acc_on, self._acc_off, self._acc_ts, n = schmitt_latch(
                accel_scaled, self._acc_on, self._acc_off, self._acc_ts,
                self._t('accel_trigger_thresh', cfg.accel_trigger_thresh),
                self._t('accel_trigger_hysteresis', cfg.accel_trigger_hysteresis))
            if n != 0:
                events.append(MotionEvent(self.hand, 'accel_burst', int(n), -1, sector))
            events.extend(fevents)
            self._pitch_state, step = pitch_step(
                scaled_pitch - self._pitch0, self._pitch_state,
                self._t('pitch_trigger_thresh_range', cfg.pitch_trigger_thresh_range),
                self._t('pitch_trigger_hysteresis', cfg.pitch_trigger_hysteresis),
                cfg.pitch_step_mode)
            if step != 0:
                events.append(MotionEvent(self.hand, 'pitch_step', int(step), -1, sector))
            return self._emit(now, True, False, fstate, events)


class EdgeSafeQueue(queue.Queue):
    """A bounded FIFO for enriched frames whose overflow policy protects EDGES.

    State frames are disposable (the newest supersedes the rest); edges --
    burst, step, finger on/off -- are not: a lost note-off is a stuck note.
    ``offer`` never blocks. When full it discards the OLDEST frame that carries
    no edge, so a frame with an edge is never the victim while any plain frame
    is queued. Only if EVERY queued frame carries edges does it drop the head
    and merge that frame's edges into the NEW HEAD (keeping them in order),
    so an edge is never lost and never reordered even then.

    Why not plain drop-oldest + carry the edges forward: under sustained
    overload the edge-carrying frame reaches the head, is dropped, re-carried to
    the tail, reaches the head again... and a consumer that only samples the
    head occasionally can wait unboundedly (measured: a note-off still queued
    16 s after it happened). Here its delay is bounded by queue depth divided
    by the consumer's rate.

    The check-and-drop runs under the queue's own lock, so concurrent producers
    (one dispatcher per hand feeding a merged queue) cannot interleave with it.
    It mirrors queue.Queue.put's locking and touches only ``queue``,
    ``not_full``/``not_empty`` and ``unfinished_tasks``.
    """

    @staticmethod
    def _has_edges(frame):
        m = frame.get("motion")
        return m is not None and bool(m.events)

    def offer(self, frame: dict) -> bool:
        """Enqueue ``frame`` without blocking; True if a frame was dropped."""
        with self.not_full:
            dropped = False
            if 0 < self.maxsize <= self._qsize():
                dropped = True
                victim = next((i for i, f in enumerate(self.queue)
                               if not self._has_edges(f)), None)
                if victim is not None:
                    del self.queue[victim]
                else:                       # every queued frame carries edges
                    carried = self.queue.popleft()["motion"].events
                    # Merge into the NEW HEAD (older than everything still
                    # queued), never the newest frame: merging forward would
                    # let a later edge overtake an earlier one, and a press-off
                    # delivered before its press-on is a stuck note.
                    hm = self.queue[0].get("motion") if self.queue else None
                    if hm is not None:
                        head = dict(self.queue[0])
                        head["motion"] = replace(hm, events=carried + hm.events)
                        self.queue[0] = head
                    else:                   # queue was size 1: only the new frame is left
                        nm = frame.get("motion")
                        if nm is not None:
                            frame = dict(frame)
                            frame["motion"] = replace(nm, events=carried + nm.events)
            self._put(frame)
            self.unfinished_tasks += 1
            self.not_empty.notify()
            return dropped


@dataclass(frozen=True)
class PoseSample:
    """Decoded pose data from one firmware sample.

    position/velocity/orientation/lin_accel are None whenever `motion` (and,
    for orientation, `orientation`) wasn't requested.

    orientation is the RAW NavEKF quaternion (w, x, y, z), world<-sensor.
    arm_ypr / R_render are derived from it by the Orientation section of this module:
      - arm_ypr: (yaw, pitch, roll) degrees in the ARM frame. This is the
        angle naming every consumer should use; the sensor's own axes are
        mounted with the long axis on +Y, so raw firmware "pitch"/"roll"
        are swapped relative to the arm.
      - R_render: world<-sensor DCM (3x3 numpy array) for drawing.
    Both are None when orientation was not requested / nav_quat absent.

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
    arm_ypr: Optional[Vector3] = None
    R_render: Optional[object] = None
    motion: Optional[MotionFrame] = None


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
    """Reference-counted pose stream manager around one ``Reader``.

    If ``reader`` is omitted, a Reader is created with IMU parsing and
    ``nav_quat_enabled=True``. Call :meth:`start` once. Then either:
      - :meth:`subscribe` for a private queue of enriched frames (choir,
        twin: anything that needs flex/press/etc. alongside pose), or
      - :meth:`latest` / :meth:`wait_for_sample` for the newest
        :class:`PoseSample` (pose-only consumers).
    """

    SUBSCRIBER_QSIZE = 64

    def __init__(self, reader: Optional[Reader] = None, hands=("l", "r"),
                 reader_kwargs=None, config: Optional["MotionConfig"] = None,
                 tuning: Optional["LiveControls"] = None, diagnostics: bool = False):
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
        # hand -> list of (queue, name). Guarded by _lock.
        self._subscribers = {hand: [] for hand in self.hands}
        # Per-hand counters for diagnostics (see stats()).
        self._stats = {
            hand: {"frames": 0, "no_nav_quat": 0, "dropped": 0, "t0": None}
            for hand in self.hands
        }
        # -- motion pipeline ------------------------------------------------
        self.config = config or MotionConfig()
        self.tuning = tuning if tuning is not None else LiveControls(
            vibrato_enabled=self.config.vibrato_enabled)
        for _k, _v in self.config.tuning_seed().items():     # a shared LiveControls
            if _v is not None and self.tuning.get_threshold(_k) is None:   # keeps values
                self.tuning.set_threshold(_k, _v)                          # already set
        self.motion = {
            hand: HandMotion(
                hand, self.config, self.tuning,
                FingerTriggers(hand, self.config.sensors_for(hand),
                               self.config.finger_thresholds,
                               self.config.finger_hysteresis, self.tuning))
            for hand in self.hands
        }
        self._latest_motion = {hand: None for hand in self.hands}
        self._last_frame_at = {hand: None for hand in self.hands}
        self._merged = []            # (queue, name): one queue fed by ALL hands
        self._diag = HandDiag(self.hands, interval_sec=2.0, enabled=diagnostics)
        self._diag_lock = Lock()
        self._watchdog = None
        self._started_at = None
        self._motion_error_reported = set()
        self._align_deg = {hand: None for hand in self.hands}
        self._align_t = {hand: None for hand in self.hands}

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

    # -- lifecycle ---------------------------------------------------------

    def start(self, start_reader: bool = True) -> None:
        """Start the reader (unless the caller already did) and the per-hand
        dispatchers. Subscribe BEFORE calling start() if a consumer must see
        the very first frame; late subscribers simply miss earlier frames.
        """
        if start_reader:
            self.reader.start_readers()
        self._stop_event = Event()
        self._started_at = time.monotonic()
        for hand in self.hands:
            thread = Thread(target=self._dispatch, args=(hand,), daemon=True,
                            name=f"PoseProvider-{hand}")
            self._dispatchers[hand] = thread
            thread.start()
        self._watchdog = Thread(target=self._watch_stale, daemon=True,
                                name="PoseProvider-watchdog")
        self._watchdog.start()

    def stop(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        self.reader.stop_readers()
        for thread in self._dispatchers.values():
            thread.join(timeout=1.0)
        self._dispatchers.clear()
        if self._watchdog is not None:
            self._watchdog.join(timeout=1.0)
            self._watchdog = None

    def bring_up(self, motion: bool = True, nav_quat: bool = False,
                 gyro_off: bool = True) -> dict:
        """Connect and start the whole data path in the ONLY order that works
        with a sole-drain provider. Apps call this instead of sequencing it:

          1. reader.start_readers()   connect + gyro-calibrate (Reader owns this)
          2. USE_GYRO_OFF             pin the imu slot's accel/gyro encoding
                                      (use_gyro is a RAM-only firmware flag an
                                      aborted calibration can leave set)
          3. confirmed stream enables, all hands concurrently, both streams in
             ONE timeout budget -- they read the parser queue directly to
             confirm, so the dispatchers must not exist yet
          4. start(): the dispatchers become the sole drain

        Subscribe BEFORE calling this if a consumer must see the first frame.
        Returns {hand: bool} -- True when every requested stream was confirmed.
        A hand that never confirms is named on the console and left running
        (its features will simply stay invalid); it does not block the others.
        """
        self.reader.start_readers()
        if gyro_off:
            for hand in self.hands:
                if hand in self.reader.threads:
                    try:
                        self.reader.send_command(hand, "USE_GYRO_OFF")
                    except Exception as exc:
                        print(f"Warning: could not send USE_GYRO_OFF on {hand}: {exc}")
        results = {}
        if motion or nav_quat:
            results = self.ensure_streams_all(motion=motion, orientation=nav_quat)
            for hand in self.hands:
                if motion:
                    self.motion[hand].mark_motion_stream_on()
                if not results.get(hand, False):
                    what = " / ".join(n for n, on in (("MOTION_STREAM_ON", motion),
                                                      ("NAV_QUAT_STREAM_ON", nav_quat)) if on)
                    print(f"WARNING: hand '{hand}' did not confirm {what} -- its "
                          f"accel-burst, pitch/roll/yaw features and gestures will "
                          f"be invalid until it streams.")
        self.start(start_reader=False)
        return results

    def calibrate(self, window_sec: Optional[float] = None, hands=None) -> dict:
        """Collect each hand's reference pose (yaw0, pitch0, roll0) over
        ``window_sec``: hold the neutral pose. Runs on the dispatcher-fed
        trackers, so it needs bring_up()/start() first. No events are emitted
        during the window. Returns {hand: sample_count}; a hand with zero
        samples falls back to the scale midpoint and is named on the console.
        """
        window = self.config.calibration_window_sec if window_sec is None else window_sec
        hands = [h for h in (hands or self.hands) if h in self.motion]
        print(f"Calibrating for {window:.1f}s -- hold your neutral pose...")
        for h in hands:
            self.motion[h].begin_calibration()
        time.sleep(window)
        counts = {}
        for h in hands:
            counts[h] = self.motion[h].end_calibration()
            if counts[h] == 0:
                print(f"WARNING: hand '{h}' produced zero calibration samples -- using "
                      f"midpoint defaults. This hand's roll/pitch/yaw triggers will be "
                      f"inaccurate until it's reconnected and recalibrated.")
            else:
                y0, p0, r0 = self.motion[h].reference
                print(f"Calibrated [{h}] n={counts[h]}: yaw0={y0} pitch0={p0} roll0={r0}")
        return counts

    def recenter_yaw(self, hands=None) -> None:
        """Make each hand's current heading its yaw reference."""
        for h in (hands or self.hands):
            if h in self.motion:
                self.motion[h].recenter_yaw()

    def latest_motion(self, hand: str) -> Optional[MotionFrame]:
        with self._lock:
            return self._latest_motion[hand]

    def fingers(self, hand: str) -> FingerState:
        """Latched finger levels as of the newest processed frame of ``hand``."""
        m = self.latest_motion(hand)
        return m.fingers if m is not None else FingerState()

    def reference(self, hand: str):
        return self.motion[hand].reference

    def _watch_stale(self) -> None:
        cfg = self.config
        warned = {}
        while self._stop_event is not None and not self._stop_event.wait(0.25):
            now = time.monotonic()
            for hand in self.hands:
                last = self._last_frame_at[hand]
                age = now - (last if last is not None else self._started_at)
                if age > cfg.stale_warn_sec and \
                        now - warned.get(hand, -1e9) >= cfg.stale_warn_repeat_sec:
                    warned[hand] = now
                    print(f"WARNING: hand '{hand}' has produced no frame in "
                          f"{age:.1f}s -- its roll/pitch/yaw trigger state is frozen "
                          f"at its last known reading.")

    # -- subscription (fan-out) -------------------------------------------

    def subscribe(self, hand: str, maxsize: Optional[int] = None,
                  name: str = "") -> "queue.Queue":
        """Return a private queue of enriched frames for ``hand``.

        Each subscriber gets EVERY frame (a copy of the reference, not a
        share of the stream), so subscribers never steal from one another.
        A slow subscriber drops its own oldest frame when full; it can never
        block the dispatcher or another subscriber.
        """
        if hand not in self._subscribers:
            raise KeyError(f"unknown hand: {hand}")
        q = EdgeSafeQueue(maxsize=maxsize or self.SUBSCRIBER_QSIZE)
        with self._lock:
            self._subscribers[hand].append((q, name))
        return q

    def unsubscribe(self, hand: str, q: "queue.Queue") -> None:
        with self._lock:
            self._subscribers[hand] = [
                (qq, n) for qq, n in self._subscribers[hand] if qq is not q
            ]

    def subscribe_merged(self, maxsize: Optional[int] = None, name: str = "") -> "queue.Queue":
        """One queue fed by EVERY hand, in arrival order; each frame carries
        ``hand``. This is what a single-threaded app loop wants. Events are
        per-hand: read ``frame['motion'].events`` and use each event's own
        ``hand``, not the frame's, because edges carried over from a dropped
        frame of the other hand can ride along."""
        q = EdgeSafeQueue(maxsize=maxsize or self.SUBSCRIBER_QSIZE)
        with self._lock:
            self._merged.append((q, name))
        return q

    def unsubscribe_merged(self, q: "queue.Queue") -> None:
        with self._lock:
            self._merged = [(qq, n) for qq, n in self._merged if qq is not q]

    def subscribe_all(self, maxsize: Optional[int] = None, name: str = ""):
        """Convenience: {hand: queue} for every hand."""
        return {h: self.subscribe(h, maxsize=maxsize, name=name)
                for h in self.hands}

    # -- stream demand -----------------------------------------------------

    def request(self, hand: str, motion=False, orientation=False):
        if hand not in self._counts:
            raise KeyError(f"unknown hand: {hand}")
        if not motion and not orientation:
            raise ValueError("request must select motion and/or orientation")
        return _PoseRequest(self, hand, bool(motion), bool(orientation))

    def ensure_streams(self, hand: str, motion: bool = False,
                       orientation: bool = False) -> bool:
        """Blocking, confirmed enable (retries until data actually shows up).

        ``request()`` is fire-and-forget: send_command has no ACK, so a
        dropped BLE write leaves the stream silently off (see
        Reader.ensure_stream_enabled). Use this instead when the caller
        depends on the data arriving -- the choir does.

        It must run BEFORE :meth:`start` spawns the dispatchers: it reads the
        parser queue directly to confirm, and would otherwise race the
        dispatcher for frames. Also bumps the ref-count so a later release
        pairs correctly. Returns True only if every requested stream was
        confirmed.
        """
        if self._stop_event is not None and not self._stop_event.is_set():
            raise RuntimeError(
                "ensure_streams() must be called before start(): the "
                "dispatcher already owns the parser queue")
        # The confirm blocks for up to timeout x retries (9 s with a dead
        # glove). It must NOT run under self._lock: that would serialise
        # concurrent per-hand confirms (see ensure_streams_all) and stall
        # latest()/counts() callers on other threads for the whole wait. The
        # confirm only touches this hand's parser queue, so no lock is needed;
        # the lock guards only the ref-count bump.
        cmds = []
        if motion:
            cmds.append(("MOTION_STREAM_ON", "lin_accel"))
        if orientation:
            cmds.append(("NAV_QUAT_STREAM_ON", "nav_quat"))
        if not cmds:
            return True
        ok = self._confirm_together(hand, cmds)
        with self._lock:
            if motion:
                self._counts[hand]["motion"] += 1
            if orientation:
                self._counts[hand]["orientation"] += 1
        return ok

    def _confirm_together(self, hand: str, cmds_keys, timeout=3.0, retries=3) -> bool:
        """Send several stream-ON commands at once, then wait ONCE until every
        expected data key has been seen on this hand's parser queue.

        Reader.ensure_stream_enabled confirms one stream at a time, so asking
        for motion and orientation costs two full timeout budgets on a silent
        glove (2 x 3 s x 3 = 18 s). The two commands are independent (see the
        firmware's send_motion / send_nav_quat flags), so send both and
        confirm both in a single budget. Same semantics as the Reader method:
        drain first so every sample read postdates the command, resend on a
        timeout, and report failure rather than raising.
        """
        parser_q = self.reader.threads[hand]["parser"].getQ()
        needed = {key for _cmd, key in cmds_keys}
        for attempt in range(1, retries + 1):
            while True:
                try:
                    parser_q.get_nowait()
                except queue.Empty:
                    break
            for cmd, _key in cmds_keys:
                self.reader.send_command(hand, cmd)
            seen = set()
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    sample = parser_q.get(timeout=0.5)
                except queue.Empty:
                    continue
                seen |= {k for k in needed if k in sample}
                if seen >= needed:
                    return True
            missing = ", ".join(sorted(needed - seen))
            print(f"Warning: [{hand.upper()}] stream(s) not confirmed within "
                  f"{timeout}s (attempt {attempt}/{retries}): {missing}"
                  f"{' -- retrying' if attempt < retries else ''}.")
        return False

    def ensure_streams_all(self, hands=None, motion: bool = False,
                           orientation: bool = False) -> dict:
        """Confirm streams on every hand CONCURRENTLY; returns {hand: bool}.

        Doing this serially costs (timeout x retries) per stream per hand, so
        one dead or slow glove stalls every hand behind it (measured: ~18 s
        to bring up the app with the left glove silent, and the right glove
        was healthy the whole time). Each hand's confirm only reads that
        hand's own parser queue, so they are independent and safe to overlap.
        Same precondition as :meth:`ensure_streams`: call before start().
        """
        hands = [h for h in (hands or self.hands) if h in self._counts]
        results = {}

        def _one(h):
            try:
                results[h] = self.ensure_streams(h, motion=motion,
                                                 orientation=orientation)
            except Exception as exc:   # never let one hand's failure hide another's result
                print(f"Warning: [{h.upper()}] stream enable raised "
                      f"{type(exc).__name__}: {exc}")
                results[h] = False

        threads = [Thread(target=_one, args=(h,), daemon=True,
                          name=f"PoseEnable-{h}") for h in hands]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        return results

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

    # -- latest-sample access ---------------------------------------------

    def latest(self, hand: str) -> Optional[PoseSample]:
        with self._lock:
            return self._latest[hand]

    def wait_for_sample(self, hand: str, timeout=None) -> Optional[PoseSample]:
        with self._condition:
            previous = self._latest[hand]
            self._condition.wait_for(lambda: self._latest[hand] is not previous,
                                     timeout=timeout)
            return self._latest[hand]

    def stats(self, hand: str):
        """Frames dispatched, frames missing nav_quat, subscriber-queue
        drops, and elapsed seconds since the first frame."""
        with self._lock:
            s = dict(self._stats[hand])
        s["elapsed"] = (time.monotonic() - s["t0"]) if s["t0"] else 0.0
        return s

    # -- dispatch ----------------------------------------------------------

    def enrich(self, hand: str, sample: dict, now: Optional[float] = None) -> dict:
        """Shallow-copy ``sample`` and attach the firmware's pose.

        Adds: hand; fw_ypr (firmware's own yaw/pitch/roll); arm_ypr (per
        config.sensor_mounting; what apps read); R_render (world<-sensor DCM of
        the firmware attitude); and, in the display frame, position / velocity /
        lin_accel (the NavEKF originals stay as *_nav) plus nav_align_deg.

        Public so offline/replay code can run recorded frames through the exact
        path the live dispatcher uses.
        """
        frame = dict(sample)
        frame["hand"] = hand
        mounting = per_hand(self.config.sensor_mounting, hand, DEFAULT_MOUNTING)
        pitch_sign = per_hand(self.config.pitch_sign, hand, 1)
        pose = firmware_pose(frame.get("imu"), mounting, pitch_sign)
        if pose is not None:
            frame["fw_ypr"] = pose["fw_ypr"]
            frame["arm_ypr"] = pose["arm_ypr"]
            frame["R_render"] = pose["R"]
            frame["sensor_mounting"] = mounting
            frame["pitch_sign"] = pitch_sign
        delta = self._nav_alignment(hand, frame, time.monotonic() if now is None else now)
        frame["nav_align_deg"] = delta
        for key in ("position", "velocity", "lin_accel"):
            vec = frame.get(key)
            if vec is not None:
                frame[key + "_nav"] = vec
                if delta is not None:
                    frame[key] = rotate_z(vec, delta)
        return frame

    def _nav_alignment(self, hand: str, frame: dict, now: float):
        """Yaw (deg) that carries NavEKF's world frame into the display frame:
        display_yaw - nav_yaw. Updated only while NavEKF is armed (its
        quaternion is the identity before that) and both yaws are known; held
        otherwise. Snaps on a step larger than align_snap_deg, else low-pass."""
        cfg = self.config
        cur = self._align_deg[hand]
        if not cfg.align_nav_to_display:
            return None
        fw, nav_quat = frame.get("fw_ypr"), frame.get("nav_quat")
        if fw is None or nav_quat is None or not frame.get("nav_armed"):
            return cur
        d = wrap180(fw[0] - quat_yaw_deg(nav_quat))
        if cur is None or abs(wrap180(d - cur)) > cfg.align_snap_deg:
            cur = d
        else:
            t_prev = self._align_t[hand]
            dt = 0.01 if t_prev is None else min(max(now - t_prev, 1e-3), 0.1)
            cur = wrap180(cur + dt / (cfg.align_tau_sec + dt) * wrap180(d - cur))
        self._align_deg[hand] = cur
        self._align_t[hand] = now
        return cur

    @staticmethod
    def _to_pose_sample(sample: dict, frame: dict) -> PoseSample:
        return PoseSample(
            position=tuple(frame["position"]) if "position" in frame else None,
            velocity=tuple(frame["velocity"]) if "velocity" in frame else None,
            orientation=tuple(sample["nav_quat"]) if "nav_quat" in sample else None,
            lin_accel=tuple(frame["lin_accel"]) if "lin_accel" in frame else None,
            # .get(...) with a None default preserves "not requested/not
            # available" rather than defaulting nav_armed to a misleading False.
            nav_armed=sample.get("nav_armed"),
            nav_zupt_active=sample.get("nav_zupt_active"),
            nav_zupt_starved=sample.get("nav_zupt_starved"),
            zupt_mahalanobis=sample.get("zupt_mahalanobis"),
            seq=sample.get("seq", 0),
            device_us=sample.get("device_us", 0),
            arm_ypr=frame.get("arm_ypr"),
            R_render=frame.get("R_render"),
            motion=frame.get("motion"),
        )

    def _publish(self, hand: str, frame: dict) -> None:
        with self._lock:
            subs = list(self._subscribers[hand]) + list(self._merged)
        dropped = 0
        for q, _name in subs:
            dropped += q.offer(frame)
        if dropped:
            with self._lock:
                self._stats[hand]["dropped"] += dropped

    def _dispatch(self, hand):
        parser_q = self.reader.threads[hand]["parser"].getQ()
        motion = self.motion[hand]
        while self._stop_event is not None and not self._stop_event.is_set():
            try:
                sample = parser_q.get(timeout=0.1)
            except queue.Empty:
                continue
            now = time.monotonic()
            frame = self.enrich(hand, sample, now)
            try:
                mf = motion.update(frame, now)
            except Exception as exc:   # a bad frame must never kill the dispatcher
                if hand not in self._motion_error_reported:
                    self._motion_error_reported.add(hand)
                    print(f"WARNING: motion update failed on hand '{hand}': "
                          f"{type(exc).__name__}: {exc} (further errors suppressed)")
                mf = MotionFrame(hand=hand, t=now, valid=False)
            frame["motion"] = mf
            pose = self._to_pose_sample(sample, frame)
            with self._condition:
                st = self._stats[hand]
                if st["t0"] is None:
                    st["t0"] = now
                st["frames"] += 1
                if "nav_quat" not in sample:
                    st["no_nav_quat"] += 1
                self._latest[hand] = pose
                self._latest_motion[hand] = mf
                self._last_frame_at[hand] = now
                self._condition.notify_all()
            if self._diag.enabled:
                with self._diag_lock:
                    self._diag.frame(hand, frame)
                    if mf.valid:
                        self._diag.angles(hand, mf.ypr_deg[1], mf.ypr_deg[2])
                        self._diag.accel(hand, mf.accel_scaled)
                    self._diag.maybe_report(
                        self.tuning.get_threshold('accel_trigger_thresh', self.config.accel_trigger_thresh)
                        + self.tuning.get_threshold('accel_trigger_hysteresis', self.config.accel_trigger_hysteresis))
            self._publish(hand, frame)
