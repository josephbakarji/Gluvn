#!/usr/bin/env python3
"""
Mahony AHRS Validation Suite — Gluvn-M5

Collects quantitative metrics to verify the effectiveness of the firmware's
Mahony filter against four criteria:

  1. Static drift      — yaw/pitch/roll change while the glove is motionless.
                          Ground truth: orientation must not change; any
                          change is drift by definition.
  2. Step-response lag — time to settle after a fast, deliberate rotation to
                          a new fixed pose held against a right-angle
                          reference (e.g. a set square, wall corner, or table
                          edge). Ground truth: the operator's own held pose.
  3. Overshoot         — how far the filter's estimate swings past the final
                          settled value before stabilizing, expressed as a
                          percentage of the step size.
  4. Static accuracy   — steady-state error against a known reference angle
                          (e.g. 0 deg flat on a table, or 90 deg against a
                          set square), entered by the operator before each
                          hold.

IMPORTANT — this script cannot invent ground truth it doesn't have. It has
no external optical/magnetic tracking reference. Static accuracy and lag
are only as good as the physical reference the operator holds the glove
against (a set square, a level surface, a protractor jig). Report this
limitation alongside the numbers: these are single-IMU, self-referenced
benchmarks, not validated against an independent motion-capture ground
truth.

Additionally piggybacks on the drift test to reconstruct, from the raw
gyro telemetry, the SAME gyro pre-filter + EWMA-variance pipeline
MahonyAHRS.h uses for its stationary-variance gate (see FW_* constants
below). This lets you pick real thresholds from bench data instead of
trusting the firmware defaults, without touching the firmware itself.

imu[3:6] is a shared slot: it streams raw accel by default and switches
to raw gyro (dps) while USE_GYRO_ON is active on the firmware.
m5stick_firmware.ino encodes an explicit sensor_type flag (0=accel,
1=gyro) in imu0's LSB (bit 0), instead of leaving the host to infer the
channel from USE_GYRO_ON/OFF round-trip timing. port_read.py's
ParseSerial now decodes this unconditionally into data['sensor_type']
(Pre-Priority #2) -- that field is the SOLE source of truth for channel
interpretation in test_drift below; the physical-plausibility check
there is a diagnostic cross-check only and never decides the channel
when sensor_type is present.

Two real bugs were found and fixed in this pipeline before any of the
above could be trusted: (1) an earlier firmware revision added a whole
new byte to the wire frame for sensor_type, which grew the payload
length and silently broke any host parser still sized for the old
length -- superseded by the imu0-LSB encoding, which needs no frame-
length change at all; (2) port_read.py's ParseSerial had 'imu' mapped
to slice(6,9) -- just (imu0,imu1,imu2) -- silently dropping yaw/pitch/
roll entirely, so decode_euler(data['imu'][0:3]) was actually decoding
raw accel/gyro counts through the orientation-angle formula. Neither
was a firmware swap or an AHRS problem; both were host-side field-
mapping bugs that produced plausible-looking wrong numbers. See
port_read.py's ParseSerial docstring and slices comment for specifics.

capture_stream() below follows a deterministic init sequence (connect ->
verify stream -> set sensor mode -> wait for confirmed sensor_type ->
optionally recalibrate -> wait for cal-complete -> discard transient
frames -> begin capture) rather than fixed sleeps, specifically so a USB
run and a BLE run start from comparably-known state and can be compared
against each other. wait_for_recalibration() blocks on RECALIBRATE_GYRO
via streaming-pause/resume detection (see its docstring for why not an
ASCII confirmation) -- verified to work identically over USB and BLE.

capture_stream(..., stream_gyro=True) sends the USE_GYRO_ON toggle before
recording and reverts it after, mirroring calibrate_eeprom.py's pattern;
only test_drift uses it, since that's the only test consuming gx/gy/gz
today. It only covers the gyro half of the firmware's stationary gate --
the accel half (needed for accelConfidence/jerk-style diagnostics) is
reachable in principle (same slot, gyro mode simply off, sensor_type=0)
but has no validated decode formula exercised here yet; that's future
work, not something to guess at.

AHRS state across tests/reconnects (audited from m5stick_firmware.ino):
quaternion orientation, gyro bias, and accel bias are firmware-side RAM
state that persists for as long as the device stays powered -- a Python-
side reconnect (BLE disconnect/reconnect, or a fresh capture_stream()
call) does NOT reset any of it. Only RESET_YAW zeros the yaw integrator
specifically; there is no RESET_PITCH/RESET_ROLL/RESET_QUATERNION
command. The quaternion is only reinitialized to identity at an actual
device boot (setup()) or implicitly re-anchored by a fresh
RECALIBRATE_GYRO (which re-measures gyro bias but does not touch the
quaternion itself). A genuinely "fresh" comparison between runs means
either a real power cycle or an explicit recalibrate=True, not just a
new script invocation.

Usage:
    python test_mahony_ahrs.py --hand r --test drift   --duration 30
    python test_mahony_ahrs.py --hand r --test drift   --duration 30 --gyro-var-threshold 6.0
    python test_mahony_ahrs.py --hand r --test step     --target-angle 90 --axis roll
    python test_mahony_ahrs.py --hand r --test accuracy --reference-angle 0 --axis pitch
    python test_mahony_ahrs.py --hand r --test all
"""

import argparse
import time
import queue
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

from core.port_read import Reader

# ── Firmware decode constants (must match m5stick_firmware.ino exactly) ────
# yaw_cal   = (yaw   + 180) * 32767 / 180
# pitch_cal = (pitch + 180) * 32767 / 180   -- was +-90 before the Priority #1
#                                              fix (MahonyAHRS.h's computeEuler,
#                                              pitch was asin-bounded then)
# roll_cal  = (roll  + 180) * 32767 / 180
def decode_euler(imu_slice):
    """imu_slice = (yaw_cal, pitch_cal, roll_cal) as received from firmware."""
    yaw_cal, pitch_cal, roll_cal = imu_slice[0], imu_slice[1], imu_slice[2]
    yaw   = yaw_cal   * 180.0 / 32767.0 - 180.0
    pitch = pitch_cal * 180.0 / 32767.0 - 180.0
    roll  = roll_cal  * 180.0 / 32767.0 - 180.0
    return yaw, pitch, roll


def decode_gyro(imu_slice):
    """imu_slice = raw (gx, gy, gz) register values, converted to dps.
    Same decode as calibrate_eeprom.py's live gyro-bias capture -- keep
    the two in sync by hand, there is no shared source of truth."""
    return [(v * 4000.0 / 65535.0) - 2000.0 for v in imu_slice]


def decode_accel(imu_slice, full_scale_g=2.0):
    """imu_slice = raw (ax, ay, az) register values, converted to g.
    Same unsigned-16-bit-to-symmetric-range transform as decode_gyro,
    with acceleration's +-full_scale_g substituted for gyro's +-2000 dps.

    full_scale_g=2.0 is INFERRED, not firmware-confirmed: the observed
    raw register value on a stationary, level drift-test capture
    clustered at v ~ 49152 (exactly 0.75 * 65535), which decodes to
    exactly 1.0000 g under a +-2g transform and to ~1000 dps (physically
    implausible) under the gyro transform. See the dual-decode
    plausibility check in test_drift, which re-verifies this against
    |accel| ~ 1g on every run rather than assuming it holds."""
    return [(v * (2.0 * full_scale_g) / 65535.0) - full_scale_g for v in imu_slice]


AXIS_INDEX = {'yaw': 0, 'pitch': 1, 'roll': 2}

# ── Mirrors of MahonyAHRS.h internal constants ──────────────────────────
# No shared source of truth across the C++ firmware and this script --
# if you retune these in MahonyAHRS.h, update the defaults here too.
FW_GYRO_FILTER_TAU     = 0.02   # s, gyro pre-filter time constant
FW_VARIANCE_WINDOW_TAU = 0.5    # s, EWMA variance window
FW_GYRO_VAR_THRESHOLD  = 9.0    # (deg/s)^2, default stationary variance gate

# A stationary MEMS gyro's raw output -- even with an uncalibrated static
# bias -- should never sustain hundreds of dps. If the mean |gyro| over a
# nominally-motionless capture exceeds this, imu[3:6] almost certainly
# isn't gyro dps for this channel -- treat it as a decode-hypothesis
# mismatch, not real motion, and don't run gyro diagnostics on it.
GYRO_PLAUSIBILITY_LIMIT_DPS = 100.0

# A stationary, roughly level accelerometer should read |accel| ~ 1.0 g
# regardless of which axis gravity happens to land on. This tolerance
# accepts some tilt/noise without accepting "this obviously isn't accel
# data either."
ACCEL_FULL_SCALE_G = 2.0
ACCEL_PLAUSIBILITY_TOLERANCE_G = 0.4


# --- Capture ----------------------------------------------------------

GYRO_MODE_SETTLE_TIMEOUT = 3.0   # s, upper bound for the mode-settle handshake below
VERIFY_STREAM_TIMEOUT    = 5.0   # s, upper bound for "did any data arrive at all"

RECAL_SILENCE_THRESHOLD_S = 1.0   # s; normal inter-frame gap is ~10ms, calibration blocks ~3-4s
RECAL_TIMEOUT_S           = 15.0  # s; generous upper bound for recalibrate-and-resume
CAL_SETTLE_TIME_S         = 2.0   # s; post-confirmation settle before recording starts
ANALYZE_LAST_S = 30.0


def _wait_for_sensor_type(q, expected, timeout=GYRO_MODE_SETTLE_TIMEOUT):
    """Pre-Priority #4: deterministic mode-settled handshake, replacing a
    fixed sleep(). Drains frames until one with sensor_type == expected is
    seen, discarding every frame along the way (including the confirming
    one) -- the caller starts capturing fresh from the next frame after
    this returns. sensor_type is read straight from data['sensor_type'],
    which port_read.py now populates unconditionally (Pre-Priority #2:
    it's the sole source of truth, not a plausibility guess).

    Returns True if confirmed within timeout, False otherwise (caller
    should warn and decide whether to proceed anyway).
    """
    deadline = time.perf_counter() + timeout
    seen = set()
    while time.perf_counter() < deadline:
        try:
            data = q.get(timeout=max(0.05, deadline - time.perf_counter()))
        except queue.Empty:
            break
        st = data.get('sensor_type')
        if st is not None:
            seen.add(int(st))
            if int(st) == expected:
                return True
    if seen:
        print(f"  WARNING: mode-settle handshake timed out after {timeout:.1f}s "
              f"-- saw sensor_type values {sorted(seen)}, never {expected}.")
    else:
        print(f"  WARNING: mode-settle handshake timed out after {timeout:.1f}s "
              f"-- no sensor_type field seen at all (check port_read.py / "
              f"that frames are arriving).")
    return False


def wait_for_recalibration(reader, hand, q, timeout=RECAL_TIMEOUT_S):
    """Pre-Priority #0.2/#0.3: send RECALIBRATE_GYRO and confirm completion
    before letting a test proceed, over EITHER transport -- deterministically.

    Primary signal: m5stick_firmware.ino's gyroCalibToggle bit (imu1's LSB,
    decoded by port_read.py as data['gyro_calib_toggle']), which flips
    exactly once when ANY gyro calibration completes -- boot or
    RECALIBRATE_GYRO, both funnel through the same firmware function. This
    reads the toggle's value BEFORE sending the command, then waits for a
    frame where it's DIFFERENT. That's ground truth, not an inference --
    "do not falsely report calibration as fresh" is satisfied by construction,
    not by a timing heuristic.

    Fallback (explicitly flagged as unverified when used): if no frame
    carries a 'gyro_calib_toggle' field at all -- an unpatched port_read.py,
    most likely -- fall back to the coarser streaming-pause/resume signal
    from earlier: runStartupGyroCalibration() never calls sendData() while
    running, so calibration is externally visible as a gap in the frame
    stream. This is real evidence, just weaker than a state change that can
    only happen once per completed calibration.
    """
    # Establish the toggle's value before requesting recalibration. Short
    # window -- if nothing shows up quickly, toggle-based confirmation
    # isn't available and the fallback below takes over.
    prev_toggle = None
    probe_deadline = time.perf_counter() + 2.0
    while time.perf_counter() < probe_deadline and prev_toggle is None:
        try:
            data = q.get(timeout=max(0.05, probe_deadline - time.perf_counter()))
        except queue.Empty:
            break
        prev_toggle = data.get('gyro_calib_toggle')

    reader.send_command(hand, "RECALIBRATE_GYRO")
    if prev_toggle is not None:
        print("  Sent RECALIBRATE_GYRO, waiting for gyro_calib_toggle to flip...")
    else:
        print("  Sent RECALIBRATE_GYRO. No gyro_calib_toggle field seen (check "
              "port_read.py is current) -- falling back to pause/resume timing, "
              "which will be reported as UNVERIFIED.")

    t_sent = time.perf_counter()

    if prev_toggle is not None:
        deadline = t_sent + timeout
        while time.perf_counter() < deadline:
            try:
                data = q.get(timeout=max(0.05, deadline - time.perf_counter()))
            except queue.Empty:
                continue
            cur = data.get('gyro_calib_toggle')
            if cur is not None and cur != prev_toggle:
                print(f"  Recalibration CONFIRMED (~{time.perf_counter() - t_sent:.1f}s: "
                      f"gyro_calib_toggle flipped {prev_toggle} -> {cur}).")
                return True
        print(f"  WARNING: gyro_calib_toggle never changed within {timeout:.1f}s "
              f"after RECALIBRATE_GYRO. Treat calibration freshness as "
              f"UNVERIFIED, not fresh.")
        return False

    # Fallback path: no toggle field available at all.
    last_frame_t = t_sent
    saw_gap = False
    while time.perf_counter() - t_sent < timeout:
        try:
            q.get(timeout=0.2)
            now = time.perf_counter()
            if saw_gap:
                print(f"  Recalibration LIKELY complete (~{now - t_sent:.1f}s: "
                      f"streaming paused then resumed) -- UNVERIFIED, no "
                      f"gyro_calib_toggle field was available to confirm it.")
                return True
            if now - last_frame_t > RECAL_SILENCE_THRESHOLD_S:
                saw_gap = True
            last_frame_t = now
        except queue.Empty:
            if not saw_gap and (time.perf_counter() - last_frame_t > RECAL_SILENCE_THRESHOLD_S):
                saw_gap = True

    print(f"  WARNING: never observed a streaming pause+resume within "
          f"{timeout:.1f}s after RECALIBRATE_GYRO either. Treat this run's "
          f"calibration freshness as UNVERIFIED, not fresh.")
    return False


def _sanity_check_samples(samples):
    """Pre-Priority #14: the telemetry pipeline has already produced one
    silently-wrong-but-plausible-looking result this session (roll
    decoding as a suspiciously perfect 0.000 std/drift, which turned out
    to be a Reader field-mapping bug, not excellent AHRS performance).
    Cheap structural checks here, run on every capture, to catch that
    CLASS of failure early rather than trusting a clean-looking number.
    """
    if len(samples) < 2:
        return
    roll = np.array([s[3] for s in samples])
    seqs = np.array([s[9] for s in samples])
    warnings = []

    if np.ptp(roll) < 1e-9:
        warnings.append("roll channel is EXACTLY constant across the whole "
                         "capture -- verify payload decoding before trusting "
                         "this as 'zero drift'.")
    if np.all(roll == 0.0):
        warnings.append("roll channel is all zeros -- likely a missing/"
                         "mis-mapped field, not a genuinely level glove.")
    n_repeated_seq = int(np.sum(np.diff(seqs.astype(np.int32)) == 0))
    if n_repeated_seq > 0:
        warnings.append(f"{n_repeated_seq} consecutive samples share the same "
                         f"seq -- possible duplicate/stuck frames.")

    for w in warnings:
        print(f"  WARNING: {w}")


def capture_stream(hand, duration, use_ble=True, port=None, ble_name=None,
                    stream_gyro=False, recalibrate=False,
                    force_ble=False, analyze_last_s=None):
    """Capture (t, yaw, pitch, roll, raw0, raw1, raw2, sensor_type,
    device_us, seq) tuples for `duration` seconds.

    Pre-Priority #0.4 deterministic sequence (order matters -- mode is set
    BEFORE recalibration, not after: calibration doesn't touch use_gyro,
    so once it completes and streaming resumes, frames are already in the
    requested mode with no second mode-switch-and-wait needed):
      CONNECT -> VERIFY STREAM -> SET SENSOR MODE -> WAIT FOR CONFIRMED
      SENSOR_TYPE -> OPTIONALLY RECALIBRATE -> WAIT FOR CAL COMPLETE ->
      SETTLE (CAL_SETTLE_TIME_S) -> DISCARD transient frames (including
      the settle-window backlog) -> BEGIN CAPTURE
    every step explicit and bounded-timeout rather than a fixed sleep,
    except the settle itself, which is deliberately a fixed wait -- it's
    not confirming anything, just giving the filter time to move past
    the reset transient calibration confirmation itself already signals.

    raw0/raw1/raw2 are the RAW, UNDECODED register values from imu[3:6],
    valid only when stream_gyro=True (NaN otherwise).

    sensor_type (Pre-Priority #0.7/#2): read directly from data['sensor_type'],
    which port_read.py decodes unconditionally from imu0's LSB. This is
    the SOLE source of truth for channel interpretation downstream in
    test_drift -- the physical-plausibility check there is a diagnostic
    cross-check only, never the decision mechanism when this tag exists.

    device_us: firmware micros()-at-sample-time, present in every frame.
    Wraps at 2**32 us (~71 min); consumers must diff with a 32-bit mask.

    seq: firmware's per-frame counter (0-255, wrapping); used here for
    stuck/duplicate-frame sanity checks (Pre-Priority #14).

    recalibrate=True sends RECALIBRATE_GYRO and blocks (via
    wait_for_recalibration's deterministic gyro_calib_toggle check) until
    confirmed, before the capture window -- works identically over USB
    and BLE (Pre-Priority #0.3).

    force_ble=True forces the BLE transport even if USB is also connected
    (Reader otherwise silently prefers USB whenever it's present) -- needed
    for a genuine USB-vs-BLE controlled comparison when the device is
    powered over USB during a BLE test. Without this, --ble is a no-op
    whenever a USB cable happens to be plugged in.
    """
    reader = Reader(
        sensor_config={hand: {'flex': False, 'press': False, 'imu': True}},
        use_ble=use_ble,
        force_ble=force_ble,
    )
    reader.start_readers()

    thread_info = reader.threads[hand]
    transport = 'BLE' if thread_info.get('is_ble', False) else 'USB'
    if thread_info.get('is_ble', False):
        if not thread_info['serial'].wait_connected(timeout=15.0):
            print("Connection timeout.")
            reader.stop_readers()
            return []
    q = reader.threads[hand]['parser'].getQ()
    print(f"Transport: {transport} (actual, confirmed on connect)")

    # VERIFY STREAM -- confirm data is flowing at all before doing anything
    # else, so a total silence failure is diagnosed here, not confused
    # with a later handshake timeout.
    verify_deadline = time.perf_counter() + VERIFY_STREAM_TIMEOUT
    stream_verified = False
    while time.perf_counter() < verify_deadline:
        try:
            data = q.get(timeout=max(0.05, verify_deadline - time.perf_counter()))
        except queue.Empty:
            break
        if 'imu' in data:
            stream_verified = True
            break
    if not stream_verified:
        print(f"  WARNING: no data received within {VERIFY_STREAM_TIMEOUT:.1f}s "
              f"of connecting over {transport} -- aborting capture.")
        reader.stop_readers()
        return []

    # Pre-Priority #0.5: report ACTUAL confirmed state as each fact becomes
    # known, not a state assumed from what was merely requested.
    mode_confirmed = None   # None = not requested; True/False = requested and (un)confirmed
    if stream_gyro:
        reader.send_command(hand, "USE_GYRO_ON")
        mode_confirmed = _wait_for_sensor_type(q, expected=1)
        print(f"Sensor mode: gyro ({'CONFIRMED via sensor_type=1' if mode_confirmed else 'REQUESTED but UNCONFIRMED -- handshake timed out'})")
    else:
        print("Sensor mode: accel (default, not switched)")

    cal_confirmed = None    # None = not requested; True/False = requested and (un)confirmed
    if recalibrate:
        cal_confirmed = wait_for_recalibration(reader, hand, q)
        print(f"Gyro calibration: {'FRESH (CONFIRMED this run)' if cal_confirmed else 'REQUESTED but UNCONFIRMED -- do not treat as fresh'}")
        if cal_confirmed:
            # runStartupGyroCalibration() resets the AHRS's internal
            # residual-bias tracker to zero and re-anchors yaw right at
            # confirmation -- give the filter a moment to settle past
            # that transient before the capture window starts, rather
            # than recording it as if it were steady-state behavior.
            print(f"  Settling {CAL_SETTLE_TIME_S:.0f}s before recording...")
            time.sleep(CAL_SETTLE_TIME_S)
            # Drain whatever accumulated in the queue during that sleep --
            # the reader thread keeps running independently, so without
            # this the capture loop below would start by consuming a
            # ~2s backlog from DURING the settle window, not fresh
            # samples from after it.
            while True:
                try:
                    q.get_nowait()
                except queue.Empty:
                    break
    else:
        print("Gyro calibration: EXISTING (not requested fresh this run -- "
              "whatever was active at connection time; use --recalibrate "
              "for a verified-fresh run)")

    samples = []
    t0 = None
    sensor_type_seen = set()
    device_us_seen = False
    print(f"Capturing {duration}s of IMU data on hand '{hand}' via {transport}"
          f"{' (gyro mode)' if stream_gyro else ''}"
          f"{' (freshly recalibrated)' if recalibrate else ''}...")

    while True:
        try:
            data = q.get(timeout=1.0)
        except queue.Empty:
            if t0 and (time.perf_counter() - t0) >= duration:
                break
            continue

        if 'imu' not in data:
            continue

        now = time.perf_counter()
        if t0 is None:
            t0 = now
        yaw, pitch, roll = decode_euler(data['imu'][0:3])

        if stream_gyro and len(data['imu']) >= 6:
            raw0, raw1, raw2 = data['imu'][3:6]
        else:
            raw0 = raw1 = raw2 = float('nan')

        sensor_type = data.get('sensor_type', float('nan'))
        if not (isinstance(sensor_type, float) and np.isnan(sensor_type)):
            sensor_type_seen.add(int(sensor_type))
        else:
            sensor_type = float('nan')

        device_us = data.get('device_us', float('nan'))
        if not np.isnan(device_us):
            device_us_seen = True

        seq = data.get('seq', -1)

        samples.append((now - t0, yaw, pitch, roll, raw0, raw1, raw2,
                         sensor_type, device_us, seq))

        if (now - t0) >= duration:
            break

    if stream_gyro:
        reader.send_command(hand, "USE_GYRO_OFF")

    reader.stop_readers()
    print(f"Captured {len(samples)} samples via {transport}.")

    if not device_us_seen:
        print("  NOTE: no device_us field on captured frames -- Reader/"
              "port_read.py isn't surfacing it as data['device_us']. "
              "dt characterization will fall back to host arrival "
              "timestamps, which conflate firmware scheduler jitter with "
              "USB/BLE host-reception jitter.")

    if stream_gyro and sensor_type_seen != {1}:
        print(f"  NOTE: sensor_type values {sorted(sensor_type_seen)} seen "
              f"during a gyro-mode capture (expected only {{1}}) -- some "
              f"frames were captured before the mode switch settled.")

    _sanity_check_samples(samples)

    if analyze_last_s is not None and analyze_last_s < duration:
        cutoff = duration - analyze_last_s
        samples = [(s[0] - cutoff,) + s[1:] for s in samples if s[0] >= cutoff]
        print(f"  Trimmed to last {analyze_last_s:.0f}s of the {duration:.0f}s "
              f"recording ({len(samples)} samples retained).")

    return samples

def replicate_gyro_stationary_gate(t, gx, gy, gz,
                                    gyro_filter_tau=FW_GYRO_FILTER_TAU,
                                    variance_window_tau=FW_VARIANCE_WINDOW_TAU,
                                    gyro_var_threshold=FW_GYRO_VAR_THRESHOLD):
    """
    Re-derives, from captured raw gyro telemetry, the same two-stage
    pipeline MahonyAHRS.h applies before its stationary-variance gate:

        raw gyro -> per-axis dt-scaled EMA pre-filter (tau=gyro_filter_tau)
                 -> magnitude
                 -> dt-scaled EWMA variance (tau=variance_window_tau)
                 -> gate: variance < gyro_var_threshold

    Lets you pick gyroVarThreshold from real bench data instead of
    trusting the firmware default, and see exactly what the filter's
    variance gate would have seen, sample by sample.

    Gyro-only: the firmware's actual `stationary` flag also ANDs an
    accelerometer-magnitude-variance condition this script can't
    reconstruct (see module docstring). Treat the returned gate as
    necessary-but-not-sufficient for the firmware's real flag.
    """
    t = np.asarray(t, dtype=np.float64)
    gx = np.asarray(gx, dtype=np.float64)
    gy = np.asarray(gy, dtype=np.float64)
    gz = np.asarray(gz, dtype=np.float64)
    n = len(t)

    filtered_mag = np.zeros(n)
    var_trace = np.zeros(n)
    gate_trace = np.zeros(n, dtype=bool)

    fx = fy = fz = 0.0
    mean_ema = 0.0
    var_ema = 0.0
    prev_t = t[0]

    for i in range(n):
        dt = (t[i] - prev_t) if i > 0 else 0.01
        dt = max(dt, 1e-4)
        prev_t = t[i]

        # Stage 1: gyro pre-filter (per-axis, dt-scaled EMA)
        a_g = dt / (gyro_filter_tau + dt)
        fx += a_g * (gx[i] - fx)
        fy += a_g * (gy[i] - fy)
        fz += a_g * (gz[i] - fz)

        m = np.sqrt(fx * fx + fy * fy + fz * fz)
        filtered_mag[i] = m

        # Stage 2: EWMA variance of the filtered magnitude
        a_v = dt / (variance_window_tau + dt)
        d = m - mean_ema
        mean_ema += a_v * d
        var_ema += a_v * (d * d - var_ema)
        var_trace[i] = var_ema

        gate_trace[i] = var_ema < gyro_var_threshold

    return {
        'gyro_magnitude_raw': np.sqrt(gx * gx + gy * gy + gz * gz),
        'gyro_magnitude_filtered': filtered_mag,
        'gyro_var_ema': var_trace,
        'would_be_stationary_gyro_only': gate_trace,
        'fraction_gyro_only_stationary': float(np.mean(gate_trace)),
        'gyro_var_threshold': gyro_var_threshold,
    }


def _detrended_std(t, series):
    """Std of the residual after removing a linear (least-squares) fit --
    isolates noise/jitter from the systematic drift ramp. A raw std over a
    monotonically drifting signal is dominated by the ramp itself, not by
    the actual noise floor, and reads as far 'noisier' than it really is."""
    coeffs = np.polyfit(t, series, deg=1)
    residual = series - np.polyval(coeffs, t)
    return float(np.std(residual))


def _unwrap_deg(series):
    """Unwrap a ±180 deg-bounded angle series so a long-enough drift test
    that actually crosses the wrap boundary doesn't show a spurious jump.
    Applies to all three axes -- yaw/roll always wrapped at ±180; pitch
    did NOT before the Priority #1 fix (it was bounded to ±90 by
    decode_euler, so could never reach the wrap boundary), but now shares
    yaw/roll's full ±180 range and needs the same treatment."""
    return np.degrees(np.unwrap(np.radians(series)))


def naive_gyro_integration(t, gx, gy, gz, yaw0, pitch0, roll0):
    """
    Bare gyro-only dead-reckoning: cumulative integration of raw angular
    rate, with NO accelerometer correction, NO bias tracking, and NO
    pre-filtering -- the 'before' baseline the Mahony fusion is measured
    against. This is the closest external reconstruction of "what the
    sensor alone would show" available from captured telemetry, since we
    can't intercept the firmware's internal pre-fusion state directly.

    Uses the small-angle approximation body-rate ~= Euler-angle-rate
    (gz~yaw_rate, gy~pitch_rate, gx~roll_rate). That only holds near a
    level, near-stationary orientation -- exactly the drift-test regime
    (glove held flat) -- so it's valid here. It would NOT be a fair
    comparison during a large-angle maneuver like the step-response test
    (body-rate != Euler-rate once pitch/roll are far from zero), which is
    why this is wired into test_drift only, not the step test.

    Starts from the same initial angle as the firmware's fused output so
    only the trajectories differ, not the starting point.
    """
    n = len(t)
    yaw_n = np.empty(n)
    pitch_n = np.empty(n)
    roll_n = np.empty(n)
    yaw_n[0], pitch_n[0], roll_n[0] = yaw0, pitch0, roll0

    for i in range(1, n):
        dt = max(t[i] - t[i - 1], 1e-4)
        yaw_n[i]   = yaw_n[i - 1]   + gz[i] * dt
        pitch_n[i] = pitch_n[i - 1] + gy[i] * dt
        roll_n[i]  = roll_n[i - 1]  + gx[i] * dt

    return yaw_n, pitch_n, roll_n


def test_drift(hand, duration, use_ble=True, gyro_var_threshold=FW_GYRO_VAR_THRESHOLD,
                recalibrate=False, force_ble=False, analyze_last_s=None):
    """
    Place the glove flat and motionless for `duration` seconds.
    Any change in yaw/pitch/roll over this window is drift, since a
    stationary object has no true orientation change.
    """
    input(f"\n[DRIFT TEST] Place the glove on a flat, stationary surface.\n"
          f"Press Enter to begin a {duration}s recording...")

    samples = capture_stream(hand, duration, use_ble, stream_gyro=True,
                              recalibrate=recalibrate, force_ble=force_ble,
                              analyze_last_s=analyze_last_s)
    if len(samples) < 2:
        print("Not enough samples captured.")
        return None

    t = np.array([s[0] for s in samples])
    yaw   = np.array([s[1] for s in samples])
    pitch = np.array([s[2] for s in samples])
    roll  = np.array([s[3] for s in samples])
    raw0  = np.array([s[4] for s in samples])
    raw1  = np.array([s[5] for s in samples])
    raw2  = np.array([s[6] for s in samples])
    sensor_type = np.array([s[7] for s in samples], dtype=np.float64)

    result = {
        'duration_s': t[-1] - t[0],
        'n_samples': len(samples),
        'yaw_drift_deg':   yaw[-1]   - yaw[0],
        'pitch_drift_deg': pitch[-1] - pitch[0],
        'roll_drift_deg':  roll[-1]  - roll[0],
        'yaw_std_deg':   np.std(yaw),
        'pitch_std_deg': np.std(pitch),
        'roll_std_deg':  np.std(roll),
        'yaw_std_detrended_deg':   _detrended_std(t, yaw),
        'pitch_std_detrended_deg': _detrended_std(t, pitch),
        'roll_std_detrended_deg':  _detrended_std(t, roll),
        'yaw_range_deg':   yaw.max()   - yaw.min(),
        'pitch_range_deg': pitch.max() - pitch.min(),
        'roll_range_deg':  roll.max()  - roll.min(),
        't': t, 'yaw': yaw, 'pitch': pitch, 'roll': roll,
    }

    # Drift rate — most useful figure for a report (deg/min)
    result['yaw_drift_rate_deg_per_min']   = result['yaw_drift_deg']   / (result['duration_s'] / 60.0)
    result['pitch_drift_rate_deg_per_min'] = result['pitch_drift_deg'] / (result['duration_s'] / 60.0)
    result['roll_drift_rate_deg_per_min']  = result['roll_drift_deg']  / (result['duration_s'] / 60.0)

    # Test BOTH candidate interpretations of the raw imu[3:6] channel
    # against physical plausibility, rather than assuming either one.
    # Gyro/variance/naive-integration diagnostics only run if the gyro
    # interpretation actually wins that test on this capture.
    if not np.any(np.isnan(raw0)):
        gx, gy, gz = decode_gyro([raw0, raw1, raw2])
        ax, ay, az = decode_accel([raw0, raw1, raw2], full_scale_g=ACCEL_FULL_SCALE_G)

        gyro_mag_mean = float(np.mean(np.sqrt(gx**2 + gy**2 + gz**2)))
        accel_mag_mean = float(np.mean(np.sqrt(ax**2 + ay**2 + az**2)))

        gyro_plausible = gyro_mag_mean <= GYRO_PLAUSIBILITY_LIMIT_DPS
        accel_plausible = abs(accel_mag_mean - 1.0) <= ACCEL_PLAUSIBILITY_TOLERANCE_G

        result['raw_channel_gyro_mag_mean_dps'] = gyro_mag_mean
        result['raw_channel_accel_mag_mean_g']  = accel_mag_mean
        result['raw_channel_gyro_plausible']    = gyro_plausible
        result['raw_channel_accel_plausible']   = accel_plausible

        # Pre-Priority #2: the firmware's sensor_type tag (sourced straight
        # from data['sensor_type'], which port_read.py's ParseSerial now
        # decodes unconditionally) is the SOLE source of truth -- decide
        # the channel from it first. The plausibility check above is a
        # cross-check that should agree with the tag, never the decision
        # mechanism. Only fall back to guessing from physics (the elif
        # chain below) if the tag itself is unavailable, i.e. it wasn't
        # captured at all.
        valid_tags = sensor_type[~np.isnan(sensor_type)]
        tag_says_gyro  = valid_tags.size > 0 and bool(np.all(valid_tags == 1))
        tag_says_accel = valid_tags.size > 0 and bool(np.all(valid_tags == 0))

        result['sensor_type_tag_available'] = valid_tags.size > 0
        result['sensor_type_tag_interpretation'] = (
            'gyro' if tag_says_gyro else 'accel' if tag_says_accel else
            'mixed_or_unavailable')

        if tag_says_gyro and not gyro_plausible:
            print(f"  WARNING: sensor_type tag says gyro, but mean |gyro| = "
                  f"{gyro_mag_mean:.1f} dps exceeds the plausibility limit "
                  f"({GYRO_PLAUSIBILITY_LIMIT_DPS:.0f} dps). Tag and physics "
                  f"disagree -- since the tag is written from the same "
                  f"use_gyro flag that selects the channel, this points at a "
                  f"real gyro fault (huge bias, wrong FSR, bad axis scaling), "
                  f"not a mislabeled frame. Investigate before trusting drift "
                  f"numbers from this capture.")
        if tag_says_accel and not accel_plausible:
            print(f"  WARNING: sensor_type tag says accel, but mean |accel| = "
                  f"{accel_mag_mean:.3f} g is outside the +-"
                  f"{ACCEL_PLAUSIBILITY_TOLERANCE_G:.1f} g band around 1.0 g. "
                  f"Check accelerometer calibration / mounting.")

        if tag_says_gyro:
            result['gyro_channel_available'] = True
            result['gyro_channel_interpretation'] = 'gyro'
            result['gx'], result['gy'], result['gz'] = gx, gy, gz

            gate_diag = replicate_gyro_stationary_gate(
                t, gx, gy, gz, gyro_var_threshold=gyro_var_threshold)
            result.update(gate_diag)

            # Consistency check: does the residual gz over this window
            # roughly account for the observed yaw drift? Both a sign
            # mismatch and a large magnitude gap are informative — either
            # points at something other than plain gz integration (axis
            # convention, quaternion/Euler conversion, etc.), not just "the
            # bias tracker didn't converge."
            mean_gz_dps = float(np.mean(gz))
            result['mean_gz_dps'] = mean_gz_dps
            result['predicted_yaw_drift_deg'] = mean_gz_dps * result['duration_s']
            result['yaw_drift_residual_deg'] = (
                result['yaw_drift_deg'] - result['predicted_yaw_drift_deg'])

            # Filter effectiveness: Mahony (fused) vs. naive gyro-only
            # integration over the same window, from the same raw gyro.
            # All three axes are unwrapped first so a long enough drift
            # test that crosses +/-180 doesn't show a spurious jump --
            # pitch needs this now too (Priority #1 fix removed its old
            # +-90 bound in decode_euler, so it can reach the wrap
            # boundary the same way yaw/roll always could).
            yaw_uw   = _unwrap_deg(yaw)
            pitch_uw = _unwrap_deg(pitch)
            roll_uw  = _unwrap_deg(roll)

            yaw_n, pitch_n, roll_n = naive_gyro_integration(
                t, gx, gy, gz, yaw_uw[0], pitch_uw[0], roll_uw[0])

            result['yaw_fused_unwrapped']   = yaw_uw
            result['pitch_fused_unwrapped'] = pitch_uw
            result['roll_fused_unwrapped']  = roll_uw
            result['yaw_naive']   = yaw_n
            result['pitch_naive'] = pitch_n
            result['roll_naive']  = roll_n

            result['yaw_naive_drift_deg']   = float(yaw_n[-1]   - yaw_n[0])
            result['pitch_naive_drift_deg'] = float(pitch_n[-1] - pitch_n[0])
            result['roll_naive_drift_deg']  = float(roll_n[-1]  - roll_n[0])

            result['yaw_fused_drift_unwrapped_deg']   = float(yaw_uw[-1]   - yaw_uw[0])
            result['pitch_fused_drift_unwrapped_deg'] = float(pitch_uw[-1] - pitch_uw[0])
            result['roll_fused_drift_unwrapped_deg']  = float(roll_uw[-1]  - roll_uw[0])

            result['yaw_naive_std_detrended_deg']   = _detrended_std(t, yaw_n)
            result['pitch_naive_std_detrended_deg'] = _detrended_std(t, pitch_n)
            result['roll_naive_std_detrended_deg']  = _detrended_std(t, roll_n)

            def _reduction(naive_val, filt_val):
                if abs(filt_val) < 1e-6:
                    return float('inf') if abs(naive_val) > 1e-6 else 1.0
                return abs(naive_val) / abs(filt_val)

            result['yaw_drift_reduction'] = _reduction(
                result['yaw_naive_drift_deg'], result['yaw_fused_drift_unwrapped_deg'])
            result['pitch_drift_reduction'] = _reduction(
                result['pitch_naive_drift_deg'], result['pitch_fused_drift_unwrapped_deg'])
            result['roll_drift_reduction'] = _reduction(
                result['roll_naive_drift_deg'], result['roll_fused_drift_unwrapped_deg'])

            result['yaw_noise_reduction'] = _reduction(
                result['yaw_naive_std_detrended_deg'], result['yaw_std_detrended_deg'])
            result['pitch_noise_reduction'] = _reduction(
                result['pitch_naive_std_detrended_deg'], result['pitch_std_detrended_deg'])
            result['roll_noise_reduction'] = _reduction(
                result['roll_naive_std_detrended_deg'], result['roll_std_detrended_deg'])

        elif tag_says_accel:
            result['gyro_channel_available'] = False
            result['gyro_channel_interpretation'] = 'accel'
            result['ax'], result['ay'], result['az'] = ax, ay, az
            result['gyro_channel_warning'] = (
                f"sensor_type tag says accel (mean |accel| = "
                f"{accel_mag_mean:.3f} g). Gyro/variance/naive-integration "
                f"diagnostics need a genuine gyro channel and are skipped -- "
                f"re-run with stream_gyro=True reaching the firmware "
                f"correctly, or accel-based diagnostics (jerk, "
                f"accelConfidence replica) could be built against this "
                f"channel instead."
            )

        else:
            # sensor_type comes straight from data['sensor_type']
            # (Pre-Priority #2), populated on every frame -- so this
            # branch is only reachable if a capture caught a genuine MIX
            # of tag values (some samples gyro, some accel), most likely
            # a mode-switch that wasn't fully settled before recording
            # started. Falls back to the physical-plausibility guess
            # rather than trusting an internally inconsistent tag.
            print("  Falling back to physical-plausibility channel guess "
                  "(sensor_type tag was inconsistent across this capture).")

            if gyro_plausible and not accel_plausible:
                result['gyro_channel_available'] = True
                result['gyro_channel_interpretation'] = 'gyro_guessed'
                result['gx'], result['gy'], result['gz'] = gx, gy, gz
                gate_diag = replicate_gyro_stationary_gate(
                    t, gx, gy, gz, gyro_var_threshold=gyro_var_threshold)
                result.update(gate_diag)

            elif accel_plausible and not gyro_plausible:
                result['gyro_channel_available'] = False
                result['gyro_channel_interpretation'] = 'accel_guessed'
                result['ax'], result['ay'], result['az'] = ax, ay, az
                result['gyro_channel_warning'] = (
                    f"imu[3:6] decodes as PLAUSIBLE ACCEL (mean |accel| = "
                    f"{accel_mag_mean:.3f} g) but NOT as plausible gyro "
                    f"(mean |gyro| would be {gyro_mag_mean:.1f} dps). "
                    f"Gyro diagnostics skipped."
                )

            elif gyro_plausible and accel_plausible:
                result['gyro_channel_available'] = False
                result['gyro_channel_interpretation'] = 'ambiguous'
                result['gyro_channel_warning'] = (
                    f"imu[3:6] decodes as plausible under BOTH "
                    f"interpretations (gyro: {gyro_mag_mean:.1f} dps, "
                    f"accel: {accel_mag_mean:.3f} g) -- ambiguous without "
                    f"a sensor_type tag, skipping diagnostics rather than "
                    f"guessing which one is real."
                )

            else:
                result['gyro_channel_available'] = False
                result['gyro_channel_interpretation'] = 'unknown'
                result['gyro_channel_warning'] = (
                    f"imu[3:6] doesn't decode plausibly as EITHER gyro "
                    f"(mean |gyro| = {gyro_mag_mean:.1f} dps) or accel "
                    f"(mean |accel| = {accel_mag_mean:.3f} g, expected "
                    f"~1.0 stationary), and the sensor_type tag was "
                    f"inconsistent across this capture. Skipping "
                    f"diagnostics; re-run the capture."
                )
    else:
        result['gyro_channel_available'] = False
        result['gyro_channel_interpretation'] = 'not_captured'

    return result


# ── Test 2: Step response — lag and overshoot ───────────────────────────

def test_step_response(hand, axis, target_angle, use_ble=True,
                        pre_hold_s=3.0, post_hold_s=5.0, settle_band_deg=2.0,
                        force_ble=False):
    """
    Operator holds the glove level/zeroed, then on cue rotates it briskly
    to a fixed pose against a physical reference (e.g. a set square at the
    target angle) and holds it still. Measures:
      - lag: time from motion onset to entering and staying within
             `settle_band_deg` of the final settled value
      - overshoot: max excursion past the settled value, as % of step size
    Ground truth for the step size is the operator's own physical reference
    jig — this script cannot verify the jig's angle independently.
    """
    idx = AXIS_INDEX[axis]

    print(f"\n[STEP RESPONSE TEST] axis={axis}, target~{target_angle} deg")
    input(f"Hold the glove level/at rest. Press Enter, then after "
          f"{pre_hold_s:.0f}s rotate briskly to the {target_angle} deg "
          f"reference and hold steady...")

    total_duration = pre_hold_s + post_hold_s
    samples = capture_stream(hand, total_duration, use_ble, force_ble=force_ble)
    if len(samples) < 10:
        print("Not enough samples captured.")
        return None

    t = np.array([s[0] for s in samples])
    series = np.array([s[1 + idx] for s in samples])

    # Baseline = mean of samples before the cued motion window
    baseline_mask = t < pre_hold_s
    baseline = np.mean(series[baseline_mask]) if baseline_mask.any() else series[0]

    # Settled value = mean of the last 1s of the recording (assumed stable)
    settle_mask = t > (t[-1] - 1.0)
    settled_value = np.mean(series[settle_mask])

    step_size = settled_value - baseline

    # Find motion onset: first sample where |series - baseline| exceeds
    # 10% of the step size, searched only after pre_hold_s
    post_mask = t >= pre_hold_s
    t_post = t[post_mask]
    s_post = series[post_mask]

    onset_thresh = baseline + 0.1 * step_size
    if step_size >= 0:
        onset_candidates = np.where(s_post >= onset_thresh)[0]
    else:
        onset_candidates = np.where(s_post <= onset_thresh)[0]

    if len(onset_candidates) == 0:
        print("Could not detect motion onset — check that a step actually occurred.")
        return None
    t_onset = t_post[onset_candidates[0]]

    # Settling time: first time after onset where the signal stays within
    # settle_band_deg of settled_value for the remainder of the recording
    lower, upper = settled_value - settle_band_deg, settled_value + settle_band_deg
    settle_time = None
    for i in range(len(t_post)):
        if t_post[i] < t_onset:
            continue
        if np.all((s_post[i:] >= lower) & (s_post[i:] <= upper)):
            settle_time = t_post[i]
            break

    lag_s = (settle_time - t_onset) if settle_time is not None else None

    # Overshoot: max excursion beyond settled_value, in the direction of the step
    post_onset = s_post[t_post >= t_onset]
    if step_size >= 0:
        peak = post_onset.max()
        overshoot_deg = max(0.0, peak - settled_value)
    else:
        peak = post_onset.min()
        overshoot_deg = max(0.0, settled_value - peak)
    overshoot_pct = (overshoot_deg / abs(step_size) * 100.0) if abs(step_size) > 1e-6 else 0.0

    result = {
        'axis': axis,
        'baseline_deg': baseline,
        'settled_value_deg': settled_value,
        'step_size_deg': step_size,
        'lag_s': lag_s,
        'overshoot_deg': overshoot_deg,
        'overshoot_pct': overshoot_pct,
        't': t, 'series': series,
        't_onset': t_onset, 'settle_time': settle_time,
        'settle_band_deg': settle_band_deg,
    }
    return result


# ── Test 3: Static accuracy ──────────────────────────────────────────────

def test_static_accuracy(hand, axis, reference_angle, use_ble=True, duration=5.0,
                          force_ble=False):
    """
    Hold the glove against a known physical reference (e.g. flat = 0 deg,
    or a set square = 90 deg) and record. Compares the filter's mean
    steady-state estimate to the reference angle the operator specifies.
    """
    idx = AXIS_INDEX[axis]
    input(f"\n[ACCURACY TEST] axis={axis}, reference={reference_angle} deg\n"
          f"Hold the glove against the {reference_angle} deg reference. "
          f"Press Enter to record for {duration:.0f}s...")

    samples = capture_stream(hand, duration, use_ble, force_ble=force_ble)
    if len(samples) < 2:
        print("Not enough samples captured.")
        return None

    series = np.array([s[1 + idx] for s in samples])
    mean_est = np.mean(series)
    std_est  = np.std(series)
    error    = mean_est - reference_angle

    return {
        'axis': axis,
        'reference_angle_deg': reference_angle,
        'mean_estimate_deg': mean_est,
        'std_deg': std_est,
        'error_deg': error,
        'n_samples': len(samples),
    }


# ── Test 4: dt / timing jitter (Priority #6) ─────────────────────────────
# Decision this test exists to support: characterize timing BEFORE
# implementing a FIFO/data-ready sampling scheme. If dt is already tight,
# FIFO is unneeded complexity; if it's jittery, FIFO is worth building.
# Uses device_us, which is already in every frame -- no firmware/protocol
# change needed here, unlike Priority #1.

DT_TARGET_S = 0.010   # matches firmware's 100 Hz target (SENSOR_INTERVAL_MS)
DT_JITTER_P99_WARN_S = 0.015   # if p99 dt exceeds this, FIFO is worth it


def test_dt_jitter(hand, duration=20.0, use_ble=True, recalibrate=False,
                    force_ble=False, analyze_last_s=None):
    """
    No physical action required -- just let the glove sit anywhere (moving
    or still, timing doesn't care) for `duration` seconds while dt is
    characterized from consecutive device_us timestamps.

    Reports device-side dt (firmware scheduler jitter, from device_us) and,
    where device_us is unavailable, falls back to host-arrival dt (which
    conflates firmware jitter with USB/BLE reception jitter -- reported
    with that caveat, not silently treated as equivalent).
    """
    print(f"\n[TIMING TEST] Capturing {duration:.0f}s to characterize dt "
          f"(no specific pose needed)...")
    samples = capture_stream(hand, duration, use_ble, stream_gyro=True,
                              recalibrate=recalibrate, force_ble=force_ble,
                              analyze_last_s=analyze_last_s)
    if len(samples) < 10:
        print("Not enough samples captured.")
        return None

    device_us = np.array([s[8] for s in samples], dtype=np.float64)
    host_t    = np.array([s[0] for s in samples], dtype=np.float64)

    result = {'n_samples': len(samples), 'duration_s': host_t[-1] - host_t[0]}

    if not np.any(np.isnan(device_us)):
        # uint32 wraparound-safe diff (device_us wraps at 2**32 us, ~71 min)
        raw = device_us.astype(np.uint64)
        d = (np.diff(raw).astype(np.int64) & 0xFFFFFFFF).astype(np.float64)
        dt = d * 1e-6
        source = 'device_us'
    else:
        dt = np.diff(host_t)
        source = 'host_arrival (device_us unavailable -- includes USB/BLE jitter)'

    result['dt_source'] = source
    result['dt_mean_s']   = float(np.mean(dt))
    result['dt_std_s']    = float(np.std(dt))
    result['dt_min_s']    = float(np.min(dt))
    result['dt_max_s']    = float(np.max(dt))
    result['dt_p95_s']    = float(np.percentile(dt, 95))
    result['dt_p99_s']    = float(np.percentile(dt, 99))
    result['n_dt_gt_15ms'] = int(np.sum(dt > 0.015))
    result['n_dt_gt_20ms'] = int(np.sum(dt > 0.020))
    result['dt'] = dt

    result['fifo_recommended'] = result['dt_p99_s'] > DT_JITTER_P99_WARN_S
    return result


def print_dt_summary(r):
    if r is None:
        return
    print(f"\n--- Timing / dt Summary ({r['dt_source']}) ---")
    print(f"  Target dt:       {DT_TARGET_S*1000:.1f} ms (firmware SENSOR_INTERVAL_MS)")
    print(f"  Mean dt:         {r['dt_mean_s']*1000:.2f} ms")
    print(f"  Std dt:          {r['dt_std_s']*1000:.2f} ms")
    print(f"  Min / Max dt:    {r['dt_min_s']*1000:.2f} / {r['dt_max_s']*1000:.2f} ms")
    print(f"  95th pct dt:     {r['dt_p95_s']*1000:.2f} ms")
    print(f"  99th pct dt:     {r['dt_p99_s']*1000:.2f} ms")
    print(f"  dt > 15ms:       {r['n_dt_gt_15ms']} / {r['n_samples']-1}")
    print(f"  dt > 20ms:       {r['n_dt_gt_20ms']} / {r['n_samples']-1}")
    if r['dt_source'] != 'device_us':
        print("  CAVEAT: this is host-arrival timing, not device_us -- it "
              "conflates firmware scheduler jitter with USB/BLE reception "
              "jitter. Update Reader/port_read.py to expose data['device_us'] "
              "for a clean separation.")
    if r['fifo_recommended']:
        print(f"  -> p99 dt ({r['dt_p99_s']*1000:.1f} ms) exceeds "
              f"{DT_JITTER_P99_WARN_S*1000:.0f} ms: timing has real jitter, "
              f"MPU6886 FIFO/data-ready sampling is likely worth implementing.")
    else:
        print(f"  -> p99 dt is within {DT_JITTER_P99_WARN_S*1000:.0f} ms: "
              f"timing looks stable, don't add FIFO complexity yet.")


# ── Pre-Priority #11: raw-frame diagnostic ──────────────────────────────

def dump_raw_frames(hand, n=8, use_ble=True, stream_gyro=False, timeout=10.0,
                     force_ble=False):
    """Print n decoded frames with every field, no full capture required.
    The fastest way to confirm the parser/protocol are correct on a given
    transport before committing to a 30s test -- run once over USB, once
    over BLE, and compare (Pre-Priority #9/#11). force_ble as in
    capture_stream()."""
    reader = Reader(
        sensor_config={hand: {'flex': False, 'press': False, 'imu': True}},
        use_ble=use_ble,
        force_ble=force_ble,
    )
    reader.start_readers()
    thread_info = reader.threads[hand]
    transport = 'BLE' if thread_info.get('is_ble', False) else 'USB'
    if thread_info.get('is_ble', False):
        if not thread_info['serial'].wait_connected(timeout=15.0):
            print("Connection timeout.")
            reader.stop_readers()
            return
    q = reader.threads[hand]['parser'].getQ()
    print(f"\nTransport: {transport}")

    if stream_gyro:
        reader.send_command(hand, "USE_GYRO_ON")
        _wait_for_sensor_type(q, expected=1)

    count = 0
    deadline = time.perf_counter() + timeout
    while count < n and time.perf_counter() < deadline:
        try:
            data = q.get(timeout=1.0)
        except queue.Empty:
            continue
        if 'imu' not in data:
            continue
        count += 1
        yaw, pitch, roll = decode_euler(data['imu'][0:3])
        if len(data['imu']) >= 6:
            raw0, raw1, raw2 = data['imu'][3:6]
        else:
            raw0 = raw1 = raw2 = float('nan')
        print(f"frame {count}: seq={data.get('seq')} "
              f"device_us={data.get('device_us')} "
              f"sensor_type={data.get('sensor_type')} "
              f"gyro_calib_toggle={data.get('gyro_calib_toggle')} "
              f"yaw={yaw:.2f} pitch={pitch:.2f} roll={roll:.2f} "
              f"imu0={raw0} imu1={raw1} imu2={raw2}")

    if stream_gyro:
        reader.send_command(hand, "USE_GYRO_OFF")
    reader.stop_readers()
    if count == 0:
        print(f"  No frames received within {timeout:.1f}s.")


# ── Priority #1: quaternion sweep (Case A vs Case B) ────────────────────

def test_quaternion_sweep(hand, duration=35.0, use_ble=True, force_ble=False,
                           recalibrate=False):
    """
    Capture qw,qx,qy,qz alongside yaw/pitch/roll while the operator slowly
    rotates the glove's pitch axis through a wide range (suggested: a
    slow, steady 0 -> 180 deg sweep over the recording window).

    computeEuler() was re-derived this session (MahonyAHRS.h) so pitch is
    no longer capped to +-90 -- it was asin-bounded there purely because
    it was the MIDDLE rotation in the original ZYX sequence, not because
    of anything wrong with the quaternion (confirmed by code review: the
    propagation+correction path is quaternion/vector-based with zero
    Euler-angle dependency -- Case A, not Case B). Yaw is now the bounded
    middle axis instead. This test remains useful to CONFIRM that
    directly: pitch in the captured data should now track smoothly past
    90 deg with no fold-back, while yaw would be the one to show the
    old ceiling-and-fold behavior if swept through its own 90 deg point.

      Case A (expected, Euler-representation only): quaternion norm
        stays ~1.0 throughout, components change smoothly, any sign
        change is a clean FOUR-WAY flip (q <-> -q, harmless -- unit
        quaternions double-cover SO(3)).
      Case B (would indicate an actual AHRS/sensor problem, not expected
        given the code review): the quaternion itself glitches -- non-
        unit norm, a single-component discontinuity, or any change that
        ISN'T a clean four-way sign flip.

    Sends QUAT_STREAM_ON before recording, QUAT_STREAM_OFF after -- this
    is the raw quaternion from getQuaternion(), which bypasses
    computeEuler()/smoothOutputs() entirely on the firmware side.

    recalibrate=True mirrors capture_stream's sequence: confirm via
    gyro_calib_toggle, settle CAL_SETTLE_TIME_S, drain the backlog that
    accumulated during that sleep, THEN start recording -- same reasoning
    as there (the AHRS's internal residual-bias tracker and yaw are reset
    at calibration completion; recording through that transient would
    misrepresent steady-state behavior as part of the sweep data).
    """
    input(f"\n[QUATERNION SWEEP] Slowly rotate the glove's pitch axis through "
          f"a wide range -- e.g. a steady 0 -> 180 deg sweep over about "
          f"{duration:.0f}s. Press Enter to begin recording...")

    reader = Reader(
        sensor_config={hand: {'flex': False, 'press': False, 'imu': True}},
        use_ble=use_ble,
        force_ble=force_ble,
    )
    reader.start_readers()
    thread_info = reader.threads[hand]
    transport = 'BLE' if thread_info.get('is_ble', False) else 'USB'
    if thread_info.get('is_ble', False):
        if not thread_info['serial'].wait_connected(timeout=15.0):
            print("Connection timeout.")
            reader.stop_readers()
            return None
    q = reader.threads[hand]['parser'].getQ()
    print(f"Transport: {transport} (actual, confirmed on connect)")

    reader.send_command(hand, "QUAT_STREAM_ON")
    quat_confirmed = False
    deadline = time.perf_counter() + GYRO_MODE_SETTLE_TIMEOUT
    while time.perf_counter() < deadline:
        try:
            data = q.get(timeout=max(0.05, deadline - time.perf_counter()))
        except queue.Empty:
            break
        if 'quat' in data:
            quat_confirmed = True
            break
    print(f"Quaternion stream: {'CONFIRMED' if quat_confirmed else 'UNCONFIRMED -- proceeding anyway; check port_read.py is current'}")

    if recalibrate:
        cal_confirmed = wait_for_recalibration(reader, hand, q)
        print(f"Gyro calibration: {'FRESH (CONFIRMED this run)' if cal_confirmed else 'REQUESTED but UNCONFIRMED -- do not treat as fresh'}")
        if cal_confirmed:
            print(f"  Settling {CAL_SETTLE_TIME_S:.0f}s before recording...")
            time.sleep(CAL_SETTLE_TIME_S)
            while True:
                try:
                    q.get_nowait()
                except queue.Empty:
                    break
    else:
        print("Gyro calibration: EXISTING (not requested fresh this run)")

    samples = []
    t0 = None
    print(f"Recording for {duration:.0f}s...")
    while True:
        try:
            data = q.get(timeout=1.0)
        except queue.Empty:
            if t0 and (time.perf_counter() - t0) >= duration:
                break
            continue
        if 'imu' not in data or 'quat' not in data:
            continue
        now = time.perf_counter()
        if t0 is None:
            t0 = now
        yaw, pitch, roll = decode_euler(data['imu'][0:3])
        qw, qx, qy, qz = data['quat']
        samples.append((now - t0, yaw, pitch, roll, qw, qx, qy, qz,
                         data.get('seq', -1), data.get('device_us', float('nan'))))
        if (now - t0) >= duration:
            break

    reader.send_command(hand, "QUAT_STREAM_OFF")
    reader.stop_readers()
    print(f"Captured {len(samples)} samples via {transport}.")

    if len(samples) < 2:
        print("  Not enough samples captured.")
        return samples

    qw_a = np.array([s[4] for s in samples])
    qx_a = np.array([s[5] for s in samples])
    qy_a = np.array([s[6] for s in samples])
    qz_a = np.array([s[7] for s in samples])
    norms = np.sqrt(qw_a**2 + qx_a**2 + qy_a**2 + qz_a**2)
    pitch_a = np.array([s[2] for s in samples])

    print(f"  Quaternion norm range: [{norms.min():.4f}, {norms.max():.4f}] "
          "(should be ~1.0 throughout)")
    if abs(norms.min() - 1.0) > 0.05 or abs(norms.max() - 1.0) > 0.05:
        print("  WARNING: norm deviates from 1.0 by more than 0.05 -- int16 "
              "quantization alone shouldn't cause this (~1/32767 per "
              "component); worth a closer look as possible Case B evidence.")

    print(f"  Pitch range observed: [{pitch_a.min():.2f}, {pitch_a.max():.2f}] deg")

    # Cheap automated check for a per-sample quaternion discontinuity
    # beyond the expected smooth adjacent-sample change -- a large jump
    # in any single component NOT accompanied by all four flipping sign
    # together is the Case B signature described above.
    dq = np.sqrt(np.diff(qw_a)**2 + np.diff(qx_a)**2 + np.diff(qy_a)**2 + np.diff(qz_a)**2)
    suspicious = np.where(dq > 0.3)[0]   # generous threshold well above normal sample-to-sample motion
    if len(suspicious) > 0:
        print(f"  NOTE: {len(suspicious)} sample-to-sample quaternion jump(s) "
              f"> 0.3 detected (indices {suspicious.tolist()[:10]}"
              f"{'...' if len(suspicious) > 10 else ''}). Check whether each "
              f"is a clean four-way sign flip (harmless) or a genuine "
              f"discontinuity (Case B).")

    return samples


# ── Reporting ─────────────────────────────────────────────────────────────

def print_test_context(hand, transport, recalibrated):
    """Announces what was REQUESTED before any test runs -- transport
    forced (or not) and whether --recalibrate was passed. This is intent,
    not a confirmed outcome; the actual confirmed state (mode, calibration
    freshness) prints inline inside capture_stream() as each fact is
    established (Pre-Priority #0.5: "use the actual state, not a guessed
    state"), since that's the only point where it's actually known."""
    print(f"\nTransport requested: {transport}")
    print(f"Gyro calibration requested: {'fresh (--recalibrate)' if recalibrated else 'whatever is currently active (no --recalibrate)'}")
    print("(confirmed state for each test prints below, once actually established)")


def print_drift_summary(r):
    if r is None:
        return
    print("\n--- Static Drift Summary ---")
    print(f"  Duration:        {r['duration_s']:.1f}s ({r['n_samples']} samples)")
    for axis in ('yaw', 'pitch', 'roll'):
        print(f"  {axis.capitalize():6s} drift: {r[f'{axis}_drift_deg']:+7.3f} deg "
              f"({r[f'{axis}_drift_rate_deg_per_min']:+6.2f} deg/min), "
              f"std={r[f'{axis}_std_deg']:.3f} deg "
              f"(detrended={r[f'{axis}_std_detrended_deg']:.3f} deg), "
              f"range={r[f'{axis}_range_deg']:.3f} deg")
    print("  (raw std includes the drift ramp itself; detrended std is the "
          "noise floor with the linear trend removed -- the more meaningful "
          "figure for judging jitter independent of drift)")

    if r.get('gyro_channel_interpretation') not in (None, 'not_captured'):
        gm = r.get('raw_channel_gyro_mag_mean_dps')
        am = r.get('raw_channel_accel_mag_mean_g')
        gp = r.get('raw_channel_gyro_plausible')
        ap = r.get('raw_channel_accel_plausible')
        print("\n  --- imu[3:6] channel interpretation check ---")
        print(f"    as gyro (+-2000dps):        mean |v| = {gm:8.1f} dps  "
              f"[{'plausible' if gp else 'IMPLAUSIBLE'}]")
        print(f"    as accel (+-{ACCEL_FULL_SCALE_G:.0f}g):           mean |v| = {am:8.3f} g    "
              f"[{'plausible' if ap else 'IMPLAUSIBLE'}, expect ~1.0g stationary]")

    if not r.get('gyro_channel_available', False):
        if r.get('gyro_channel_warning'):
            marker = 'ℹ' if r.get('gyro_channel_interpretation') == 'accel' else '⚠'
            print(f"\n  {marker} {r['gyro_channel_warning']}")
        else:
            print("\n  (raw gyro channel not present in this packet — "
                  "skipping variance-gate / bias diagnostics)")
        return

    print("\n  --- Gyro variance-gate replica (mirrors MahonyAHRS.h) ---")
    print(f"  gyroVarThreshold used:      {r['gyro_var_threshold']:.2f} (deg/s)^2")
    print(f"  Fraction flagged stationary: {r['fraction_gyro_only_stationary']*100:.1f}% "
          f"of this (assumed motionless) recording")
    print(f"  Gyro |filtered| mean/max:   "
          f"{r['gyro_magnitude_filtered'].mean():.3f} / {r['gyro_magnitude_filtered'].max():.3f} dps")
    print(f"  Gyro variance (EWMA) max:   {r['gyro_var_ema'].max():.4f} (deg/s)^2")
    if r['fraction_gyro_only_stationary'] < 0.90:
        print("  ⚠ glove was held still but the gyro-only gate rarely fires — "
              "gyroVarThreshold is probably too tight for this unit; raise it "
              "(--gyro-var-threshold) and re-run before trusting the firmware default.")
    print("  NOTE: gyro-only — the firmware's real `stationary` flag also ANDs "
          "an accel-variance condition not reconstructable from this telemetry.")

    print("\n  --- Yaw drift vs. residual gz consistency check ---")
    print(f"  Mean gz over window:        {r['mean_gz_dps']:+.4f} dps")
    print(f"  Predicted yaw drift (∫gz):  {r['predicted_yaw_drift_deg']:+.3f} deg")
    print(f"  Observed yaw drift:         {r['yaw_drift_deg']:+.3f} deg")
    print(f"  Residual (unexplained):     {r['yaw_drift_residual_deg']:+.3f} deg")
    print("  If predicted and observed roughly agree (same sign, similar "
          "magnitude), the drift is attributable to residual gz rate — "
          "expected for a 6-DOF filter, and the target for further bias-"
          "tracking tuning. A sign flip or large residual instead points "
          "at something else (axis convention, integration, Euler "
          "conversion) worth investigating separately.")

    print("\n  --- Filter effectiveness: Mahony (fused) vs. naive gyro-only integration ---")
    print("  naive = cumulative integration of raw gyro alone: no accel "
          "correction, no bias tracking, no pre-filtering. Small-angle "
          "approx (body-rate ~= Euler-rate) -- valid here since the glove "
          "is held flat/stationary; NOT valid during large-angle motion.")
    for axis in ('yaw', 'pitch', 'roll'):
        nd = r[f'{axis}_naive_drift_deg']
        fd = (r['yaw_fused_drift_unwrapped_deg'] if axis == 'yaw' else
              r['roll_fused_drift_unwrapped_deg'] if axis == 'roll' else
              r[f'{axis}_drift_deg'])
        dr = r[f'{axis}_drift_reduction']
        dr_str = f"{dr:6.1f}x" if np.isfinite(dr) else "  >>1x"
        print(f"  {axis.capitalize():6s} drift:  naive={nd:+9.2f} deg   "
              f"fused={fd:+8.3f} deg   -> {dr_str} less drift")
    for axis in ('yaw', 'pitch', 'roll'):
        ns = r[f'{axis}_naive_std_detrended_deg']
        fs = r[f'{axis}_std_detrended_deg']
        nr = r[f'{axis}_noise_reduction']
        nr_str = f"{nr:6.1f}x" if np.isfinite(nr) else "  >>1x"
        print(f"  {axis.capitalize():6s} noise:  naive={ns:9.3f} deg   "
              f"fused={fs:8.3f} deg   -> {nr_str} less noise")
    print("  Drift reduction should be substantial on pitch/roll (accelerometer "
          "gives them an absolute reference) and typically smaller on yaw "
          "(only bias-tracking/integral help there, no absolute reference).")
    print("  Noise reduction does NOT necessarily follow the same pattern: "
          "Mahony fusion trades drift for injected accelerometer noise via "
          "kp, so a naive gyro-only integration -- a smooth, small-variance "
          "random walk -- can legitimately show LOWER sample-to-sample "
          "jitter than the fused output, which continuously re-incorporates "
          "a noisier absolute reference to keep drift bounded. A noise-"
          "reduction ratio below 1x here reflects that real trade-off, not "
          "a bug -- if it surprises you, it's worth cross-checking against "
          "kpStatic/kpDynamic rather than assuming something is broken.")


def print_step_summary(r):
    if r is None:
        return
    print(f"\n--- Step Response Summary ({r['axis']}) ---")
    print(f"  Baseline:        {r['baseline_deg']:.2f} deg")
    print(f"  Settled value:   {r['settled_value_deg']:.2f} deg")
    print(f"  Step size:       {r['step_size_deg']:+.2f} deg")
    lag_str = f"{r['lag_s']:.3f}s" if r['lag_s'] is not None else "N/A (did not settle)"
    print(f"  Settling lag:    {lag_str} (band = +/-{r['settle_band_deg']} deg)")
    print(f"  Overshoot:       {r['overshoot_deg']:.2f} deg ({r['overshoot_pct']:.1f}% of step)")

    if r['lag_s'] is not None and r['lag_s'] < 0.3 and abs(r['step_size_deg']) > 20:
        print("  ⚠ suspiciously low lag for a step this size. Onset detection "
              "only searches t >= pre_hold_s, anchored to the announced cue — "
              "if the operator started rotating even slightly before that cue "
              "(easy to do reacting to a spoken countdown), the search only "
              "sees the tail of an already-settling transient, not the true "
              "onset. This is a data-collection timing issue, not a filter/"
              "software one — re-run with a clearer audible or physical "
              "trigger if you need a trustworthy lag figure for this axis.")


def print_accuracy_summary(r):
    if r is None:
        return
    print(f"\n--- Static Accuracy Summary ({r['axis']}) ---")
    print(f"  Reference angle: {r['reference_angle_deg']:.2f} deg")
    print(f"  Mean estimate:   {r['mean_estimate_deg']:.2f} deg")
    print(f"  Error:           {r['error_deg']:+.2f} deg")
    print(f"  Std (noise):     {r['std_deg']:.3f} deg  ({r['n_samples']} samples)")


def plot_results(drift=None, step=None, accuracy=None, timing=None,
                  output_path='mahony_validation.png'):
    # accuracy is intentionally not plotted: it's a single mean+std value
    # (see print_accuracy_summary), not a time series -- a bar/point alone
    # isn't worth a panel. Accepted here only so callers can pass it
    # without a TypeError.
    has_gyro_panel = drift is not None and drift.get('gyro_channel_available', False)
    n_panels = (sum(x is not None for x in (drift, step, timing)) +
                (2 if has_gyro_panel else 0))
    if n_panels == 0:
        return
    fig, axes = plt.subplots(1, n_panels, figsize=(7 * n_panels, 5))
    if n_panels == 1:
        axes = [axes]
    ax_i = 0

    if drift is not None:
        ax = axes[ax_i]; ax_i += 1
        ax.plot(drift['t'], drift['yaw'],   label='Yaw')
        ax.plot(drift['t'], drift['pitch'], label='Pitch')
        ax.plot(drift['t'], drift['roll'],  label='Roll')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Angle (deg)')
        ax.set_title('Static Drift'); ax.legend(); ax.grid(True, alpha=0.3)

    if has_gyro_panel:
        ax = axes[ax_i]; ax_i += 1
        ax2 = ax.twinx()
        ax.plot(drift['t'], drift['gyro_magnitude_filtered'],
                color='#3498db', label='|gyro| filtered (dps)')
        ax2.plot(drift['t'], drift['gyro_var_ema'],
                 color='#e74c3c', alpha=0.7, label='EWMA variance')
        ax2.axhline(drift['gyro_var_threshold'], color='#e74c3c', linestyle='--',
                    alpha=0.5, label='gyroVarThreshold')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('|gyro| filtered (dps)', color='#3498db')
        ax2.set_ylabel('variance ((deg/s)^2)', color='#e74c3c')
        ax.set_title('Gyro Stationary-Gate Replica (gyro-only)')
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes[ax_i]; ax_i += 1
        axis_colors = (('yaw', drift['yaw_fused_unwrapped'], '#1f77b4'),
                       ('pitch', drift['pitch_fused_unwrapped'], '#ff7f0e'),
                       ('roll', drift['roll_fused_unwrapped'], '#2ca02c'))
        for name, fused_series, color in axis_colors:
            ax.plot(drift['t'], fused_series, color=color, linestyle='-',
                    label=f'{name} (fused)')
            ax.plot(drift['t'], drift[f'{name}_naive'], color=color, linestyle='--',
                    alpha=0.6, label=f'{name} (naive gyro-only)')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Angle (deg)')
        ax.set_title('Mahony (fused) vs. Naive Gyro-Only Integration')
        ax.legend(fontsize=7, ncol=2); ax.grid(True, alpha=0.3)

    if step is not None:
        ax = axes[ax_i]; ax_i += 1
        ax.plot(step['t'], step['series'], color='#3498db', label=step['axis'])
        ax.axhline(step['settled_value_deg'], color='gray', linestyle='--', label='Settled value')
        ax.axvline(step['t_onset'], color='green', linestyle=':', label='Motion onset')
        if step['settle_time'] is not None:
            ax.axvline(step['settle_time'], color='red', linestyle=':', label='Settled (lag end)')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Angle (deg)')
        ax.set_title(f"Step Response ({step['axis']})"); ax.legend(); ax.grid(True, alpha=0.3)

    if timing is not None:
        ax = axes[ax_i]; ax_i += 1
        dt_ms = timing['dt'] * 1000.0
        ax.hist(dt_ms, bins=50, color='#3498db', alpha=0.8)
        ax.axvline(DT_TARGET_S * 1000.0, color='gray', linestyle='--', label='Target dt')
        ax.axvline(timing['dt_p95_s'] * 1000.0, color='orange', linestyle=':', label='p95')
        ax.axvline(timing['dt_p99_s'] * 1000.0, color='red', linestyle=':', label='p99')
        ax.set_xlabel('dt (ms)'); ax.set_ylabel('count')
        ax.set_title(f"Sample Interval ({timing['dt_source']})")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"\nPlot saved to {output_path}")
    plt.show()


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Mahony AHRS validation suite')
    parser.add_argument('--hand', choices=['r', 'l'], default='r')
    parser.add_argument('--test', choices=['drift', 'step', 'accuracy', 'timing', 'quatsweep', 'all'],
                         default='all')
    parser.add_argument('--duration', type=float, default=35.0,
                         help='Drift/timing test duration (s)')
    parser.add_argument('--axis', choices=['yaw', 'pitch', 'roll'], default='yaw')
    parser.add_argument('--target-angle', type=float, default=90.0,
                         help='Reference angle for step test (deg)')
    parser.add_argument('--reference-angle', type=float, default=0.0,
                         help='Known reference angle for accuracy test (deg)')
    parser.add_argument('--ble', action='store_true',
                         help='Force BLE transport even if USB is also connected '
                              '(was a no-op before this fix: Reader always preferred '
                              'USB when present, silently defeating a USB-vs-BLE '
                              'comparison run over USB power)')
    parser.add_argument('--recalibrate', action='store_true',
                         help='Send RECALIBRATE_GYRO and confirm completion before '
                              'the selected test(s) -- works identically over USB '
                              'and BLE. Without this, calibration freshness depends '
                              'entirely on whether/when the device last rebooted.')
    parser.add_argument('--dump-frames', type=int, default=None, metavar='N',
                         help='Diagnostic mode: print N decoded frames and exit, '
                              'no full test run. Fastest way to confirm the parser '
                              'is correct on this transport.')
    parser.add_argument('--stream-gyro', action='store_true',
                         help='With --dump-frames: send USE_GYRO_ON first so the '
                              'dumped frames show gyro (sensor_type=1) instead of '
                              'the accel default (sensor_type=0).')
    parser.add_argument('--gyro-var-threshold', type=float, default=FW_GYRO_VAR_THRESHOLD,
                         help='Gyro EWMA-variance stationary gate, (deg/s)^2 '
                              '-- mirrors MahonyAHRS.h gyroVarThreshold (default: '
                              f'{FW_GYRO_VAR_THRESHOLD})')
    args = parser.parse_args()

    use_ble = True          # Reader still falls back to BLE if no USB port is found
    force_ble = args.ble    # ...but this is what actually forces BLE over a present USB port
    requested_transport = 'BLE (forced)' if force_ble else 'auto (USB preferred if present)'

    if args.dump_frames is not None:
        print(f"\nTransport requested: {requested_transport}")
        dump_raw_frames(args.hand, n=args.dump_frames, use_ble=use_ble,
                         force_ble=force_ble, stream_gyro=args.stream_gyro)
        return

    print_test_context(args.hand, requested_transport, args.recalibrate)

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    transport_tag = 'BLE' if force_ble else 'USB'  # best a-priori label; actual transport is logged per-connect

    drift_result = step_result = accuracy_result = timing_result = None

    if args.test in ('drift', 'all'):
        drift_result = test_drift(
            args.hand, args.duration, use_ble,
            gyro_var_threshold=args.gyro_var_threshold,
            recalibrate=args.recalibrate, force_ble=force_ble,
            analyze_last_s=ANALYZE_LAST_S)
        print_drift_summary(drift_result)

    if args.test in ('step', 'all'):
        step_result = test_step_response(args.hand, args.axis, args.target_angle,
                                          use_ble, force_ble=force_ble)
        print_step_summary(step_result)

    if args.test in ('accuracy', 'all'):
        accuracy_result = test_static_accuracy(args.hand, args.axis, args.reference_angle,
                                                use_ble, force_ble=force_ble)
        print_accuracy_summary(accuracy_result)

    if args.test == 'timing':   # not folded into 'all': separate concern, run explicitly
        timing_result = test_dt_jitter(
            args.hand, args.duration, use_ble,
            recalibrate=args.recalibrate, force_ble=force_ble)
        print_dt_summary(timing_result)

    if args.test == 'quatsweep':   # not folded into 'all': guided manual motion, own report, no plot
        test_quaternion_sweep(
            args.hand, args.duration, use_ble, force_ble=force_ble,
            recalibrate=args.recalibrate)
        return   # nothing here fits plot_results' drift/step/timing panels

    print("\n" + "=" * 60)
    print("NOTE: lag/overshoot/accuracy are only as precise as the")
    print("physical reference (set square, level surface, jig) used")
    print("during recording. No independent motion-capture ground")
    print("truth was used to validate these figures.")
    print("=" * 60)

    plot_results(drift_result, step_result, timing=timing_result,
                 output_path=f'mahony_validation_{ts}_{transport_tag}.png')


if __name__ == '__main__':
    main()