"""
kinematics.py — shared math primitives for Tier 1 mappings: quaternion
geometry AND small stateful signal-processing blocks, in one file since
neither has any music/MIDI/hardware knowledge of its own (that lives in
mappings/*.py and app_tier1.py respectively) -- keeping them apart across
two files was a distinction without a difference.

All mappings below consume `nav_quat` (corrected attitude, [w,x,y,z], from
NavEKF) and NavEKF velocity, NOT raw Mahony/Euler channels. Every quaternion
function here is written to be well-behaved at wraparound / near-singular
poses, since that's exactly the class of bug the estimator work already
burned time on (Jacobian bugs, gravity-update ill-conditioning) — no point
reintroducing an equivalent bug at the mapping layer via naive Euler math.

Convention: quaternions are unit, [w, x, y, z], Hamilton, body-to-world.

The DSP classes (OnePoleSmoother, LeakyEnergyTank, SlewLimiter,
HysteresisGate) are stateful filters, each stepped once per incoming pose
sample (BLE-rate, not audio-rate), physically motivated rather than
ad-hoc smoothing.
"""

import numpy as np


def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def quat_conj(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_rotate(q, v):
    """Rotate vector v (body or world frame per q's convention) by q."""
    qv = np.array([0.0, *v])
    return quat_mul(quat_mul(q, qv), quat_conj(q))[1:]


def gravity_tilt_decomposition(nav_quat, world_up=(0.0, 0.0, 1.0)):
    """
    Decompose orientation into (tilt_angle, tilt_azimuth) relative to gravity,
    discarding yaw entirely. This is the physically correct generalization of
    "roll and pitch" — roll/pitch is only a meaningful *separate* pair once
    you pick an arbitrary reference axis to split the tilt cone against, and
    that split is where discontinuities/asymmetric behavior like Jacob's
    ROLL_TRIGGER_THRESH_RANGE-vs-pitch asymmetry creep in.

    Returns
    -------
    tilt_angle : float, radians in [0, pi]
        Angle between the device's local "up" axis and world-up. 0 = flat/
        neutral, pi/2 = fully sideways, independent of which direction it
        tilted in. This is the rotation-invariant analog of "how far tilted."
    tilt_azimuth : float, radians in (-pi, pi]
        Direction of tilt projected onto the horizontal plane (which way it
        tipped). This is yaw-*dependent* by construction (tilting "north"
        vs "east" are genuinely different azimuths) but does NOT inherit
        yaw's slow-drift problem for musical use as long as it's consumed
        as a bounded angle, not integrated — see module docstring in
        mappings/control_surface.py for the drift argument.

    Notes
    -----
    Uses atan2/acos throughout — no branch cuts beyond the unavoidable one
    at tilt_angle == pi (device fully inverted), which is not a pose this
    instrument is expected to pass through in normal play.
    """
    q = quat_normalize(nav_quat)
    local_up_world = quat_rotate(q, (0.0, 0.0, 1.0))  # device +Z sensed in world frame
    up = np.asarray(world_up, dtype=float)
    up = up / np.linalg.norm(up)

    cos_tilt = np.clip(np.dot(local_up_world, up), -1.0, 1.0)
    tilt_angle = np.arccos(cos_tilt)

    # Azimuth: project local_up_world onto the plane perpendicular to `up`,
    # then angle it against an arbitrary-but-fixed reference axis in that
    # plane. Reference axis choice is arbitrary (any in-plane axis works)
    # and does not need per-session calibration the way yaw0 does, because
    # we never integrate this — it's read fresh each frame.
    ref = np.array([1.0, 0.0, 0.0])
    if abs(up[0]) > 0.9:  # guard: ref nearly parallel to up, pick another
        ref = np.array([0.0, 1.0, 0.0])
    ref_in_plane = ref - np.dot(ref, up) * up
    ref_in_plane /= np.linalg.norm(ref_in_plane)
    perp_in_plane = np.cross(up, ref_in_plane)

    proj = local_up_world - cos_tilt * up
    proj_norm = np.linalg.norm(proj)
    if proj_norm < 1e-9:
        tilt_azimuth = 0.0  # tilt_angle ~0, azimuth undefined -> arbitrary but stable
    else:
        proj_unit = proj / proj_norm
        tilt_azimuth = np.arctan2(np.dot(proj_unit, perp_in_plane),
                                   np.dot(proj_unit, ref_in_plane))
    return float(tilt_angle), float(tilt_azimuth)


def quat_relative(q_a, q_b):
    """Relative rotation carrying frame A's orientation to frame B's:
    q_rel = q_a^-1 (x) q_b. Order matters — this is NOT commutative, and
    is deliberately not "q_b - q_a" (Euler subtraction), which breaks at
    wraparound exactly the way wrap_around_count in Jacob's code had to
    patch around after the fact for scalar yaw."""
    return quat_normalize(quat_mul(quat_conj(quat_normalize(q_a)), quat_normalize(q_b)))


def quat_geodesic_angle(q_rel):
    """Shortest-path rotation angle (radians, [0, pi]) encoded by q_rel.
    This is the proper metric on SO(3)/double-cover — robust to sign
    ambiguity (q and -q represent the same rotation)."""
    w = np.clip(abs(q_rel[0]), -1.0, 1.0)
    return float(2.0 * np.arccos(w))


def quat_swing_twist(q_rel, twist_axis=(0.0, 0.0, 1.0)):
    """
    Decompose q_rel into swing (rotation of twist_axis away from itself)
    and twist (rotation about twist_axis), via the standard swing-twist
    factorization. Used to separate "hands rotating apart" (swing, e.g.
    opening a book) from "hands twisting relative to each other about a
    shared axis" (twist, e.g. one hand rotating a doorknob relative to
    the other) — these are different gestures and should drive different
    musical parameters, not be conflated into one scalar difference angle.

    Returns (swing_angle_rad, twist_angle_rad), both in [0, pi] / (-pi, pi].
    """
    q_rel = quat_normalize(q_rel)
    axis = np.asarray(twist_axis, dtype=float)
    axis /= np.linalg.norm(axis)

    w, v = q_rel[0], q_rel[1:]
    proj = np.dot(v, axis) * axis
    twist = quat_normalize(np.array([w, *proj]))
    twist_angle = 2.0 * np.arctan2(np.dot(proj, axis), w) if np.linalg.norm(proj) > 1e-9 or abs(w) > 1e-9 else 0.0

    swing = quat_mul(q_rel, quat_conj(twist))
    swing_angle = quat_geodesic_angle(swing)

    # wrap twist to (-pi, pi]
    twist_angle = float((twist_angle + np.pi) % (2 * np.pi) - np.pi)
    return float(swing_angle), twist_angle


def unwrap_angle(prev_unwrapped, prev_wrapped, curr_wrapped):
    """
    Continuous phase-unwrap for a signal that wraps at +/-pi — the
    proper (non-quantized) analog of Jacob's wrap_around_count, which
    operated on an already-discretized [0,127] channel. Works directly
    on radians so no BYTE/TWO_BYTE scaling constants leak into mapping
    code.
    """
    delta = curr_wrapped - prev_wrapped
    delta = (delta + np.pi) % (2 * np.pi) - np.pi  # shortest signed delta
    return prev_unwrapped + delta


# ---------------------------------------------------------------------------
# Signal-processing primitives (formerly core/dsp.py -- merged here since
# neither file has any music/MIDI knowledge; splitting "quaternion math" from
# "filter math" bought no real separation of concerns).
# ---------------------------------------------------------------------------


class OnePoleSmoother:
    """First-order IIR low-pass, time-constant specified in seconds so
    behavior is independent of the (variable, ~BLE-limited) sample rate.
    y[n] = y[n-1] + alpha * (x[n] - y[n-1]),  alpha = 1 - exp(-dt/tau)."""

    def __init__(self, tau_s, initial=0.0):
        self.tau_s = tau_s
        self.y = initial
        self._initialized = False

    def step(self, x, dt):
        if not self._initialized:
            self.y = x
            self._initialized = True
            return self.y
        alpha = 1.0 - np.exp(-dt / self.tau_s) if self.tau_s > 0 else 1.0
        self.y = self.y + alpha * (x - self.y)
        return self.y


class LeakyEnergyTank:
    """
    Energy accumulator with continuous leak — models a resonant object that
    absorbs kinetic energy on excitation and dissipates it over time,
    rather than an instantaneous velocity-to-volume fader (which has no
    memory and produces silence the instant the hand stops).

        E[n] = max(0, E[n-1] * exp(-dt / tau_decay) + gain * input_power * dt)

    input_power should already be a physically-motivated power-like
    quantity (e.g. kinetic energy rate, or |v|^2), not raw velocity, so
    that a fast brief motion and a slow sustained motion of equal total
    "work" charge the tank comparably. Consuming code chooses the mapping
    from tank level to synthesis parameter (amplitude, filter cutoff, etc).
    """

    def __init__(self, tau_decay_s=1.2, gain=1.0, max_energy=None):
        self.tau_decay_s = tau_decay_s
        self.gain = gain
        self.max_energy = max_energy
        self.energy = 0.0

    def step(self, input_power, dt):
        decay = np.exp(-dt / self.tau_decay_s) if self.tau_decay_s > 0 else 0.0
        self.energy = self.energy * decay + self.gain * max(input_power, 0.0) * dt
        if self.max_energy is not None:
            self.energy = min(self.energy, self.max_energy)
        return self.energy

    def normalized(self):
        if not self.max_energy:
            return self.energy
        return min(self.energy / self.max_energy, 1.0)


class SlewLimiter:
    """Caps the rate of change of a control signal (units/sec). Prevents
    zipper noise / clicks on discretely-quantized downstream params (e.g.
    MIDI CC) when the underlying kinematic signal is noisy or has estimator
    transients (e.g. the drift-then-snap-back window after fast motion
    stops, mentioned in the pipeline caveats) that would otherwise appear
    as an audible glitch rather than a smooth glide."""

    def __init__(self, max_rate_per_s, initial=0.0):
        self.max_rate = max_rate_per_s
        self.y = initial

    def step(self, x, dt):
        max_delta = self.max_rate * dt
        delta = np.clip(x - self.y, -max_delta, max_delta)
        self.y += delta
        return self.y


class HysteresisGate:
    """Minimal Schmitt trigger, kept as a utility (not the primary mapping
    mechanism, unlike Jacob's app) for the few genuinely discrete events
    Tier 1 still needs, e.g. tank-energy crossing an audibility floor."""

    def __init__(self, on_thresh, off_thresh):
        assert on_thresh >= off_thresh
        self.on_thresh, self.off_thresh = on_thresh, off_thresh
        self.state = False

    def step(self, x):
        if not self.state and x > self.on_thresh:
            self.state = True
        elif self.state and x < self.off_thresh:
            self.state = False
        return self.state
