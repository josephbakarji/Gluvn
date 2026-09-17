"""
Gluvn-M5 live dual-hand digital twin (3D visualization).
"""

import queue
import sys
import time
import html
from collections import deque

from core.port_read import Reader

import numpy as np
from scipy.spatial.transform import Rotation
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets

# ======================================================================
# 1. DATA ACQUISITION  decode, drain
# ======================================================================
SCALE = 32767.0

def decode_ypr(yaw_cal: int, pitch_cal: int, roll_cal: int):
    yaw   = yaw_cal   * 180.0 / SCALE - 180.0
    pitch = pitch_cal * 180.0 / SCALE - 90.0
    roll  = roll_cal  * 180.0 / SCALE - 180.0
    return yaw, pitch, roll


def rotation_from_ypr(yaw, pitch, roll) -> np.ndarray:
    return Rotation.from_euler("ZYX", [yaw, pitch, roll], degrees=True).as_matrix()


def rotation_from_quat(quat) -> np.ndarray:
    w, x, y, z = quat
    return Rotation.from_quat([x, y, z, w]).as_matrix()


def ypr_from_rotation(R) -> tuple:
    yaw, pitch, roll = Rotation.from_matrix(R).as_euler("ZYX", degrees=True)
    return float(yaw), float(pitch), float(roll)

G_MS2 = 9.80665

def decode_accel_raw(i0, i1, i2) -> np.ndarray:
    # Firmware reserves imu0 bit 0 for sensor_type.
    raw = np.array([i0 & 0xFFFE, i1, i2], dtype=float)
    accel_g = raw / 16383.5 - 2.0
    return accel_g * G_MS2

def drain_latest(q: "queue.Queue"):
    """Non-blocking drain to the newest sample  bounds render-loop latency."""
    latest = None
    while True:
        try:
            latest = q.get_nowait()
        except queue.Empty:
            return latest


def drain_all(q: "queue.Queue"):
    items = []
    while True:
        try:
            items.append(q.get_nowait())
        except queue.Empty:
            return items


# ======================================================================
# 2. SCENE / SENSOR CONSTANTS
# ======================================================================
ANCHORS = {"l": np.array([-0.1, 0.0, 0.0]), "r": np.array([0.1, 0.0, 0.0])}

CAMERA_DISTANCE = 1.5
CAMERA_ELEVATION_DEG = 14.0
CAMERA_AZIMUTH_DEG = -90.0
GRID_Z_OFFSET = -0.18
SCENE_BG_COLOR = (58, 62, 72)   # lighter dark-gray for clearer hand motion visibility

BODY_AXES = np.eye(3)
AXIS_COLORS = [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)]   # X=red, Y=green, Z=blue
AXIS_NAMES = ["X", "Y", "Z"]
AXIS_LEN = 0.1
AXIS_ARROW_LEN = 0.014
AXIS_ARROW_R = 0.0045

RENDER_HZ = 60
TRAIL_MAXLEN = 150
VEL_VECTOR_TIME_SCALE = 0.3
VEL_VECTOR_COLOR = (1.0, 0.84, 0.37, 1.0)
VEL_ARROW_LEN, VEL_ARROW_R = 0.010, 0.0035

ACCEL_VEC_COLOR = (0.85, 0.35, 1.0, 1.0)     # magenta -- distinct from vel(yellow)/axes(RGB)
ACCEL_VIS_SCALE = 0.005                      # cosmetic-only: m/s^2 -> arrow length
ACCEL_VEC_MAXLEN = 0.09
ACCEL_ARROW_LEN, ACCEL_ARROW_R = 0.009, 0.0032

HAND_COLORS_GL = {"l": (0.25, 0.65, 1.0, 1.0), "r": (1.0, 0.42, 0.25, 1.0)}
HAND_COLORS_CSS = {"l": "#3fa7ff", "r": "#ff6b3f"}
HAND_NAMES = {"l": "LEFT", "r": "RIGHT"}

# M5StickC-Plus-1.1-like proportions (half-extents).
# The IMU axis triad still reads as "coming out of the device" at the
# same scale as before.
_HX, _HY, _HZ = 0.024, 0.048, 0.0135
DEVICE_COLOR = (0.10, 0.11, 0.14, 1.0)
DEVICE_SCREEN_INSET = (0.62, 0.42)   # (w, h) fraction of device face
DEVICE_BUTTON_R = 0.003
DEVICE_BUTTON_LEN = 0.005

FOREARM_LENGTH = 0.25
FOREARM_R_WRIST = 0.025
FOREARM_R_ELBOW = 0.035
FOREARM_SIDES = 10

PALM_RX, PALM_RY, PALM_RZ = 0.040, 0.045, 0.015
PALM_LAT_SEGS, PALM_LON_SEGS = 6, 12
SKIN_BASE = np.array([0.85, 0.70, 0.58])

# --------------------------------------------------------------------
# Hand/finger sensor geometry (box-local frame; rigidly attached to the
# wrist's R -- see module docstring's HAND GEOMETRY note)
# --------------------------------------------------------------------
PALM_FORWARD_LOCAL = np.array([0.0, 1.0, 0.0])
CURL_LOCAL_AXIS = np.array([0.0, 0.0, -1.0])

# anatomical Y-axis anchors relative to the M5Stick center (0.0)
JUNCTION_Y = 0.08
FOREARM_END_Y = -0.06
ARM_Z_OFFSET = -0.022

PALM_DIST = 0.16
FINGER_BASE_SPREAD = np.array([-0.032, -0.016, 0.0, 0.016, 0.032])
FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]
FINGER_L1, FINGER_L2 = 0.045, 0.035
MAX_BEND_DEG = 100.0
FSR_COLOR_LOW, FSR_COLOR_HIGH = (0.2, 0.9, 0.2, 1.0), (1.0, 0.15, 0.15, 1.0)
FSR_SIZE_MIN, FSR_SIZE_MAX = 10.0, 30.0
FLEX_DOT_COLOR = (0.55, 0.80, 1.0, 0.9)      # cyan-ish -- visually distinct from FSR green->red
FLEX_DOT_SIZE_MIN, FLEX_DOT_SIZE_MAX = 4.0, 11.0

FINGER_R_BASE = {"thumb": 0.008, "index": 0.007, "middle": 0.007, "ring": 0.006, "pinky": 0.005}
FINGER_R_TIP_FACTOR = 0.70
FINGER_SIDES = 6

# Cosmetic-only thumb placement (see module docstring) -- does not add a
# sensed DOF, still driven purely by flex[0] + the wrist's R.
THUMB_TILT_DEG = 30.0
THUMB_BASE_OFFSET = np.array([0.015, -0.01, 0.005])

# --------------------------------------------------------------------
# Particle system / energy glow (all purely visual -- see module
# docstring's ACTUAL VS VISUAL section)
# --------------------------------------------------------------------
PARTICLE_CAPACITY = 260
PARTICLE_AMBIENT_BASE_RATE = 1.0          # particles/tick at rest
PARTICLE_AMBIENT_SPEED_GAIN = 40.0        # extra particles/tick per (m/s)
PARTICLE_AMBIENT_MAX_PER_TICK = 10
PARTICLE_BURST_N = 46
PARTICLE_LIFE_RANGE = (0.35, 0.9)         # seconds
PARTICLE_SIZE_RANGE = (3.0, 9.0)
PARTICLE_JITTER = 0.002                    # m, cosmetic dispersion only
PARTICLE_BURST_SPEED = 0.22

ENERGY_VEL_NORM = 0.12          # m/s, visual normalizer (not a physical limit)
ENERGY_ACCEL_NORM = 0.20 * G_MS2  # calibrated m/s^2, visual normalizer
ENERGY_SMOOTH_TAU = 0.015       # s, exponential smoothing time-constant
ENERGY_GLOW_SIZE_MIN, ENERGY_GLOW_SIZE_MAX = 18.0, 46.0

EFFECT_FLASH_DECAY_TAU = 0.35   # s
STATIONARY_SPEED_THRESHOLD = 0.03   # m/s, UI-only threshold (not a calibrated constant)
STALE_DATA_WARNING_SEC = 1.0        # no real sample for a hand this long -> flag it in the UI
SYNC_BURST_WINDOW_SEC = 0.15
HAND_LINK_DISTANCE = 1.4        # m, below which the two-hand proximity field appears


# ======================================================================
# 3. GEOMETRY PRIMITIVES
# ======================================================================
def _combine_meshes(parts):
    """Concatenate several (verts, faces, normals) tuples into one mesh,
    offsetting face indices. Skips empty parts."""
    vs, fs, ns = [], [], []
    offset = 0
    for v, f, n in parts:
        if v is None or len(v) == 0:
            continue
        fs.append(np.asarray(f, dtype=int) + offset)
        vs.append(v)
        ns.append(n)
        offset += len(v)
    if not vs:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=int), np.zeros((0, 3))
    return np.vstack(vs), np.vstack(fs).astype(int), np.vstack(ns)


def transform_local_mesh(local_verts, local_normals, R, origin):
    world_verts = origin + (R @ local_verts.T).T
    world_normals = (R @ local_normals.T).T
    return world_verts, world_normals


def place_cylinder(p0, p1, r0, r1, sides=8, cap0=True, cap1=True):
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    axis = p1 - p0
    length = np.linalg.norm(axis)
    w = axis / length if length > 1e-9 else np.array([0.0, 1.0, 0.0])
    helper = np.array([1.0, 0.0, 0.0]) if abs(w[1]) > 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(helper, w); u /= (np.linalg.norm(u) + 1e-12)
    v = np.cross(w, u)

    thetas = np.linspace(0.0, 2 * np.pi, sides, endpoint=False)
    ring_dirs = np.outer(np.cos(thetas), u) + np.outer(np.sin(thetas), v)   # (sides,3)

    base_ring = p0 + ring_dirs * r0
    tip_ring = p1 + ring_dirs * r1
    verts = [base_ring, tip_ring]
    normals = [ring_dirs, ring_dirs]

    faces = []
    for i in range(sides):
        j = (i + 1) % sides
        # NOTE: vertex order here is deliberately [i, sides+j, sides+i] /
        # [i, j, sides+j] (not the more "obvious" [i, sides+i, sides+j] /
        # [i, sides+j, j]) -- verified numerically against the known
        # analytic outward normal (the radial ring direction) that the
        # naive ordering produces INWARD-facing triangles for every
        # lateral face. Do not swap this back without re-checking.
        faces.append([i, sides + j, sides + i])
        faces.append([i, j, sides + j])

    n_lateral = 2 * sides
    if cap0:
        c0 = n_lateral
        verts.append(p0[None, :]); normals.append((-w)[None, :])
        for i in range(sides):
            j = (i + 1) % sides
            faces.append([c0, j, i])
        n_lateral += 1
    if cap1:
        c1 = n_lateral
        verts.append(p1[None, :]); normals.append(w[None, :])
        for i in range(sides):
            j = (i + 1) % sides
            faces.append([c1, sides + i, sides + j])

    return np.vstack(verts), np.array(faces, dtype=int), np.vstack(normals)


def make_ellipsoid_mesh(rx, ry, rz, lat=8, lon=12):
    thetas = np.linspace(0.0, np.pi, lat + 1)
    phis = np.linspace(0.0, 2 * np.pi, lon, endpoint=False)
    verts, normals = [], []
    for th in thetas:
        st, ct = np.sin(th), np.cos(th)
        for ph in phis:
            sx, sy, sz = st * np.cos(ph), st * np.sin(ph), ct
            verts.append([rx * sx, ry * sy, rz * sz])
            nrm = np.array([sx / rx, sy / ry, sz / rz])
            nrm /= (np.linalg.norm(nrm) + 1e-12)
            normals.append(nrm)
    verts = np.array(verts); normals = np.array(normals)

    def idx(i, j):
        return i * lon + j

    faces = []
    for i in range(lat):
        for j in range(lon):
            j2 = (j + 1) % lon
            a, b, c, d = idx(i, j), idx(i, j2), idx(i + 1, j2), idx(i + 1, j)
            # NOTE: [a, d, b] / [b, d, c], not the more "obvious" [a, b, d]
            # / [b, c, d] -- verified numerically against the analytic
            # outward normal (position/radii direction on the ellipsoid);
            # the naive ordering produces INWARD-facing triangles.
            if i != 0:
                faces.append([a, d, b])
            if i != lat - 1:
                faces.append([b, d, c])
    return verts, np.array(faces, dtype=int), normals


def make_box_mesh(hx, hy, hz):
    """Axis-aligned box, 4 verts/face (24 total) for flat per-face normals."""
    half = np.array([hx, hy, hz])
    faces_def = [(0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)]
    verts, normals, faces = [], [], []
    for axis, sign in faces_def:
        normal = np.zeros(3); normal[axis] = sign
        u = np.zeros(3); u[(axis + 1) % 3] = 1.0
        v = np.zeros(3); v[(axis + 2) % 3] = 1.0
        center = normal * half[axis]
        hu, hv = half[(axis + 1) % 3], half[(axis + 2) % 3]
        corners = [center - u * hu - v * hv, center + u * hu - v * hv,
                   center + u * hu + v * hv, center - u * hu + v * hv]
        if sign < 0:
            corners = corners[::-1]
        base = len(verts)
        verts += corners
        normals += [normal] * 4
        faces += [[base, base + 1, base + 2], [base, base + 2, base + 3]]
    return np.array(verts), np.array(faces, dtype=int), np.array(normals)


def make_quad_mesh(w, h, axis, offset):
    n = np.zeros(3); n[axis] = 1.0 if offset >= 0 else -1.0
    u = np.zeros(3); u[(axis + 1) % 3] = 1.0
    v = np.zeros(3); v[(axis + 2) % 3] = 1.0
    center = n * abs(offset)
    corners = [center - u * w / 2 - v * h / 2, center + u * w / 2 - v * h / 2,
               center + u * w / 2 + v * h / 2, center - u * w / 2 + v * h / 2]
    if offset < 0:
        corners = corners[::-1]
    verts = np.array(corners)
    faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=int)
    normals = np.tile(n, (4, 1))
    return verts, faces, normals


# --------------------------------------------------------------------
# Finger kinematics (physical model unchanged; tilt/offset args are new,
# opt-in, cosmetic-only -- see module docstring)
# --------------------------------------------------------------------
def finger_points_local(spread_x, flex_frac, tilt_deg=0.0, base_offset=None):
    if base_offset is None:
        base_offset = np.zeros(3)
    theta = np.radians(np.clip(flex_frac, 0.0, 1.0) * MAX_BEND_DEG)
    if tilt_deg:
        c, s = np.cos(np.radians(tilt_deg)), np.sin(np.radians(tilt_deg))
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        fwd = Rz @ PALM_FORWARD_LOCAL
        curl_axis = Rz @ CURL_LOCAL_AXIS
    else:
        fwd, curl_axis = PALM_FORWARD_LOCAL, CURL_LOCAL_AXIS
        
    # apply Z-offset to maintain rigid structural attachment to the shifted palm
    base = np.array([spread_x, PALM_DIST, ARM_Z_OFFSET]) + base_offset
    joint = base + FINGER_L1 * fwd
    seg2_dir = np.cos(theta) * fwd + np.sin(theta) * curl_axis
    tip = joint + FINGER_L2 * seg2_dir
    return base, joint, tip


def _finger_geom(hand, i, flex_frac):
    """Returns (base, joint, tip) in box-local frame for a specific hand,
    preserving finger-name mapping while applying static geometric mirror."""
    hand_sign = 1.0 if hand == "r" else -1.0
    spread_x = hand_sign * FINGER_BASE_SPREAD[i]
    name = FINGER_NAMES[i]
    if name == "thumb":
        thumb_offset = THUMB_BASE_OFFSET.copy()
        thumb_offset[0] *= hand_sign
        return finger_points_local(spread_x, flex_frac,
                                     tilt_deg=THUMB_TILT_DEG * hand_sign,
                                     base_offset=thumb_offset)
    return finger_points_local(spread_x, flex_frac)


# ======================================================================
# 4. CACHED LOCAL-FRAME TEMPLATES (built once at import time)
# ======================================================================
_PALM_LOCAL_OFFSET = np.array([0.0, JUNCTION_Y + PALM_RY, ARM_Z_OFFSET])
_palm_v, _palm_f, _palm_n = make_ellipsoid_mesh(PALM_RX, PALM_RY, PALM_RZ, PALM_LAT_SEGS, PALM_LON_SEGS)
PALM_TEMPLATE = (_palm_v + _PALM_LOCAL_OFFSET, _palm_f, _palm_n)

_dev_box = make_box_mesh(_HX, _HY, _HZ)
BTN_Y_OFFSET = _HY * 0.4
_btn_a = place_cylinder(np.array([-_HX, BTN_Y_OFFSET, 0.0]), np.array([-_HX - DEVICE_BUTTON_LEN, BTN_Y_OFFSET, 0.0]),
                         DEVICE_BUTTON_R, DEVICE_BUTTON_R, sides=6)
_btn_b = place_cylinder(np.array([-_HX, -BTN_Y_OFFSET, 0.0]), np.array([-_HX - DEVICE_BUTTON_LEN, -BTN_Y_OFFSET, 0.0]),
                         DEVICE_BUTTON_R, DEVICE_BUTTON_R, sides=6)
DEVICE_BODY_TEMPLATE = _combine_meshes([_dev_box, _btn_a, _btn_b])
DEVICE_SCREEN_TEMPLATE = make_quad_mesh(DEVICE_SCREEN_INSET[0] * 2 * _HX,
                                          DEVICE_SCREEN_INSET[1] * 2 * _HY,
                                          axis=2, offset=_HZ + 0.003)


# ======================================================================
# 5. HandState  shared per-hand representation
# ======================================================================
class HandState:
    def __init__(self, hand):
        self.hand = hand
        self.yaw = self.pitch = self.roll = 0.0
        self.R = np.eye(3)
        self.origin = ANCHORS[hand].copy()
        self.flex = None
        self.press = None
        self.velocity = None
        self.accel_raw = np.zeros(3)
        self.speed = 0.0
        self.accel_mag = 0.0
        self.energy = 0.0
        self.flash_intensity = 0.0
        self.last_effect_t = -1e9
        self.moving = False
        self.has_data = False
        self.low_battery = False
        self.last_sample_wall_time = None

    def update_kinematics(self, yaw, pitch, roll, R, origin, flex, press,
                           velocity, accel_raw, dt):
        self.yaw, self.pitch, self.roll = yaw, pitch, roll
        self.R, self.origin = R, origin
        self.flex, self.press = flex, press
        self.velocity = velocity
        self.accel_raw = accel_raw
        self.speed = float(np.linalg.norm(velocity)) if velocity is not None else 0.0
        self.accel_mag = float(np.linalg.norm(accel_raw))
        self.moving = self.speed > STATIONARY_SPEED_THRESHOLD
        self.has_data = True

        # Exponentially-smoothed [0,1] activity score -- combines velocity,
        # acceleration, FSR and flex activity, and recent effect triggers.
        # Purely a visual driver for the energy glow; never fed back as data.
        vel_term = min(self.speed / ENERGY_VEL_NORM, 1.0)
        acc_term = min(self.accel_mag / ENERGY_ACCEL_NORM, 1.0)
        fsr_term = float(np.mean(press) / 255.0) if press is not None else 0.0
        flex_term = float(np.mean(flex) / 255.0) if flex is not None else 0.0
        raw_energy = 0.35 * vel_term + 0.25 * acc_term + 0.25 * fsr_term + 0.15 * flex_term
        raw_energy = min(1.0, raw_energy + 0.6 * self.flash_intensity)
        alpha = 1.0 - np.exp(-dt / ENERGY_SMOOTH_TAU) if dt > 0 else 1.0
        self.energy += alpha * (raw_energy - self.energy)

        if dt > 0:
            self.flash_intensity *= np.exp(-dt / EFFECT_FLASH_DECAY_TAU)


# ======================================================================
# 6. RENDER COMPONENTS
# ======================================================================
class ForearmModel:
    def __init__(self, view, hand):
        self.color = _skin_tone(hand)
        self.item = gl.GLMeshItem(smooth=True, shader="shaded", glOptions="opaque",
                           color=self.color, drawEdges=False,
                           vertexes=np.zeros((0, 3)), faces=np.zeros((0, 3), dtype=int))
        view.addItem(self.item)

    def update(self, st: HandState):
        p0 = st.origin + st.R @ np.array([0.0, JUNCTION_Y, ARM_Z_OFFSET])
        p1 = st.origin + st.R @ np.array([0.0, FOREARM_END_Y, ARM_Z_OFFSET])
        v, f, n = place_cylinder(p0, p1, FOREARM_R_WRIST, FOREARM_R_ELBOW,
                                  sides=FOREARM_SIDES, cap0=False, cap1=True)
        self.item.setMeshData(vertexes=v, faces=f, color=self.color)


class M5StickModel:
    def __init__(self, view, hand):
        self.body_item = gl.GLMeshItem(smooth=False, shader="shaded", glOptions="opaque",
                                        color=DEVICE_COLOR, drawEdges=False, vertexes=np.zeros((0, 3)), faces=np.zeros((0, 3), dtype=int))
        self.screen_item = gl.GLMeshItem(smooth=False, shader="shaded", glOptions="additive",
                                          drawEdges=False, vertexes=np.zeros((0, 3)), faces=np.zeros((0, 3), dtype=int))
        view.addItem(self.body_item)
        view.addItem(self.screen_item)
        self.screen_base_color = HAND_COLORS_GL[hand]

    def update(self, st: HandState):
        # rigidly center the device mesh on the IMU state origin
        body_origin = st.origin
        screen_origin = st.origin
        
        bv, _ = transform_local_mesh(DEVICE_BODY_TEMPLATE[0], DEVICE_BODY_TEMPLATE[2], st.R, body_origin)
        self.body_item.setMeshData(vertexes=bv, faces=DEVICE_BODY_TEMPLATE[1], color=DEVICE_COLOR)
        sv, _ = transform_local_mesh(DEVICE_SCREEN_TEMPLATE[0], DEVICE_SCREEN_TEMPLATE[2], st.R, screen_origin)
        
        glow = 0.5 + 0.5 * st.energy
        r, g, b, a = self.screen_base_color
        self.screen_item.setMeshData(vertexes=sv, faces=DEVICE_SCREEN_TEMPLATE[1],
                                      color=(r * glow, g * glow, b * glow, 0.85))

def _skin_tone(hand):
    accent = np.array(HAND_COLORS_GL[hand][:3])
    tone = SKIN_BASE * 0.85 + accent * 0.15
    return (tone[0], tone[1], tone[2], 1.0)


class HandModel:
    """Palm (rigid-template ellipsoid) + 5 articulated fingers (fresh
    point-to-point tubes each frame, since each carries its own local
    curl on top of the wrist's R)."""

    def __init__(self, view, hand):
        self.color = _skin_tone(hand)
        self.palm_item = gl.GLMeshItem(smooth=True, shader="shaded", glOptions="opaque",
                                        color=self.color, drawEdges=False, vertexes=np.zeros((0, 3)), faces=np.zeros((0, 3), dtype=int))
        self.fingers_item = gl.GLMeshItem(smooth=True, shader="shaded", glOptions="opaque",
                                           color=self.color, drawEdges=False, vertexes=np.zeros((0, 3)), faces=np.zeros((0, 3), dtype=int))
        view.addItem(self.palm_item)
        view.addItem(self.fingers_item)

    def update(self, st: HandState):
        pv, _ = transform_local_mesh(PALM_TEMPLATE[0], PALM_TEMPLATE[2], st.R, st.origin)
        self.palm_item.setMeshData(vertexes=pv, faces=PALM_TEMPLATE[1], color=self.color)

        if st.flex is None:
            self.fingers_item.setMeshData(vertexes=np.zeros((0, 3)), faces=np.zeros((0, 3), dtype=int))
            return

        parts = []
        for i in range(5):
            flex_frac = st.flex[i] / 255.0
            base, joint, tip = _finger_geom(st.hand, i, flex_frac)
            base_w = st.origin + st.R @ base
            joint_w = st.origin + st.R @ joint
            tip_w = st.origin + st.R @ tip
            r0 = FINGER_R_BASE[FINGER_NAMES[i]]
            r_mid = r0 * 0.9
            r_tip = r0 * FINGER_R_TIP_FACTOR
            parts.append(place_cylinder(base_w, joint_w, r0, r_mid, sides=FINGER_SIDES, cap0=True, cap1=False))
            parts.append(place_cylinder(joint_w, tip_w, r_mid, r_tip, sides=FINGER_SIDES, cap0=False, cap1=True))
        v, f, n = _combine_meshes(parts)
        self.fingers_item.setMeshData(vertexes=v, faces=f, color=self.color)


class FingerSensorVisuals:
    """Fingertip FSR glow (pulsing core + halo) and base-of-finger flex
    indicator dots -- two separate visual channels, never combined."""

    def __init__(self, view):
        self.fsr_core = gl.GLScatterPlotItem(pxMode=True)
        self.fsr_halo = gl.GLScatterPlotItem(pxMode=True)
        self.flex_dots = gl.GLScatterPlotItem(pxMode=True)
        for item in (self.fsr_halo, self.fsr_core, self.flex_dots):
            view.addItem(item)

    def update(self, st: HandState, t: float):
        if st.flex is None or st.press is None:
            for item in (self.fsr_core, self.fsr_halo, self.flex_dots):
                item.setData(pos=np.zeros((0, 3)))
            return

        tips, bases = [], []
        fsr_colors, fsr_sizes = [], []
        flex_colors, flex_sizes = [], []
        pulse = 0.85 + 0.15 * np.sin(t * 6.0)
        for i in range(5):
            flex_frac = float(np.clip(st.flex[i] / 255.0, 0.0, 1.0))
            press_frac = float(np.clip(st.press[i] / 255.0, 0.0, 1.0))
            base, joint, tip = _finger_geom(st.hand, i, flex_frac)
            tips.append(st.origin + st.R @ tip)
            bases.append(st.origin + st.R @ base)
            fsr_colors.append(tuple(lo + press_frac * (hi - lo) for lo, hi in zip(FSR_COLOR_LOW, FSR_COLOR_HIGH)))
            fsr_sizes.append((FSR_SIZE_MIN + press_frac * (FSR_SIZE_MAX - FSR_SIZE_MIN)) * (0.9 + 0.2 * press_frac * pulse))
            flex_colors.append(FLEX_DOT_COLOR)
            flex_sizes.append(FLEX_DOT_SIZE_MIN + flex_frac * (FLEX_DOT_SIZE_MAX - FLEX_DOT_SIZE_MIN))

        tips = np.array(tips); bases = np.array(bases)
        self.fsr_core.setData(pos=tips, color=np.array(fsr_colors), size=np.array(fsr_sizes))
        halo_sizes = np.array(fsr_sizes) * 1.8
        halo_colors = np.array(fsr_colors) * np.array([1, 1, 1, 0.35])
        self.fsr_halo.setData(pos=tips, color=halo_colors, size=halo_sizes)
        self.flex_dots.setData(pos=bases, color=np.array(flex_colors), size=np.array(flex_sizes))


class IMUAxisGizmo:
    """RGB axis lines + small arrowhead cones + static X/Y/Z labels.
    Angle VALUES are deliberately never attached to individual axes --
    see module docstring's DESIGN NOTE."""

    def __init__(self, view):
        self.axis_items = [gl.GLLinePlotItem(width=3, antialias=True, color=c) for c in AXIS_COLORS]
        for item in self.axis_items:
            view.addItem(item)
        self.axis_labels = [gl.GLTextItem(text=name, color=(255, 255, 255, 255)) for name in AXIS_NAMES]
        for label in self.axis_labels:
            view.addItem(label)
        self.arrow_item = gl.GLMeshItem(smooth=False, shader="shaded", glOptions="opaque", drawEdges=False, vertexes=np.zeros((0, 3)), faces=np.zeros((0, 3), dtype=int))
        view.addItem(self.arrow_item)

    def update(self, st: HandState):
        parts = []
        for axis_vec, item, label, color in zip(BODY_AXES, self.axis_items, self.axis_labels, AXIS_COLORS):
            tip = st.origin + AXIS_LEN * (st.R @ axis_vec)
            far = st.origin + (AXIS_LEN + AXIS_ARROW_LEN) * (st.R @ axis_vec)
            item.setData(pos=np.array([st.origin, tip]))
            label.setData(pos=far)
            v, f, n = place_cylinder(tip, far, AXIS_ARROW_R, 0.0, sides=8, cap0=True, cap1=False)
            face_colors = np.tile(np.array(color), (len(f), 1))
            parts.append((v, f, n, face_colors))
        v, f, n = _combine_meshes([(p[0], p[1], p[2]) for p in parts])
        face_colors = np.vstack([p[3] for p in parts]) if parts else np.zeros((0, 4))
        self.arrow_item.setMeshData(vertexes=v, faces=f, faceColors=face_colors)


class VelocityVector:
    def __init__(self, view):
        self.line = gl.GLLinePlotItem(mode="lines", width=4, color=VEL_VECTOR_COLOR, antialias=True)
        self.arrow = gl.GLMeshItem(smooth=False, shader="shaded", glOptions="opaque",
                                    color=VEL_VECTOR_COLOR, drawEdges=False, vertexes=np.zeros((0, 3)), faces=np.zeros((0, 3), dtype=int))
        view.addItem(self.line)
        view.addItem(self.arrow)

    def update(self, st: HandState):
        if st.velocity is None:
            self.line.setData(pos=np.zeros((0, 3)))
            self.arrow.setMeshData(vertexes=np.zeros((0, 3)), faces=np.zeros((0, 3), dtype=int))
            return
        tip = st.origin + np.array(st.velocity) * VEL_VECTOR_TIME_SCALE
        self.line.setData(pos=np.array([st.origin, tip]))
        if np.linalg.norm(tip - st.origin) > 1e-6:
            far = tip + VEL_ARROW_LEN * (tip - st.origin) / np.linalg.norm(tip - st.origin)
            v, f, n = place_cylinder(tip, far, VEL_ARROW_R, 0.0, sides=8, cap0=True, cap1=False)
            self.arrow.setMeshData(vertexes=v, faces=f, color=VEL_VECTOR_COLOR)
        else:
            self.arrow.setMeshData(vertexes=np.zeros((0, 3)), faces=np.zeros((0, 3), dtype=int))


class AccelerationVector:
    """Calibrated, non-gravity-compensated body-frame acceleration from the
    firmware's +/-2 g telemetry encoding."""

    def __init__(self, view):
        self.line = gl.GLLinePlotItem(mode="lines", width=4, color=ACCEL_VEC_COLOR, antialias=True)
        self.arrow = gl.GLMeshItem(smooth=False, shader="shaded", glOptions="opaque",
                                    color=ACCEL_VEC_COLOR, drawEdges=False, vertexes=np.zeros((0, 3)), faces=np.zeros((0, 3), dtype=int))
        view.addItem(self.line)
        view.addItem(self.arrow)

    def update(self, st: HandState):
        vec_world = st.R @ (st.accel_raw * ACCEL_VIS_SCALE)
        length = np.linalg.norm(vec_world)
        if length > ACCEL_VEC_MAXLEN:
            vec_world = vec_world / length * ACCEL_VEC_MAXLEN
            length = ACCEL_VEC_MAXLEN
        tip = st.origin + vec_world
        self.line.setData(pos=np.array([st.origin, tip]))
        if length > 1e-6:
            far = tip + ACCEL_ARROW_LEN * vec_world / length
            v, f, n = place_cylinder(tip, far, ACCEL_ARROW_R, 0.0, sides=8, cap0=True, cap1=False)
            self.arrow.setMeshData(vertexes=v, faces=f, color=ACCEL_VEC_COLOR)
        else:
            self.arrow.setMeshData(vertexes=np.zeros((0, 3)), faces=np.zeros((0, 3), dtype=int))


class ParticleSystem:
    def __init__(self, view, hand, capacity=PARTICLE_CAPACITY):
        self.capacity = capacity
        self.pos = np.zeros((capacity, 3))
        self.vel = np.zeros((capacity, 3))
        self.age = np.full(capacity, 1e9)
        self.life = np.ones(capacity)
        self.base_color = np.array(HAND_COLORS_GL[hand])
        seed = {"l": 1, "r": 2}.get(hand, sum(ord(c) for c in hand))
        self._rng = np.random.default_rng(seed)
        self._cursor = 0
        self.item = gl.GLScatterPlotItem(pxMode=True)
        view.addItem(self.item)

    def _emit(self, origin, n, speed_scale, jitter=PARTICLE_JITTER):
        n = min(n, self.capacity)
        for _ in range(n):
            i = self._cursor
            self._cursor = (self._cursor + 1) % self.capacity
            self.pos[i] = origin + self._rng.normal(scale=jitter, size=3)
            direction = self._rng.normal(size=3)
            norm = np.linalg.norm(direction)
            direction = direction / norm if norm > 1e-9 else np.array([0.0, 0.0, 1.0])
            self.vel[i] = direction * speed_scale * self._rng.uniform(0.3, 1.0)
            self.age[i] = 0.0
            self.life[i] = self._rng.uniform(*PARTICLE_LIFE_RANGE)

    def emit_ambient(self, origin, speed):
        n = int(np.clip(PARTICLE_AMBIENT_BASE_RATE + PARTICLE_AMBIENT_SPEED_GAIN * speed,
                         0, PARTICLE_AMBIENT_MAX_PER_TICK))
        if n > 0:
            self._emit(origin, n, speed_scale=max(speed, 0.15))

    def emit_burst(self, origin):
        self._emit(origin, PARTICLE_BURST_N, speed_scale=PARTICLE_BURST_SPEED, jitter=PARTICLE_JITTER * 2)

    def step(self, dt):
        self.age += dt
        self.pos += self.vel * dt
        self.vel *= 0.92   # gentle drag so bursts settle instead of flying forever

    def render(self):
        alive = self.age < self.life
        if not np.any(alive):
            self.item.setData(pos=np.zeros((0, 3)))
            return
        frac = 1.0 - (self.age[alive] / self.life[alive])
        colors = np.tile(self.base_color, (frac.shape[0], 1))
        colors[:, 3] = np.clip(frac, 0.0, 1.0) * 0.85
        sizes = PARTICLE_SIZE_RANGE[0] + frac * (PARTICLE_SIZE_RANGE[1] - PARTICLE_SIZE_RANGE[0])
        self.item.setData(pos=self.pos[alive], color=colors, size=sizes)


class EnergyGlow:
    """Subtle activity-driven glow point at the hand -- kept deliberately
    small/translucent so the hand mesh stays visually dominant."""

    def __init__(self, view, hand):
        self.color = np.array(HAND_COLORS_GL[hand])
        self.item = gl.GLScatterPlotItem(pxMode=True)
        view.addItem(self.item)

    def update(self, st: HandState):
        size = ENERGY_GLOW_SIZE_MIN + st.energy * (ENERGY_GLOW_SIZE_MAX - ENERGY_GLOW_SIZE_MIN)
        alpha = 0.10 + 0.25 * st.energy
        color = self.color.copy(); color[3] = alpha
        self.item.setData(pos=np.array([st.origin]), color=np.array([color]), size=np.array([size]))


class TwoHandLink:
    def __init__(self, view):
        self.line = gl.GLLinePlotItem(mode="lines", width=2, antialias=True)
        view.addItem(self.line)

    def update(self, states):
        if len(states) < 2:
            self.line.setData(pos=np.zeros((0, 3)))
            return
        a, b = states[0], states[1]
        dist = float(np.linalg.norm(a.origin - b.origin))
        if dist < HAND_LINK_DISTANCE and a.has_data and b.has_data:
            closeness = 1.0 - dist / HAND_LINK_DISTANCE
            alpha = 0.08 + 0.35 * closeness
            color = (0.8, 0.8, 0.9, alpha)
            self.line.setData(pos=np.array([a.origin, b.origin]), color=color)
        else:
            self.line.setData(pos=np.zeros((0, 3)))


class HandVisual:
    """Bundles every per-hand render component; one instance per hand."""

    def __init__(self, view, hand):
        self.hand = hand
        self.forearm = ForearmModel(view, hand)
        self.device = M5StickModel(view, hand)
        self.hand_model = HandModel(view, hand)
        self.sensors = FingerSensorVisuals(view)
        self.axes = IMUAxisGizmo(view)
        self.velocity = VelocityVector(view)
        self.accel = AccelerationVector(view)
        self.particles = ParticleSystem(view, hand)
        self.energy = EnergyGlow(view, hand)
        self.trail_item = gl.GLLinePlotItem(mode="line_strip", width=2, antialias=True)
        view.addItem(self.trail_item)
        self.base_color = HAND_COLORS_GL[hand]

    def _trail_colors(self, n):
        alphas = np.linspace(0.05, 0.55, n)
        colors = np.tile(np.array(self.base_color, dtype=float), (n, 1))
        colors[:, 3] = alphas
        return colors

    def update(self, st: HandState, trail_points, t: float, dt: float):
        self.forearm.update(st)
        self.device.update(st)
        self.hand_model.update(st)
        self.sensors.update(st, t)
        self.axes.update(st)
        self.velocity.update(st)
        self.accel.update(st)
        self.energy.update(st)

        if trail_points is not None and len(trail_points) >= 2:
            self.trail_item.setData(pos=np.array(trail_points), color=self._trail_colors(len(trail_points)))
        else:
            self.trail_item.setData(pos=np.zeros((0, 3)))

        self.particles.emit_ambient(st.origin, st.speed)
        self.particles.step(dt)
        self.particles.render()


# ======================================================================
# 7. TELEMETRY (Qt widgets)
# ======================================================================
class TelemetryPanel(QtWidgets.QWidget):
    """Per-hand dashboard: YPR, position, velocity, calibrated accel, finger
    flex/FSR bars, moving state, current effect."""

    def __init__(self, hand, parent=None):
        super().__init__(parent)
        self.hand = hand
        css = HAND_COLORS_CSS[hand]
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)

        title = QtWidgets.QLabel(f"{HAND_NAMES[hand]} HAND")
        title.setStyleSheet(f"color:{css}; font: bold 13px 'Consolas','Menlo',monospace; letter-spacing:1px;")
        outer.addWidget(title)

        self.orient_lbl = self._mono_label(outer)
        self.pos_lbl = self._mono_label(outer)
        self.vel_lbl = self._mono_label(outer)
        self.accel_lbl = self._mono_label(outer)
        self.state_lbl = self._mono_label(outer)
        outer.addWidget(self._divider())

        outer.addWidget(self._divider())
        flex_hdr = QtWidgets.QLabel("FLEX (curl)")
        flex_hdr.setStyleSheet("color:#7fa8cc; font: 10px 'Consolas',monospace;")
        outer.addWidget(flex_hdr)
        self.flex_bars = self._make_bars(outer, "#4fa8ff")

        fsr_hdr = QtWidgets.QLabel("FSR (contact force)")
        fsr_hdr.setStyleSheet("color:#cc8f7f; font: 10px 'Consolas',monospace;")
        outer.addWidget(fsr_hdr)
        self.fsr_bars = self._make_bars(outer, "#ff7f5f")

        outer.addWidget(self._divider())
        self._effect_history = deque(maxlen=4)   # most-recent-first; short so the panel stays uncluttered

        self.effect_lbl = QtWidgets.QLabel("effect: \u2014")
        self.effect_lbl.setWordWrap(True)
        self.effect_lbl.setTextFormat(QtCore.Qt.TextFormat.RichText)
        self.effect_lbl.setStyleSheet("color:#ffd75f; font: 11px 'Consolas','Menlo',monospace;")
        outer.addWidget(self.effect_lbl)
        self.setStyleSheet("background:#161821; border-radius:6px;")
        self.setMinimumWidth(210)
        self.setMaximumWidth(260)

    def _mono_label(self, layout):
        lbl = QtWidgets.QLabel("\u2014")
        lbl.setStyleSheet("color:#cfd3dc; font: 11px 'Consolas','Menlo',monospace;")
        layout.addWidget(lbl)
        return lbl

    def _divider(self):
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        line.setStyleSheet("color:#2a2d38;")
        return line

    def _make_bars(self, layout, chunk_color):
        bars = []
        for name in FINGER_NAMES:
            row = QtWidgets.QHBoxLayout()
            lbl = QtWidgets.QLabel(name[:4])
            lbl.setFixedWidth(32)
            lbl.setStyleSheet("color:#888; font: 10px 'Consolas',monospace;")
            bar = QtWidgets.QProgressBar()
            bar.setRange(0, 255)
            bar.setTextVisible(False)
            bar.setFixedHeight(8)
            bar.setStyleSheet(
                "QProgressBar{background:#0e0f13; border:none; border-radius:4px;}"
                f"QProgressBar::chunk{{background:{chunk_color}; border-radius:4px;}}"
            )
            row.addWidget(lbl); row.addWidget(bar)
            layout.addLayout(row)
            bars.append(bar)
        return bars

    def update(self, st: HandState):
        self.orient_lbl.setText(f"yaw {st.yaw:+6.1f}\u00b0 pitch {st.pitch:+6.1f}\u00b0 roll {st.roll:+6.1f}\u00b0")
        ox, oy, oz = st.origin
        self.pos_lbl.setText(f"pos   x{ox:+5.2f} y{oy:+5.2f} z{oz:+5.2f}")
        if st.velocity is not None:
            vx, vy, vz = st.velocity
            self.vel_lbl.setText(f"vel   x{vx:+5.2f} y{vy:+5.2f} z{vz:+5.2f}  |{st.speed:4.2f}|")
        else:
            self.vel_lbl.setText("vel   n/a (MOTION_STREAM_ON not active)")
        ax, ay, az = st.accel_raw
        self.accel_lbl.setText(f"acc   x{ax:+6.2f} y{ay:+6.2f} z{az:+6.2f}  "f"|{st.accel_mag:5.2f}| m/s^2")
        batt_suffix = "  \u26a0 LOW BATTERY" if st.low_battery else ""
        stale_suffix = ""
        if st.last_sample_wall_time is not None:
            age = time.monotonic() - st.last_sample_wall_time
            if age > STALE_DATA_WARNING_SEC:
                stale_suffix = f"  \u26a0 STALE {age:4.1f}s"
        self.state_lbl.setText(f"state {'MOVING' if st.moving else 'stationary'}   energy {st.energy:4.2f}{batt_suffix}{stale_suffix}")
        if st.flex is not None:
            for i, bar in enumerate(self.flex_bars):
                bar.setValue(int(np.clip(st.flex[i], 0, 255)))
        if st.press is not None:
            for i, bar in enumerate(self.fsr_bars):
                bar.setValue(int(np.clip(st.press[i], 0, 255)))

    def set_effect(self, text):
        self._effect_history.appendleft(text)
        lines = []
        for i, t in enumerate(self._effect_history):
            safe = html.escape(t)
            if i == 0:
                lines.append(f'<b>\u25b8 {safe}</b>')
            else:
                fade = max(180 - i * 45, 70)   # dims with age
                lines.append(f'<span style="color:#{fade:02x}{fade:02x}70;">&nbsp;&nbsp;{safe}</span>')
        self.effect_lbl.setText("<br>".join(lines))


class EffectNotification(QtWidgets.QWidget):
    """Animated 'GESTURE DETECTED' overlay banner, drawn above the 3D
    view via a StackAll QStackedLayout (see TwinWindow). Purely visual;
    never alters sensor state."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 24, 0, 0)
        self.banner = QtWidgets.QLabel("", self)
        self.banner.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.banner.setStyleSheet(
            "background: rgba(20,22,30,210); color:#ffd75f; "
            "font: bold 20px 'Consolas','Menlo',monospace; "
            "border-radius: 10px; padding: 10px 22px;"
        )
        layout.addWidget(self.banner, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter)
        layout.addStretch(1)
        self.setStyleSheet("background: transparent;")

        self._effect = QtWidgets.QGraphicsOpacityEffect(self.banner)
        self.banner.setGraphicsEffect(self._effect)
        self._effect.setOpacity(0.0)
        self._anim = QtCore.QPropertyAnimation(self._effect, b"opacity")
        self._hide_timer = QtCore.QTimer(self)
        self._hide_timer.setSingleShot(True)
        self._hide_timer.timeout.connect(self._fade_out)

    def show_event(self, hand, text, hold_ms=1400, fade_ms=220):
        css = HAND_COLORS_CSS.get(hand, "#ffd75f")
        self.banner.setText(f"\u2726 GESTURE DETECTED [{HAND_NAMES.get(hand, '?')}] \u2726\n{text}")
        self.banner.setStyleSheet(
            f"background: rgba(20,22,30,220); color:{css}; "
            "font: bold 20px 'Consolas','Menlo',monospace; "
            "border-radius: 10px; padding: 10px 22px;"
        )
        self._anim.stop()
        self._anim.setDuration(fade_ms)
        self._anim.setStartValue(self._effect.opacity())
        self._anim.setEndValue(1.0)
        self._anim.start()
        self._hide_timer.start(hold_ms)

    def _fade_out(self):
        self._anim.stop()
        self._anim.setDuration(400)
        self._anim.setStartValue(self._effect.opacity())
        self._anim.setEndValue(0.0)
        self._anim.start()

# ======================================================================
# 9. TWIN WINDOW
# ======================================================================
DARK_QSS = """
QWidget { background: #0e0f13; }
QToolTip { color: #cfd3dc; background: #161821; border: 1px solid #2a2d38; }
QPushButton {
    background: #1c1f29; color: #cfd3dc; border: 1px solid #2a2d38;
    border-radius: 5px; padding: 5px 10px; font: 11px 'Consolas','Menlo',monospace;
}
QPushButton:hover { background: #262a37; }
QPushButton:pressed { background: #14151b; }
QLabel { color: #cfd3dc; }
QSlider::groove:horizontal { background: #1c1f29; height: 4px; border-radius: 2px; }
QSlider::handle:horizontal {
    background: #4fa8ff; width: 12px; margin: -5px 0; border-radius: 6px;
}
QGroupBox {
    color: #7fa8cc; font: 10px 'Consolas','Menlo',monospace;
    border: 1px solid #2a2d38; border-radius: 6px; margin-top: 8px; padding-top: 6px;
}
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
"""


class ThresholdPanel(QtWidgets.QWidget):
    SPECS = [
        ('accel_trigger_thresh', 'accel trigger thresh', 0, 127, 110),
        ('accel_trigger_hysteresis', 'accel hysteresis', 0, 40, 10),
        ('roll_trigger_thresh_range', 'roll thresh range', 0, 63, 20),
        ('roll_trigger_hysteresis', 'roll hysteresis', 0, 30, 5),
        ('pitch_trigger_thresh_range', 'pitch thresh range', 0, 63, 12),
        ('pitch_trigger_hysteresis', 'pitch hysteresis', 0, 30, 5),
        ('yaw_window', 'yaw window', 0, 63, 10),
        ('press_thresh', 'press thresh', 0, 255, 20),
        ('press_hysteresis', 'press hysteresis', 0, 40, 5),
        ('flex_thresh', 'flex thresh', 0, 255, 100),
        ('flex_hysteresis', 'flex hysteresis', 0, 40, 5),
    ]

    def __init__(self, live_controls, parent=None):
        super().__init__(parent)
        self.live_controls = live_controls
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(8, 5, 8, 5)
        outer.setSpacing(2)

        group = QtWidgets.QGroupBox("LIVE THRESHOLDS")
        group.setMinimumHeight(166)
        form = QtWidgets.QGridLayout(group)
        form.setContentsMargins(10, 10, 10, 8)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(5)
        self.value_labels = {}

        split = (len(self.SPECS) + 1) // 2
        for column, specs in enumerate((self.SPECS[:split], self.SPECS[split:])):
            for row_idx, (key, label, lo, hi, default) in enumerate(specs):
                name_lbl = QtWidgets.QLabel(label)
                name_lbl.setMinimumWidth(150)
                name_lbl.setStyleSheet(
                    "color:#d8dce5; font: 11px 'Consolas','Menlo',monospace;"
                )

                slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
                slider.setRange(lo, hi)
                slider.setMinimumHeight(22)
                initial = default if live_controls is None else live_controls.get_threshold(key, default)
                slider.setValue(int(initial))

                value_lbl = QtWidgets.QLabel(str(int(initial)))
                value_lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                value_lbl.setMinimumWidth(48)
                value_lbl.setStyleSheet(
                    "color:#ffd75f; font: bold 14px 'Consolas','Menlo',monospace;"
                )
                slider.valueChanged.connect(
                    lambda value, k=key, lbl=value_lbl: self._on_change(k, value, lbl)
                )

                form.addWidget(name_lbl, row_idx, column * 3)
                form.addWidget(slider, row_idx, column * 3 + 1)
                form.addWidget(value_lbl, row_idx, column * 3 + 2)
                self.value_labels[key] = value_lbl

        outer.addWidget(group)
        self.setEnabled(live_controls is not None)

    def _on_change(self, key, value, label):
        label.setText(str(value))
        if self.live_controls is not None:
            self.live_controls.set_threshold(key, value)


class TwinWindow(QtWidgets.QWidget):

    def __init__(self, dataq, effect_q=None, hands=("l", "r"), send_command_fn=None, live_controls=None):
        super().__init__()
        self.dataq = dataq
        self.effect_q = effect_q
        self.hands = [h for h in hands if h in dataq]
        self.send_command_fn = send_command_fn
        self.live_controls = live_controls

        self.hand_states = {h: HandState(h) for h in self.hands}
        self.trails = {h: deque(maxlen=None) for h in self.hands}

        self._t_start = time.monotonic()
        self._last_tick = self._t_start

        self.setWindowTitle("Gluvn  live digital twin")
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.setStyleSheet(DARK_QSS)

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # ---- LH telemetry | 3D view | RH telemetry --------------------
        main_row = QtWidgets.QHBoxLayout()
        main_row.setContentsMargins(0, 0, 0, 0)
        main_row.setSpacing(8)

        def make_side_telemetry(hand):
            panel = TelemetryPanel(hand)
            panel.setMinimumHeight(376)
            panel.setMaximumHeight(420)
            holder = QtWidgets.QWidget()
            holder.setFixedWidth(270)
            layout = QtWidgets.QVBoxLayout(holder)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignTop)
            layout.addWidget(panel)
            layout.addStretch(1)
            return holder, panel

        self.telemetry = {}

        # LH telemetry sits to the left of the 3D model.
        if "l" in self.hands:
            lh_holder, self.telemetry["l"] = make_side_telemetry("l")
            main_row.addWidget(lh_holder, stretch=0, alignment=QtCore.Qt.AlignmentFlag.AlignTop)

        # Center: 3D digital twin + gesture notification overlay.
        center_stack_holder = QtWidgets.QWidget()
        self.center_stack = QtWidgets.QStackedLayout(center_stack_holder)
        self.center_stack.setStackingMode(QtWidgets.QStackedLayout.StackingMode.StackAll)

        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor(SCENE_BG_COLOR)
        self._reset_camera_view()
        grid = gl.GLGridItem()
        grid.setSize(2, 2, 1)
        grid.setSpacing(0.05, 0.05, 1)
        grid.translate(0.0, 0.0, GRID_Z_OFFSET)
        self.view.addItem(grid)
        self.view.installEventFilter(self)
        self.center_stack.addWidget(self.view)

        self.notification = EffectNotification()
        self.center_stack.addWidget(self.notification)
        main_row.addWidget(center_stack_holder, stretch=1)

        # RH telemetry sits to the right of the 3D model.
        if "r" in self.hands:
            rh_holder, self.telemetry["r"] = make_side_telemetry("r")
            main_row.addWidget(rh_holder, stretch=0, alignment=QtCore.Qt.AlignmentFlag.AlignTop)

        root.addLayout(main_row, stretch=1)

        # ---- compact keyboard/control strip ---------------------------
        controls_row = QtWidgets.QHBoxLayout()
        controls_row.setSpacing(5)
        self.position_lock_btn = QtWidgets.QPushButton("POS LOCK OFF  [B]")
        self.position_lock_btn.setCheckable(True)
        self.position_lock_btn.toggled.connect(self._toggle_position_lock)
        self.gyro_lock_btn = QtWidgets.QPushButton("GYRO LOCK OFF  [G]")
        self.gyro_lock_btn.setCheckable(True)
        self.gyro_lock_btn.toggled.connect(self._toggle_gyro_lock)
        self.trace_hold_btn = QtWidgets.QPushButton("TRACE DRAW OFF  [T]")
        self.trace_hold_btn.setCheckable(True)
        self.trace_hold_btn.toggled.connect(self._toggle_trace_hold)
        self.clear_trace_btn = QtWidgets.QPushButton("CLEAR TRACE  [C]")
        self.clear_trace_btn.clicked.connect(self._clear_trace)
        self.vibrato_btn = QtWidgets.QPushButton("VIBRATO ON  [M]")
        self.vibrato_btn.setCheckable(True)
        self.vibrato_btn.toggled.connect(self._toggle_vibrato)
        controls_row.addStretch(1)
        for widget in (self.position_lock_btn, self.gyro_lock_btn, self.trace_hold_btn,
                       self.clear_trace_btn, self.vibrato_btn):
            controls_row.addWidget(widget)
        controls_row.addStretch(1)
        root.addLayout(controls_row)

        controls_text = "[R] recenter facing" if self.send_command_fn else "[R] recenter facing (unavailable, no command channel)"
        controls_text += "   [P] reset position (clears accumulated drift)" if self.send_command_fn else "   [P] reset position (unavailable)"
        controls_text += "   [B] position lock   [G] gyro lock   [T] trace draw   [C] clear trace   [M] vibrato"
        controls = QtWidgets.QLabel(controls_text)
        controls.setStyleSheet("color:#666; font: 10px 'Consolas','Menlo',monospace;")
        controls.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        root.addWidget(controls)

        legend = QtWidgets.QLabel(
            "curl \u2190 flex sensor   |   fingertip glow: green=low FSR force, red=high   |   "
            "magenta arrow = calibrated accel (not gravity-compensated)"
        )
        legend.setStyleSheet("color:#777; font: 10px 'Consolas','Menlo',monospace;")
        legend.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        root.addWidget(legend)

        # ---- bottom threshold dock ------------------------------------
        self.threshold_panel = ThresholdPanel(self.live_controls)
        root.addWidget(self.threshold_panel)

        # ---- hand visuals + two-hand link ------------------------------
        self.hand_visuals = {h: HandVisual(self.view, h) for h in self.hands}
        self.link = TwoHandLink(self.view)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(int(1000 / RENDER_HZ))
        self.position_lock = False
        self.gyro_lock = False
        self.trace_hold = False
        self.position_anchor = {h: ANCHORS[h].copy() for h in self.hands}
        self.position_origin_ref = {h: None for h in self.hands}
        self.position_rezero_pending = {h: True for h in self.hands}
        self._update_position_lock_button()
        self._update_gyro_lock_button()
        self._update_trace_hold_button()
        self.vibrato_btn.setEnabled(self.live_controls is not None)
        self._update_vibrato_button()

    # ------------------------------------------------------------------
    # camera / command shortcuts
    # ------------------------------------------------------------------
    def _reset_camera_view(self):
        self.view.setCameraPosition(distance=CAMERA_DISTANCE, elevation=CAMERA_ELEVATION_DEG,
                                     azimuth=CAMERA_AZIMUTH_DEG)

    def _recenter_facing(self):
        if self.send_command_fn is None:
            return
        for hand in self.hands:
            self.send_command_fn(hand, "RESET_YAW")

    def _reset_position(self):
        if self.send_command_fn is None:
            return
        for hand in self.hands:
            self.send_command_fn(hand, "RESET_POSITION")

    def eventFilter(self, obj, event):
        if obj is self.view and event.type() == QtCore.QEvent.Type.KeyPress:
            if self._handle_key(event.key()):
                return True
        return super().eventFilter(obj, event)

    def keyPressEvent(self, event):
        if not self._handle_key(event.key()):
            super().keyPressEvent(event)

    def _handle_key(self, key):
        Key = QtCore.Qt.Key
        if key == Key.Key_R:
            self._recenter_facing()
        elif key == Key.Key_P:
            self._reset_position()
        elif key == Key.Key_V:
            self._reset_camera_view()
        elif key == Key.Key_B:
            self._toggle_position_lock()
        elif key == Key.Key_G:
            self._toggle_gyro_lock()
        elif key == Key.Key_T:
            self._toggle_trace_hold()
        elif key == Key.Key_C:
            self._clear_trace()
        elif key == Key.Key_M:
            self._toggle_vibrato()
        else:
            return False
        return True

    # position lock - gyro only 
    def _update_position_lock_button(self):
        if not hasattr(self, "position_lock_btn"):
            return
        if self.position_lock:
            self.position_lock_btn.setText("POS LOCK ON  [B]")
            self.position_lock_btn.setStyleSheet("background:#1f3a2d; color:#c6f0d6; border:1px solid #4bd989; border-radius:5px; padding:5px 10px; font: 11px 'Consolas','Menlo',monospace;")
        else:
            self.position_lock_btn.setText("POS LOCK OFF  [B]")
            self.position_lock_btn.setStyleSheet("background:#1c1f29; color:#cfd3dc; border:1px solid #2a2d38; border-radius:5px; padding:5px 10px; font: 11px 'Consolas','Menlo',monospace;")
        self.position_lock_btn.setChecked(self.position_lock)

    def _toggle_position_lock(self, checked=None):
        if checked is None:
            checked = not self.position_lock
        self.position_lock = bool(checked)
        if self.position_lock:
            self.gyro_lock = False
            for hand in self.hands:
                self.position_anchor[hand] = ANCHORS[hand].copy()
                self.position_origin_ref[hand] = None
                self.position_rezero_pending[hand] = True
                self.trails[hand].clear()
                self.hand_states[hand].origin = self.position_anchor[hand].copy()
        else:
            for hand in self.hands:
                self.position_origin_ref[hand] = None
                self.position_rezero_pending[hand] = True
                self.trails[hand].clear()
                self.hand_states[hand].origin = self.position_anchor[hand].copy()
        self._update_position_lock_button()
        self._update_gyro_lock_button()

    def _update_gyro_lock_button(self):
        if not hasattr(self, "gyro_lock_btn"):
            return
        if self.gyro_lock:
            self.gyro_lock_btn.setText("GYRO LOCK ON  [G]")
            self.gyro_lock_btn.setStyleSheet("background:#1f3a2d; color:#c6f0d6; border:1px solid #4bd989; border-radius:5px; padding:5px 10px; font: 11px 'Consolas','Menlo',monospace;")
        else:
            self.gyro_lock_btn.setText("GYRO LOCK OFF  [G]")
            self.gyro_lock_btn.setStyleSheet("background:#1c1f29; color:#cfd3dc; border:1px solid #2a2d38; border-radius:5px; padding:5px 10px; font: 11px 'Consolas','Menlo',monospace;")
        self.gyro_lock_btn.setChecked(self.gyro_lock)

    def _update_trace_hold_button(self):
        if not hasattr(self, "trace_hold_btn"):
            return
        if self.trace_hold:
            self.trace_hold_btn.setText("TRACE DRAW ON  [T]")
            self.trace_hold_btn.setStyleSheet("background:#1f3a2d; color:#c6f0d6; border:1px solid #4bd989; border-radius:5px; padding:5px 10px; font: 11px 'Consolas','Menlo',monospace;")
        else:
            self.trace_hold_btn.setText("TRACE DRAW OFF  [T]")
            self.trace_hold_btn.setStyleSheet("background:#1c1f29; color:#cfd3dc; border:1px solid #2a2d38; border-radius:5px; padding:5px 10px; font: 11px 'Consolas','Menlo',monospace;")
        self.trace_hold_btn.setChecked(self.trace_hold)

    def _clear_trace(self):
        for hand in self.hands:
            self.trails[hand].clear()

    def _toggle_gyro_lock(self, checked=None):
        if checked is None:
            checked = not self.gyro_lock
        self.gyro_lock = bool(checked)
        if self.gyro_lock:
            self.position_lock = False
        self._update_position_lock_button()
        self._update_gyro_lock_button()

    def _toggle_trace_hold(self, checked=None):
        if checked is None:
            checked = not self.trace_hold
        self.trace_hold = bool(checked)
        for hand in self.hands:
            if self.trails[hand].maxlen is not None:
                self.trails[hand] = deque(self.trails[hand], maxlen=None)
        self._update_trace_hold_button()

    def _update_vibrato_button(self):
        if not hasattr(self, "vibrato_btn"):
            return
        enabled = self.live_controls.get_vibrato_enabled() if self.live_controls is not None else True
        if enabled:
            self.vibrato_btn.setText("VIBRATO ON  [M]")
            self.vibrato_btn.setStyleSheet("background:#1f3a2d; color:#c6f0d6; border:1px solid #4bd989; border-radius:5px; padding:5px 10px; font: 11px 'Consolas','Menlo',monospace;")
        else:
            self.vibrato_btn.setText("VIBRATO OFF  [M]")
            self.vibrato_btn.setStyleSheet("background:#1c1f29; color:#cfd3dc; border:1px solid #2a2d38; border-radius:5px; padding:5px 10px; font: 11px 'Consolas','Menlo',monospace;")
        self.vibrato_btn.setChecked(enabled)

    def _toggle_vibrato(self, checked=None):
        if self.live_controls is None:
            return
        if checked is None:
            checked = not self.live_controls.get_vibrato_enabled()
        self.live_controls.set_vibrato_enabled(bool(checked))
        self._update_vibrato_button()

    # ------------------------------------------------------------------
    # shared ingestion path
    # ------------------------------------------------------------------
    def _apply_sample_to_hand(self, hand, sample, t, dt):
        if "imu" not in sample:
            return
        imu = sample["imu"]
        yaw_cal, pitch_cal, roll_cal = imu[0], imu[1], imu[2]
        i0, i1, i2 = imu[3:6] if len(imu) >= 6 else (list(imu[3:]) + [0, 0, 0])[:3]
        yaw, pitch, roll = decode_ypr(yaw_cal, pitch_cal, roll_cal)
        R = rotation_from_ypr(yaw, pitch, roll)
        st = self.hand_states[hand]
        if self.gyro_lock and st.has_data:
            yaw, pitch, roll = st.yaw, st.pitch, st.roll
            R = st.R
        accel = (decode_accel_raw(i0, i1, i2)
                   if sample.get("sensor_type", 0) == 0
                   else np.zeros(3))
        sample_pos = np.array(sample["position"]) if "position" in sample else None
        if self.position_lock:
            origin = self.position_anchor[hand].copy()
        elif sample_pos is not None:
            if self.position_rezero_pending[hand] or self.position_origin_ref[hand] is None:
                self.position_origin_ref[hand] = sample_pos.copy()
                self.position_rezero_pending[hand] = False
            origin = self.position_anchor[hand] + (sample_pos - self.position_origin_ref[hand])
        else:
            origin = st.origin if st.has_data else self.position_anchor[hand].copy()
        velocity = None if self.position_lock else sample.get("velocity")
        st.update_kinematics(yaw, pitch, roll, R, origin,
                              flex=sample.get("flex"), press=sample.get("press"),
                              velocity=velocity,
                              accel_raw=accel, dt=dt)
        st.low_battery = bool(sample.get("low_battery", False))
        st.last_sample_wall_time = time.monotonic()
        self.hand_visuals[hand].update(st, list(self.trails[hand]), t, dt)
        if hand in self.telemetry:
            self.telemetry[hand].update(st)

    def _apply_effect(self, status):
        hand = status.get("hand")
        text = status.get("text", "")
        # With two gloves streaming concurrently an effect event with a
        # missing/unrecognized hand tag is ambiguous -- previously this fell
        # through to an uncoloured, unattributed banner instead of being
        # rejected, which silently masked which glove actually fired.
        if hand not in self.hand_states:
            return
        if hand in self.telemetry:
            self.telemetry[hand].set_effect(text)
        self.notification.show_event(hand, text)

        now = time.monotonic()
        st = self.hand_states[hand]
        st.flash_intensity = 1.0
        st.last_effect_t = now
        if hand in self.hand_visuals:
            self.hand_visuals[hand].particles.emit_burst(st.origin)

        others = [h for h in self.hands if h != hand
                  and (now - self.hand_states[h].last_effect_t) < SYNC_BURST_WINDOW_SEC]
        if others:
            mid = st.origin.copy()
            for h in others:
                mid = mid + self.hand_states[h].origin
            mid /= (len(others) + 1)
            for h in [hand] + others:
                self.hand_visuals[h].particles.emit_burst(mid)

    # ------------------------------------------------------------------
    # main render tick
    # ------------------------------------------------------------------
    
    def update_frame(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        self._dt = dt
        t = now - self._t_start
        for hand in self.hands:
            samples = drain_all(self.dataq[hand])
            newest_imu_sample = None
            for s in samples:
                if "imu" not in s:
                    continue
                # position_lock ("orientation-only mode") intentionally
                # freezes the rendered hand at its anchor -- don't let the
                # trail keep drawing from the still-drifting raw position
                # underneath it while locked.
                if "position" in s and not self.position_lock and self.trace_hold:
                    self.trails[hand].append(ANCHORS[hand] + np.array(s["position"]))
                newest_imu_sample = s
            if newest_imu_sample is not None:
                self._apply_sample_to_hand(hand, newest_imu_sample, t, dt)
            elif self.position_lock:
                # No fresh packet this tick for this hand (normal when two
                # gloves share one BLE link and their samples interleave
                # unevenly). Lock/unlock state is a rendering decision, not
                # a sensor value, so it must not wait on the next packet --
                # otherwise a starved hand visibly "ignores" the [B] press
                # until its next sample happens to arrive. Re-pin the
                # already-known orientation/flex/etc. to the anchor now.
                st = self.hand_states[hand]
                anchor = self.position_anchor[hand].copy()
                if st.has_data:
                    st.update_kinematics(st.yaw, st.pitch, st.roll, st.R, anchor,
                                          flex=st.flex, press=st.press,
                                          velocity=None, accel_raw=st.accel_raw, dt=dt)
                else:
                    st.origin = anchor
                self.hand_visuals[hand].update(st, list(self.trails[hand]), t, dt)
                if hand in self.telemetry:
                    self.telemetry[hand].update(st)
        if self.effect_q is not None:
            for status in drain_all(self.effect_q):
                self._apply_effect(status)
        self.link.update([self.hand_states[h] for h in self.hands if self.hand_states[h].has_data])

def build_app_and_window(dataq=None, effect_q=None, hands=("l", "r"), send_command_fn=None, reader=None,
                          live_controls=None):
    """
    reader: a port_read.Reader, OR any object that wraps one (e.g. a
    ChoirBaseApp/ChoirMovingWindow instance -- anything exposing .reader,
    .viz_q or .dataq, .hands, .effect_q, .live_controls). When supplied,
    dataq/hands/effect_q/send_command_fn/live_controls are pulled from it
    automatically wherever the caller didn't already supply that argument
    explicitly.

    This exists because calling build_app_and_window(app.viz_q,
    effect_q=app.effect_q, hands=app.hands) WITHOUT also passing
    send_command_fn=app.reader.send_command silently leaves R/P
    "(unavailable, no command channel)" -- it just defaults to None, no
    error. Passing reader=app (or reader=a bare Reader) here removes that
    foot-gun: send_command_fn is filled in from whichever of
    reader.send_command / reader.reader.send_command exists.
    """
    if reader is not None:
        # Accept either a bare Reader (has .send_command directly) or a
        # wrapper object that holds one as .reader (e.g. ChoirBaseApp).
        send_command_source = reader if hasattr(reader, "send_command") else getattr(reader, "reader", None)
        if dataq is None:
            if hasattr(reader, "viz_q"):
                dataq = reader.viz_q
            elif hasattr(reader, "dataq"):
                dataq = reader.dataq
            elif hasattr(reader, "threads"):
                dataq = {hand: reader.threads[hand]["parser"].getQ() for hand in reader.hands}
        if hands == ("l", "r") and hasattr(reader, "hands"):
            hands = reader.hands
        if effect_q is None and hasattr(reader, "effect_q"):
            effect_q = reader.effect_q
        if send_command_fn is None and send_command_source is not None:
            send_command_fn = send_command_source.send_command
        if live_controls is None and hasattr(reader, "live_controls"):
            live_controls = reader.live_controls
    if dataq is None:
        raise ValueError("build_app_and_window needs either dataq= or reader=")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = TwinWindow(dataq, effect_q=effect_q, hands=hands, send_command_fn=send_command_fn,
                         live_controls=live_controls)
    window.resize(1400, 900)
    window.show()
    return app, window


if __name__ == "__main__":

    reader = Reader(
        sensor_config={
            "l": {"flex": True, "press": True, "imu": True},
            "r": {"flex": True, "press": True, "imu": True},
        },
        use_ble=True,
    )
    reader.start_readers()
    for hand in reader.hands:
        reader.send_command(hand, "MOTION_STREAM_ON")

    app, window = build_app_and_window(reader=reader)
    app.aboutToQuit.connect(reader.stop_readers)
    sys.exit(app.exec())