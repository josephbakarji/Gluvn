"""
dual_hand_coupling.py — Tier 1, module C.

Neither hand is treated as independent here; the RELATIVE orientation
between them is the primary signal, decomposed via swing-twist (not naive
per-axis subtraction, which is exactly the kind of thing wrap_around_count
had to retrofit-fix for scalar yaw in Jacob's code -- doing it properly on
quaternions from the start avoids an equivalent bug at the two-hand level).

Physical/gestural framing:
  - SWING (hands' "up" axes diverging, e.g. opening a book / spreading
    arms apart) -> harmonic distance. Hands aligned (swing~0) = unison/
    consonant interval; hands maximally diverged (swing~pi) = maximally
    dissonant interval. This gives a continuous consonance-to-dissonance
    knob controlled by a genuinely bimanual gesture, not replicable with
    one hand -- distinct from anything in Jacob's per-hand-independent
    design.
  - TWIST (rotation of one hand relative to the other ABOUT the shared
    reference axis, e.g. one hand turning like a doorknob relative to the
    other staying still) -> stereo/spatial rotation (pan or ambisonic
    azimuth). Physically this is "the two controllers screwing against
    each other," a distinct DOF from swing, and mapping it to spatial
    position keeps the gesture-to-effect relationship intuitive (twisting
    motion -> rotating sound field).

A third derived output, hand_separation_proxy, is included as an explicit
placeholder for relative POSITION once available -- deliberately NOT wired
to raw NavEKF position by default, per the pipeline's drift-then-snap-back
caveat; see the parameter docstring below before enabling it.
"""

import numpy as np
from software_python.strategies.kinematics import quat_relative, quat_swing_twist, OnePoleSmoother


class DualHandCouplingMapper:
    def __init__(self,
                 twist_axis=(0.0, 0.0, 1.0),
                 swing_smooth_tau_s=0.08,
                 twist_smooth_tau_s=0.05,
                 use_relative_position=False):
        """
        use_relative_position : if True, also computes a normalized hand-
            separation proxy from raw NavEKF position. Leave False by
            default: per the pipeline notes, position shows a transient
            drift-then-snap-back for a few seconds after fast motion
            stops, which would appear as an audible glitch/jump in
            whatever this drives (e.g. a sudden reverb-size jump) rather
            than a smooth gesture response. Safe to enable once paired
            with a windowed/relative dedrift scheme (Tier 2 territory) --
            flagged here rather than silently omitted so the seam is
            visible.
        """
        self.twist_axis = twist_axis
        self.swing_smoother = OnePoleSmoother(tau_s=swing_smooth_tau_s)
        self.twist_smoother = OnePoleSmoother(tau_s=twist_smooth_tau_s)
        self.use_relative_position = use_relative_position

    def step(self, nav_quat_left, nav_quat_right, dt,
              pos_left=None, pos_right=None):
        """
        nav_quat_left/right : [w,x,y,z] NavEKF attitude per hand.
        pos_left/right      : optional (3,) NavEKF position per hand,
                               only consumed if use_relative_position=True.

        Returns dict:
          harmonic_distance   : [0,1], 0=hands aligned (consonant/unison),
                                 1=hands maximally diverged (dissonant)
          spatial_azimuth_deg : (-180,180], relative twist mapped to a
                                 rotation/pan angle
          separation_norm     : [0,1] or None if not enabled/available
        """
        q_rel = quat_relative(nav_quat_left, nav_quat_right)
        swing_angle, twist_angle = quat_swing_twist(q_rel, twist_axis=self.twist_axis)

        swing_smoothed = self.swing_smoother.step(swing_angle, dt)
        twist_smoothed = self.twist_smoother.step(twist_angle, dt)

        harmonic_distance = float(np.clip(swing_smoothed / np.pi, 0.0, 1.0))
        spatial_azimuth_deg = float(np.degrees(twist_smoothed))

        separation_norm = None
        if self.use_relative_position and pos_left is not None and pos_right is not None:
            sep = float(np.linalg.norm(np.asarray(pos_right) - np.asarray(pos_left)))
            separation_norm = float(np.clip(sep / 1.0, 0.0, 1.0))  # 1.0 m calibration placeholder

        return {
            'harmonic_distance': harmonic_distance,
            'spatial_azimuth_deg': spatial_azimuth_deg,
            'separation_norm': separation_norm,
        }
