"""
tilt_surface.py — Tier 1, module B.

Uses gravity_tilt_decomposition (tilt_angle, tilt_azimuth) instead of raw
roll/pitch as separate channels. Why this matters, not just style:
roll and pitch are an arbitrary Euler split of a single physical quantity
("how far and which way is the device tilted from vertical") -- Jacob's
code has to carry two independently-tuned threshold sets (ROLL_TRIGGER_*,
PITCH_TRIGGER_*) with a documented asymmetry between them precisely because
that split is arbitrary and doesn't respect the underlying rotational
symmetry. Using (tilt_angle, tilt_azimuth) collapses that to one
rotation-invariant magnitude + one circular direction, both continuous.

Drift note: tilt_azimuth is read fresh each frame from nav_quat (gravity-
corrected), NEVER integrated/accumulated, so it does not inherit yaw's
drift problem despite being "yaw-like" in flavor -- it's bounded in
(-pi, pi] at all times, consistent with the roll/pitch-are-safe,
yaw-is-not caveat from the pipeline notes.

Scope note: this module is intentionally music-theory-free. It emits a
continuous, unquantized *scale-degree position* (a float, not a MIDI
note) -- snapping that to an actual note is NoteMapper's job (see
note_mapper.py's own docstring: "This module is the single musical-theory
authority for the GLUVN system... Strategy classes should delegate all
note/scale/chord logic here rather than reimplementing it"). An earlier
version of this file had its own _nearest_scale_degree() reimplementing
scale snapping -- removed in favor of that delegation.

Outputs (two independent channels from the same 2D tilt surface):

  1. tilt_angle  -> continuous "openness" parameter (e.g. filter resonance,
                    or bow-force analog): flat wrist = 0 (closed/damped),
                    fully tilted = 1 (open/resonant). Slew-limited so
                    estimator-transient jitter doesn't click the filter.

  2. tilt_azimuth -> continuous scale-degree position (float): azimuth
                    maps to a full rotation around a configurable number
                    of scale degrees. The integer part selects a degree
                    index (for NoteMapper.notes_in_scale lookup); the
                    fractional part is the sub-degree offset, useful for
                    pitch-bend-toward-target so gesture position between
                    two scale notes is not silently discarded at the
                    quantization step.
"""

import numpy as np
from software_python.strategies.kinematics import gravity_tilt_decomposition, SlewLimiter, OnePoleSmoother


class TiltSurfaceMapper:
    def __init__(self,
                 degrees_per_turn=7,       # scale degrees spanned by one full azimuth rotation
                 openness_slew_per_s=3.0,  # max openness change per second
                 azimuth_smooth_tau_s=0.03):
        self.degrees_per_turn = degrees_per_turn
        self.openness_slew = SlewLimiter(max_rate_per_s=openness_slew_per_s)
        self.azimuth_smoother = OnePoleSmoother(tau_s=azimuth_smooth_tau_s)

    def step(self, nav_quat, dt):
        """
        nav_quat : [w,x,y,z] from NavEKF (gravity-corrected attitude).
        Returns dict with:
          openness        : [0,1], slew-limited tilt magnitude
          azimuth_deg     : (-180,180], smoothed raw azimuth (debug/viz)
          degree_position : float, continuous scale-degree position.
                             int(degree_position) is the degree index
                             (mod degrees_per_turn); the fractional part
                             is the sub-degree offset toward the next
                             degree, for pitch-bend-toward-target use.
        """
        tilt_angle, tilt_azimuth = gravity_tilt_decomposition(nav_quat)

        openness_raw = float(np.clip(tilt_angle / (np.pi / 2), 0.0, 1.0))  # pi/2 = fully sideways
        openness = self.openness_slew.step(openness_raw, dt)

        azimuth_smoothed = self.azimuth_smoother.step(tilt_azimuth, dt)
        azimuth_deg = float(np.degrees(azimuth_smoothed))

        degree_position = (azimuth_smoothed / (2 * np.pi)) * self.degrees_per_turn % self.degrees_per_turn

        return {
            'openness': openness,
            'azimuth_deg': azimuth_deg,
            'degree_position': float(degree_position),
        }
