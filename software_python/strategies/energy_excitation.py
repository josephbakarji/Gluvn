"""
energy_excitation.py — Tier 1, module A.

Replaces Jacob's instantaneous-accel-burst -> discrete note-trigger with a
continuous, physically-motivated excitation model: hand velocity charges a
LeakyEnergyTank (per hand), and the tank's state drives THREE simultaneous,
independently-textured outputs rather than a single volume/velocity scalar.
This is the "creative" expansion requested — one physical quantity (kinetic
energy) fans out into distinct musical roles instead of a 1:1 CC mapping.

Physical rationale: kinetic energy E = 1/2 m|v|^2. Mass is unknown/unneeded
(it's a fixed unobservable scale factor absorbed into gain), but using |v|^2
rather than |v| means fast brief motions and slow sustained motions of equal
work-done charge the tank comparably -- closer to "how hard was this struck"
than a raw speed readout.

Outputs (all continuous, [0,1] normalized unless noted):
  1. excitation   — tank level -> amplitude envelope of a sustained tone.
                    Behaves like a struck resonant object: fast attack on
                    motion, exponential decay on stillness (tau_decay_s),
                    NOT an instant on/off gate.
  2. brightness   — tank level also opens a low-pass filter cutoff. Physical
                    motivation: real struck/bowed objects get spectrally
                    brighter under harder excitation (more high-frequency
                    content in the impulse) -- same source signal, different
                    time-constant (faster attack, no decay lag) so brightness
                    "leads" amplitude very slightly, mimicking the transient-
                    then-settle behavior of a real strike.
  3. grain_density — a SEPARATE, faster leaky tank (short tau) tracks jerk
                    (rate of change of acceleration proxy = d|v|/dt) to
                    drive a granular-density-style parameter: rapid
                    direction changes / tremor -> dense, textured output;
                    smooth sustained motion -> sparse. This captures gesture
                    *quality* (smooth vs. jittery) independently of gesture
                    *magnitude*, which the amplitude/brightness pair alone
                    cannot distinguish.

All three are derived from the SAME validated velocity stream and share the
gate check (tank must be above an audibility floor) so a stationary hand is
silent on all three outputs, not partially "leaking" sound.
"""

import numpy as np
from software_python.strategies.kinematics import LeakyEnergyTank, OnePoleSmoother, HysteresisGate


class EnergyExcitationMapper:
    def __init__(self,
                 tau_decay_s=1.2,          # amplitude tank decay -- tune per "instrument size" feel
                 tau_decay_fast_s=0.15,    # jerk tank decay -- short, tracks texture not sustain
                 brightness_tau_s=0.05,    # near-instant: brightness leads amplitude
                 energy_gain=1.0,
                 max_energy=6.0,           # calibrate against your hardware's observed |v| range
                 audibility_floor=0.02,
                 jerk_gain=8.0,
                 max_jerk_energy=1.0):
        self.amp_tank = LeakyEnergyTank(tau_decay_s=tau_decay_s, gain=energy_gain, max_energy=max_energy)
        self.jerk_tank = LeakyEnergyTank(tau_decay_s=tau_decay_fast_s, gain=jerk_gain, max_energy=max_jerk_energy)
        self.brightness_smoother = OnePoleSmoother(tau_s=brightness_tau_s)
        self.gate = HysteresisGate(on_thresh=audibility_floor, off_thresh=audibility_floor * 0.5)
        self._prev_speed = None

    def step(self, velocity_mps, dt):
        """
        velocity_mps : np.ndarray shape (3,), NavEKF velocity estimate.
        dt            : seconds since last call (from BLE packet timestamps,
                        not assumed-constant -- BLE jitter means this must
                        not be hardcoded).

        Returns dict: excitation, brightness, grain_density (all [0,1] post-
        gate), sounding (bool), raw_speed (m/s, for logging/debug).
        """
        speed = float(np.linalg.norm(velocity_mps))
        kinetic_power_proxy = speed ** 2  # instantaneous "power-like" term; tank integrates over dt

        amp_energy = self.amp_tank.step(kinetic_power_proxy, dt)
        amp_norm = self.amp_tank.normalized()

        if self._prev_speed is None:
            jerk = 0.0
        else:
            jerk = abs(speed - self._prev_speed) / max(dt, 1e-6)
        self._prev_speed = speed

        jerk_energy = self.jerk_tank.step(jerk, dt)
        jerk_norm = self.jerk_tank.normalized()

        brightness_raw = amp_norm  # same source, different smoothing time-constant (see module docstring)
        brightness = self.brightness_smoother.step(brightness_raw, dt)

        sounding = self.gate.step(amp_norm)

        return {
            'sounding': sounding,
            'excitation': amp_norm if sounding else 0.0,
            'brightness': float(np.clip(brightness, 0.0, 1.0)) if sounding else 0.0,
            'grain_density': jerk_norm if sounding else 0.0,
            'raw_speed_mps': speed,
        }
