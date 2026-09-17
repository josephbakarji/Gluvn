"""
app_continuous_gesture_instrument.py (formerly app_tier1.py) — continuous,
physically-informed gesture mapping across both gloves, bridged to Ableton
Live via MIDI (loopMIDI + core/midi_writer.py), matching the same
DAW-integration AND hardware-acquisition pattern as the existing choir
instrument (app_jacob_choir.py), including the live digital-twin viewer.

CONTINUOUS -> MIDI TRANSLATION: Tier 1's mappings are continuous,
always-on control surfaces. MIDI has no equivalent of "a parameter that
is just always present" -- everything rides on a sounding note, and
pitch resolution below a semitone requires pitch bend on that note, not
a separate message. This file is where that translation happens:

  Module A (energy excitation) -> note lifecycle + CC:
    - `excitation` crossing the audibility gate (rising) = note-on;
      falling = note-off, via EnergyExcitationMapper's own
      HysteresisGate output (`sounding`) -- no separate threshold logic
      reintroduced here.
    - `excitation` level (while sounding) -> aftertouch, exactly like
      Jacob's choir app uses aftertouch for volume from pitch-rate.
    - `brightness`, `grain_density` -> CC (filter cutoff / CC74 by
      convention, and a free CC for texture respectively).

  Module B (tilt surface) -> note selection + pitch bend:
    - `degree_position`'s integer part selects a scale degree from
      NoteMapper.notes_in_scale (delegated, not reimplemented).
    - `degree_position`'s fractional part (sub-degree offset) drives
      pitch_bend(), giving continuous glide between scale notes.
    - `openness` -> CC (resonance/bow-force analog).
    - Changing scale degree while a note is sounding retriggers the note
      (note-off old pitch, note-on new pitch) rather than pitch-bending
      across a full scale step, since scale steps are frequently >200
      cents -- outside typical pitch-bend range.

  Module C (dual-hand coupling) -> CC only:
    - `harmonic_distance`, `spatial_azimuth` -> CC. Neither is
      note-based, so no note-lifecycle translation is needed here.

MIDI CC ASSIGNMENTS (channel 1 for left hand, channel 2 for right hand --
avoids CC collisions between hands without needing distinct CC numbers):
  CC 74  brightness      (filter cutoff -- General MIDI convention)
  CC 20  grain_density   (undefined/free CC)
  CC 21  openness
  CC 22  harmonic_distance  (sent on ch. 1 only -- see note in step_once)
  CC 23  spatial_azimuth    (sent on ch. 1 only -- see note in step_once)
  Aftertouch (channel pressure): excitation level while a note sounds.
  Pitch bend: sub-degree glide, +-degrees_per_turn scale steps mapped to
  +-8192 (matches MidiWriter.pitch_bend's 14-bit range and Jacob's own
  pitch_bend_limit = 8192 convention).
"""

import sys
import time
import argparse
import queue
from threading import Thread

from strategies.energy_excitation import EnergyExcitationMapper
from strategies.tilt_surface import TiltSurfaceMapper
from strategies.dual_hand_coupling import DualHandCouplingMapper
from core.note_mapper import NoteMapper
from core.port_read import Reader
from ui.digital_twin import build_app_and_window


DEVICE_US_WRAP = 2 ** 32  # presumed firmware counter width; harmless if it never wraps in practice
PITCH_BEND_LIMIT = 8192   # matches MidiWriter.pitch_bend's 14-bit range and Jacob's choir app convention

CC_BRIGHTNESS = 74
CC_GRAIN_DENSITY = 20
CC_OPENNESS = 21
CC_HARMONIC_DISTANCE = 22
CC_SPATIAL_AZIMUTH = 23

HAND_CHANNEL = {'l': 1, 'r': 2}  # matches MIDI channel convention (1-indexed, per midi_writer.py)


class PoseSource:
    """Acquires per-hand pose (orientation quaternion + velocity) via
    core.port_read.Reader -- the same acquisition path app_jacob_choir.py
    uses -- rather than a separate, unconfirmed PoseProvider path.

    Reader streams raw sensor frames per hand through
    `reader.threads[hand]['parser'].getQ()`; NavEKF-derived orientation/
    velocity/device_us are decoded from those frames the same way
    app_jacob_choir.py's MultiSensorProcess consumes them. This class
    owns that decode + per-hand device_us -> dt conversion so mapping
    code downstream keeps seeing the same (quat, velocity, dt) contract
    as before -- only the acquisition path underneath changed.
    """

    def __init__(self, sensor_config, hands=('l', 'r'), use_ble=True):
        self.hands = list(hands)
        self.reader = Reader(sensor_config=sensor_config, use_ble=use_ble)
        self._prev_device_us = {h: None for h in self.hands}
        self._live = False  # flips true once start() confirms real hardware threads

    def start(self):
        self.reader.start_readers()
        # Matches ChoirBaseApp._enable_nav_quat_streaming: request NavEKF's
        # corrected quaternion/velocity stream explicitly rather than
        # assuming it's on by default.
        for hand in self.hands:
            if hand in self.reader.threads:
                try:
                    self.reader.send_command(hand, "NAV_QUAT_STREAM_ON")
                except Exception as e:
                    print(f"[PoseSource] Warning: could not enable NAV_QUAT_STREAM_ON on {hand}: {e}")
        self._live = any(h in self.reader.threads for h in self.hands)
        if not self._live:
            print("[PoseSource] No hand threads connected -- falling back to "
                  "synthetic pose generator for offline testing.")
            self._t0 = time.monotonic()

    def close(self):
        self.reader.stop_readers()

    def _device_dt(self, hand, device_us):
        """Seconds since this hand's previous sample, from the firmware
        clock. Returns None on the first sample for a hand (no prior
        reference) so callers can supply their own bootstrap dt."""
        prev = self._prev_device_us[hand]
        self._prev_device_us[hand] = device_us
        if prev is None:
            return None
        delta_us = (device_us - prev) % DEVICE_US_WRAP
        return delta_us / 1e6

    def get(self, hand):
        """Returns (quat, velocity, dt_seconds) or None if this hand has
        no fresh sample yet (e.g. still in the stream re-arm window)."""
        if self._live and hand in self.reader.threads:
            frame = self._latest_frame(hand)
            if frame is None:
                return None
            dt = self._device_dt(hand, frame['device_us'])
            if dt is None or dt <= 0:
                dt = 1.0 / 60.0  # first-sample bootstrap / guard against stale duplicate device_us
            return frame['nav_quat'], frame['nav_velocity'], dt
        return self._synthetic(hand)

    def _latest_frame(self, hand):
        """Drains this hand's parser queue to the most recent NavEKF
        frame, discarding stale intermediate samples the way
        MultiSensorProcess's own queue.get(block=True) loop effectively
        does one-at-a-time -- here we just fast-forward to the newest."""
        q = self.reader.threads[hand]['parser'].getQ()
        latest = None
        try:
            while True:
                latest = q.get_nowait()
        except queue.Empty:
            pass
        return latest  # dict with 'nav_quat', 'nav_velocity', 'device_us', or None if nothing arrived

    def _synthetic(self, hand):
        """Deterministic offline stand-in so mapping code is runnable/
        testable without hardware attached. NOT a hardware simulator --
        just enough motion to exercise every code path (tilt, twist,
        velocity bursts) for development. Returns the same (quat,
        velocity, dt) contract as the live path."""
        import numpy as np
        now = time.monotonic()
        t = now - self._t0
        phase = t if hand == 'r' else t * 0.7 + 1.5
        tilt = 0.6 * np.sin(phase * 0.8)
        yaw_ish = phase * 0.3
        w = np.cos(tilt / 2)
        x = np.sin(tilt / 2) * np.cos(yaw_ish)
        y = np.sin(tilt / 2) * np.sin(yaw_ish)
        z = 0.0
        quat = np.array([w, x, y, z])
        quat /= np.linalg.norm(quat)
        speed = 0.5 + 0.5 * np.sin(phase * 3.0)
        velocity = np.array([speed, 0.0, 0.0])

        prev = self._prev_device_us[hand]  # reused as a wall-clock stand-in in synthetic mode
        self._prev_device_us[hand] = now
        dt = (now - prev) if prev is not None else (1.0 / 60.0)
        return quat, velocity, max(dt, 1e-4)


class MidiSink:
    """Wraps core.midi_writer.MidiWriter (winmm.dll + loopMIDI), matching
    app_jacob_choir.py's usage exactly. Degrades to console print when
    MidiWriter can't initialize (non-Windows dev machine, loopMIDI not
    running) so this file stays runnable off-target for development."""

    def __init__(self):
        try:
            from core.midi_writer import MidiWriter
            self._writer = MidiWriter()
            self._live = True
        except Exception as e:
            print(f"[MidiSink] Live MidiWriter unavailable ({e}); "
                  f"printing MIDI events to console instead.")
            self._writer = None
            self._live = False

    def note_on(self, note, velocity):
        if self._live:
            self._writer.trig_note(note, velocity)
        else:
            print(f"note_on   note={note} vel={velocity}")

    def note_off(self, note):
        if self._live:
            self._writer.trig_note(note, 0)
        else:
            print(f"note_off  note={note}")

    def aftertouch(self, value, channel=1):
        if self._live:
            self._writer.aftertouch(value, channel=channel)
        else:
            print(f"aftertouch ch={channel} val={value}")

    def control_change(self, control, value, channel=1):
        if self._live:
            self._writer.control_change(value, control, channel=channel)
        else:
            print(f"cc{control}      ch={channel} val={value}")

    def pitch_bend(self, offset, channel=1):
        # NOTE: MidiWriter.pitch_bend() as supplied does not accept a
        # channel argument (hardcoded to 0xE0 = channel 1) -- flagged
        # here rather than silently sending both hands' bend to the same
        # channel. See ContinuousGestureApp.step_once for how this is
        # handled given that constraint.
        if self._live:
            self._writer.pitch_bend(offset)
        else:
            print(f"pitchbend ch={channel} offset={offset}")

    def all_notes_off(self, playing_notes):
        if self._live:
            return self._writer.turn_off_all_playing(playing_notes)
        print("all_notes_off")
        return [None] * len(playing_notes)


class ContinuousGestureApp(Thread):
    """Continuous-mapping counterpart to app_jacob_choir.py's
    ChoirBaseApp/ChoirMovingWindow. Subclasses Thread (matching
    ChoirBaseApp, not a plain object) specifically so the digital twin
    can run on the main thread's Qt event loop while this drives
    Reader + MIDI on its own thread -- the same split app_jacob_choir.py
    uses.
    """

    def __init__(self, hands=('l', 'r'),
                 root_note='D', scale='minor',    # delegated to NoteMapper -- see note_mapper.py
                 degrees_per_turn=7, use_ble=True, rate_hz=60.0):
        super().__init__()
        self.daemon = True
        self.hands = list(hands)
        self.rate_hz = rate_hz

        # send_all_data mirrors app_jacob_choir.py's own default for the
        # working choir app -- imu streaming is required by every mapper
        # here (energy needs velocity, tilt needs orientation), so there
        # is no narrower sensor_config worth hand-picking.
        sensor_config = {h: {'flex': False, 'press': False, 'imu': True} for h in self.hands}
        self.pose = PoseSource(sensor_config, hands=self.hands, use_ble=use_ble)

        # viz_q / effect_q: same shape and purpose as ChoirBaseApp's --
        # build_app_and_window reads from these to drive the twin
        # display. Populated in _update_hand below.
        self.viz_q = {h: queue.Queue(maxsize=5) for h in self.hands}
        self.effect_q = queue.Queue(maxsize=5)

        self.midi = MidiSink()

        # NoteMapper is the single musical-theory authority (its own
        # docstring's words) -- this app does not maintain its own scale
        # logic anywhere, matching that module's stated design contract.
        self.note_mapper = NoteMapper(root_note=root_note, scale=scale)
        # Per-hand octave window into notes_in_scale, echoing
        # basic_map_2hands' convention (right hand higher than left).
        base_idx = self.note_mapper._find_root_idx(octave_hint=3)
        self._degree_base_idx = {
            'r': base_idx,
            'l': max(0, base_idx - degrees_per_turn),
        }
        self.degrees_per_turn = degrees_per_turn

        self.energy_mappers = {h: EnergyExcitationMapper() for h in self.hands}
        self.tilt_mappers = {h: TiltSurfaceMapper(degrees_per_turn=degrees_per_turn)
                              for h in self.hands}
        self.dual_mapper = DualHandCouplingMapper() if len(self.hands) == 2 else None

        # Note-lifecycle state, mirroring app_jacob_choir.py's playing_notes
        # pattern: one sounding note (or None) per hand, plus the last
        # committed scale-degree index so a genuine degree change (not
        # just sub-degree glide) triggers a retrigger rather than a bend.
        self._playing_note = {h: None for h in self.hands}
        self._current_degree_idx = {h: None for h in self.hands}
        self._running = False

    def _degree_to_note(self, hand, degree_idx):
        """Look up a MIDI note number for this hand's scale-degree index,
        via NoteMapper.notes_in_scale -- the delegated, single source of
        truth for note/scale logic (see module docstring)."""
        notes = self.note_mapper.notes_in_scale
        idx = self._degree_base_idx[hand] + (degree_idx % self.degrees_per_turn)
        idx = max(0, min(idx, len(notes) - 1))  # clamp rather than wrap past the mapper's own range
        return notes[idx]

    def _push_viz(self, hand, quat, velocity, energy_out, tilt_out):
        """Non-blocking push to this hand's viz queue for the digital
        twin, matching ChoirBaseApp's viz_q usage: drop rather than
        block if the twin is behind, since visualization must never
        stall the control-rate loop."""
        payload = {
            'quat': quat, 'velocity': velocity,
            'excitation': energy_out['excitation'],
            'brightness': energy_out['brightness'],
            'degree_position': tilt_out['degree_position'],
            'openness': tilt_out['openness'],
        }
        try:
            self.viz_q[hand].put_nowait(payload)
        except queue.Full:
            pass

    def _update_hand(self, hand, quat, velocity, dt):
        energy_out = self.energy_mappers[hand].step(velocity, dt)
        tilt_out = self.tilt_mappers[hand].step(quat, dt)
        channel = HAND_CHANNEL[hand]

        degree_position = tilt_out['degree_position']
        degree_idx = int(degree_position) % self.degrees_per_turn
        sub_degree_frac = degree_position - int(degree_position)  # [0,1) offset toward next degree

        sounding = energy_out['sounding']
        was_sounding = self._playing_note[hand] is not None
        degree_changed = (self._current_degree_idx[hand] is not None
                           and self._current_degree_idx[hand] != degree_idx)

        # --- Note lifecycle -------------------------------------------------
        if sounding and (not was_sounding or degree_changed):
            if was_sounding:
                self.midi.note_off(self._playing_note[hand])
            note = self._degree_to_note(hand, degree_idx)
            # Velocity from excitation level, clamped to [1,127]: 0 would be
            # a note-off per the MIDI spec, and trig_note(note, 0) is
            # already how this codebase deliberately spells note-off
            # (see MidiWriter.trig_note / turn_off_all_playing), so a
            # "quiet" note-on must never round down to exactly 0.
            velocity = max(1, min(127, int(round(energy_out['excitation'] * 127))))
            self.midi.note_on(note, velocity)
            self._playing_note[hand] = note
            self._current_degree_idx[hand] = degree_idx
        elif not sounding and was_sounding:
            self.midi.note_off(self._playing_note[hand])
            self._playing_note[hand] = None
            self._current_degree_idx[hand] = None

        # --- Continuous parameters while a note is sounding -----------------
        if self._playing_note[hand] is not None:
            aftertouch_val = int(round(energy_out['excitation'] * 127))
            self.midi.aftertouch(aftertouch_val, channel=channel)

            self.midi.control_change(CC_BRIGHTNESS, int(round(energy_out['brightness'] * 127)), channel=channel)
            self.midi.control_change(CC_GRAIN_DENSITY, int(round(energy_out['grain_density'] * 127)), channel=channel)
            self.midi.control_change(CC_OPENNESS, int(round(tilt_out['openness'] * 127)), channel=channel)

            # Sub-degree pitch-bend glide: channel 1 (left hand) ONLY.
            # midi_writer.py's pitch_bend() has no channel parameter (it
            # hardcodes 0xE0 = MIDI channel 1), so sending it for the
            # right hand would silently bend the LEFT hand's note instead
            # of its own -- right hand hard-snaps to scale degrees with
            # no glide rather than risk that collision.
            if hand == 'l':
                bend_offset = int(round((sub_degree_frac * 2 - 1) * PITCH_BEND_LIMIT * 0.15))
                # scaled to a modest fraction of full range -- a full
                # +-8192 bend is more than one scale step in most tunings;
                # this is deliberately a subtle "lean toward next degree"
                # effect, not a full-step bend (see module docstring).
                self.midi.pitch_bend(bend_offset)

        self._push_viz(hand, quat, velocity, energy_out, tilt_out)
        return energy_out, tilt_out

    def step_once(self):
        """Pulls one sample per hand from PoseSource, which already
        supplies dt derived from the firmware's device_us clock (live
        mode) or wall-clock (synthetic fallback) -- see PoseSource.get.
        A hand with no sample yet (e.g. mid stream-re-arm) is skipped
        for this tick rather than fed a stale/zero value."""
        samples = {}  # hand -> (quat, velocity, dt)
        for h in self.hands:
            result = self.pose.get(h)
            if result is None:
                continue
            quat, velocity, dt = result
            samples[h] = (quat, velocity, dt)
            self._update_hand(h, quat, velocity, dt)

        if self.dual_mapper is not None and 'l' in samples and 'r' in samples:
            q_l, _, dt_l = samples['l']
            q_r, _, dt_r = samples['r']
            # dual mapper's smoothing taus (tens of ms) are short relative
            # to typical BLE-rate dt, so using either hand's dt (rather
            # than tracking a third independent clock for the pair) is a
            # fine approximation -- take the larger of the two as the more
            # conservative (slower-responding-if-wrong) choice.
            dual_out = self.dual_mapper.step(q_l, q_r, dt=max(dt_l, dt_r))
            # Neither dual-hand parameter is note-based, so both go out as
            # CC with no lifecycle concerns. Sent on channel 1 (left hand's
            # channel) by convention, since they describe the pair rather
            # than either hand individually and a channel must be chosen.
            self.midi.control_change(CC_HARMONIC_DISTANCE,
                                       int(round(dual_out['harmonic_distance'] * 127)), channel=1)
            azimuth_norm = (dual_out['spatial_azimuth_deg'] + 180.0) / 360.0  # -> [0,1] for CC range
            self.midi.control_change(CC_SPATIAL_AZIMUTH,
                                       int(round(max(0.0, min(1.0, azimuth_norm)) * 127)), channel=1)

    def run(self, settle_s=0.4):
        """Thread entry point -- matches ChoirBaseApp.run()'s role:
        starts hardware acquisition and loops at rate_hz, leaving the
        Qt/digital-twin event loop free to run on the main thread."""
        self.pose.start()
        if self.pose._live:
            # NavEKF/stream re-arm window (~0.3s per the pipeline notes)
            # after enabling streams -- give it a beat before the first
            # real tick so early samples aren't mid-settle.
            time.sleep(settle_s)
        period = 1.0 / self.rate_hz
        self._running = True
        print(f"Continuous gesture instrument running, sending MIDI at ~{self.rate_hz} Hz.")
        while self._running:
            t_start = time.monotonic()
            self.step_once()
            elapsed = time.monotonic() - t_start
            time.sleep(max(0.0, period - elapsed))

        # Matches app_jacob_choir.py's shutdown pattern: make sure no
        # note is left stuck sounding in Ableton after exit.
        playing = [self._playing_note[h] for h in self.hands]
        self.midi.all_notes_off(playing)
        for h in self.hands:
            self._playing_note[h] = None

    def stop(self):
        self._running = False


def main():
    parser = argparse.ArgumentParser(description="Continuous gesture-to-MIDI glove instrument (Ableton via loopMIDI)")
    parser.add_argument('--hands', default='lr', choices=['l', 'r', 'lr'])
    parser.add_argument('--root-note', default='D', help="Root note for NoteMapper, e.g. 'D' (default matches choir app's tonal center)")
    parser.add_argument('--scale', default='minor', choices=['major', 'minor', 'natural_minor', 'harmonic_minor',
                                                               'melodic_minor_asc', 'melodic_minor_desc', 'pentatonic'])
    parser.add_argument('--degrees-per-turn', type=int, default=7,
                         help="Scale degrees spanned by one full wrist-azimuth rotation")
    parser.add_argument('--rate', type=float, default=60.0, help="MIDI update rate (Hz)")
    args = parser.parse_args()

    hands = tuple(args.hands)
    app = ContinuousGestureApp(hands=hands, root_note=args.root_note, scale=args.scale,
                                degrees_per_turn=args.degrees_per_turn, rate_hz=args.rate)
    app.start()

    # Matches app_jacob_choir.py's __main__ wiring exactly: same three
    # kwargs (viz_q, effect_q, send_command_fn), same shutdown-on-quit
    # pattern via app.reader.stop_readers().
    qt_app, twin_window = build_app_and_window(
        app.viz_q, effect_q=app.effect_q, hands=app.hands,
        send_command_fn=app.pose.reader.send_command,
    )

    def _shutdown():
        print('Shutting down...')
        app.stop()
        app.pose.reader.stop_readers()
        app.join(timeout=0.5)

    qt_app.aboutToQuit.connect(_shutdown)
    sys.exit(qt_app.exec())


if __name__ == "__main__":
    main()