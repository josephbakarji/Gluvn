"""
app_jacob_choir.py -- three-note choir instrument.

This file contains ONLY musical decisions. Everything about the gloves --
connecting, orientation, calibration, scaling, thresholds, finger latches,
gesture edges, stale-hand warnings -- is core (core.pose_provider delivers it
as MotionFrame/MotionEvent; core.music_app supplies the run loop). To change how
a gesture is *detected*, edit core; to change what it *plays*, edit this file.

Gesture -> sound (finger numbers are 1-based here, 0-based in code):
  accel burst              -> retrigger up to 3 notes from the held flex fingers,
                              and re-center yaw (yaw-sector selects which note a
                              later pitch step moves)
  roll (while press 2)     -> vibrato: roll depth x LFO -> pitch bend (both hands sum)
  pitch tilt (while press 3) -> step the sounding note up/down within the scale
  press 4 held             -> volume delta from pitch rate-of-change (aftertouch)
  press 5                  -> all notes off

Performance recipe -- how the yaw sectors work when playing:
  Strike (accel burst) to re-center yaw: wherever the forearm is aimed at that
  instant becomes CENTER (slot 1), and a small window (default +/-28.3 degrees,
  MotionConfig.yaw_window) is drawn around it. Turning the forearm further
  RIGHT than that window selects slot 0; further LEFT selects slot 2. With
  press 3 held, tilting the forearm up or down steps the note assigned to
  WHICHEVER slot you are currently pointed at (up = +1 in the scale, down =
  -1). Because yaw drifts without a magnetometer, re-strike whenever the
  sectors feel off-center -- that is the only way to re-anchor them.

Performer recipe: hold neutral during the 2 s calibration; flex the desired
notes (right hand fills slots first, then left); strike a burst while aiming at
the desired CENTER; hold press 3 and tilt at least 14.2 degrees to fire, then
return inside 7.1 degrees to re-arm. CENTER is within +/-28.3 degrees of the
strike heading, further RIGHT selects slot 0, and further LEFT selects slot 2.
Yaw drifts without a magnetometer, so re-strike to re-center when needed.
"""

import sys
import time

import numpy as np

from core.music_app import MusicApp, midi_note_name, scan_available_ble
from core.pose_provider import MotionConfig


class ChoirMovingWindow(MusicApp):
    # Reference-pose calibration window (seconds); handed to MotionConfig.
    CALIBRATION_WINDOW_SEC = 2.0

    # Vibrato lane. pitch_bend has no per-hand channel (midi_writer hardcodes
    # MIDI channel 1), so both hands SUM into one depth and one LFO.
    VIBRATO_MAX_PITCH_BEND = 2000   # per-hand cap; two hands sum to 4000 of +/-8192
    VIBRATO_LFO_HZ = 4.5            # slow, wide, operatic feel

    MAX_SOUNDING_NOTES = 3

    def __init__(self, *args, volume_controller=None, pitch_bender=None,
                 num_lh_fingers=5, num_rh_fingers=5, avg_window_size=10,
                 base_volume=20, use_yaw=True,
                 accel_trigger_thresh=110, accel_trigger_hysteresis=10,
                 accel_norm_max=15.0,
                 roll_trigger_thresh_range=20, roll_trigger_hysteresis=5,
                 pitch_trigger_thresh_range=5, pitch_trigger_hysteresis=5,
                 yaw_window=10, vibrato_enabled=True,
                 pitch_step_mode='directional',
                 motion_config=None, **kwargs):
        # The motion parameters below are core's (MotionConfig). They stay
        # accepted here so existing launch code keeps working; pass your own
        # ``motion_config`` to override them wholesale.
        if motion_config is None:
            motion_config = MotionConfig(
                accel_norm_max=accel_norm_max, avg_window_size=avg_window_size,
                accel_trigger_thresh=accel_trigger_thresh,
                accel_trigger_hysteresis=accel_trigger_hysteresis,
                roll_trigger_thresh_range=roll_trigger_thresh_range,
                roll_trigger_hysteresis=roll_trigger_hysteresis,
                pitch_trigger_thresh_range=pitch_trigger_thresh_range,
                pitch_trigger_hysteresis=pitch_trigger_hysteresis,
                yaw_window=yaw_window, pitch_step_mode=pitch_step_mode,
                vibrato_enabled=vibrato_enabled,
                calibration_window_sec=self.CALIBRATION_WINDOW_SEC)
        super().__init__(*args, motion_config=motion_config, **kwargs)
        # volume_controller / pitch_bender / num_lh_fingers belonged to the old
        # raw-sensor routing that nothing calls any more; accepted, ignored.
        self.num_rh_fingers = num_rh_fingers
        self.playing_notes = [None] * num_rh_fingers
        self.base_volume = base_volume
        self.global_volume = base_volume
        self.use_yaw = use_yaw          # if yaw drift is high, disable this
        self.notemaps = None

    # -- hooks -------------------------------------------------------------

    def on_start(self):
        self.notemaps = self.mapper.basic_map_2hands(hands=self.hands)

    def on_frame(self, frame, motion):
        self._vibrato()
        if motion.fingers.press[3]:
            self._volume_delta(motion)

    def on_event(self, event, motion):
        if event.kind == 'accel_burst':
            if event.value == 1:
                self._burst(event)
        elif event.kind == 'press':
            if event.index == 4 and event.value == 1:
                self._all_notes_off(event)
        elif event.kind == 'pitch_step':
            self._pitch_step(event)

    # -- musical actions ---------------------------------------------------

    def _sounding_names(self):
        return [midi_note_name(n) for n in self.playing_notes if n is not None]

    def _slot_names(self):
        # One entry per slot, None for an empty one, so the twin can show all three
        # sectors positionally (slot i is the note the arm selects in sector i).
        return [midi_note_name(n) if n is not None else None for n in self.playing_notes]

    def _vibrato(self):
        """Roll depth -> pitch bend, continuous. Each hand contributes only
        while its own press 2 is held, so an incidental roll (resting the arm,
        adjusting the glove) adds nothing. Uses each hand's newest known depth:
        one hand's frame arrives at a time, which is fine for a wobble."""
        if self.live_controls.get_vibrato_enabled():
            total = 0.0
            for h in self.hands:
                mh = self.pose.latest_motion(h)
                if mh is not None and mh.fingers.press[1]:
                    total += mh.roll_depth
            if total > 0.0:
                lfo = np.sin(2 * np.pi * self.VIBRATO_LFO_HZ * time.monotonic())
                bend = lfo * total * self.VIBRATO_MAX_PITCH_BEND
            else:
                bend = 0.0
            self.midi_writer.pitch_bend(bend)
        elif self.live_controls.pop_vibrato_reset_pending():
            self.midi_writer.pitch_bend(0)

    def _volume_delta(self, motion):
        self.global_volume = min(max(self.global_volume + int(motion.pitch_delta), 0), 127)
        self.midi_writer.aftertouch(self.global_volume)
        self.emit(motion.hand,
                  f'VOLUME DELTA (pitch tilt) [{motion.hand}]: {self._sounding_names()}')

    def _burst(self, ev):
        print('Triggering notes...')
        self.pose.recenter_yaw()
        self.playing_notes = self.midi_writer.turn_off_all_playing(self.playing_notes)
        limit = min(self.MAX_SOUNDING_NOTES, len(self.playing_notes))
        count = 0
        for hnd in self.hands[::-1]:
            notes = self.notemaps[hnd]
            for i, held in enumerate(self.pose.fingers(hnd).flex):
                if held and i < len(notes):
                    self.midi_writer.trig_note(notes[i], vel=self.global_volume)
                    self.playing_notes[count] = notes[i]
                    count += 1
                if count >= limit:
                    break
            if count >= limit:
                break
        # Snapshot AFTER the notes are placed, so this reports what just sounded.
        self.emit(ev.hand,
              f'NOTE TRIGGER (accel burst) [{ev.hand}]: {self._sounding_names()}',
              self._slot_names(), use_yaw=self.use_yaw)

    def _all_notes_off(self, ev):
        self.playing_notes = self.midi_writer.turn_off_all_playing(self.playing_notes)
        self.emit(ev.hand, f'ALL NOTES OFF [{ev.hand}]', self._slot_names(), use_yaw=self.use_yaw)

    def _pitch_step(self, ev):
        """Step one sounding note up/down the scale. The yaw sector the arm was
        in when it tilted picks WHICH note; a held press 3 arms the gesture."""
        note_idx = ev.yaw_sector if self.use_yaw else 0   # yaw off: first note
        if note_idx < 0:
            # Right of the yaw window. Legacy indexed playing_notes[-1] here -- the
            # LAST slot -- which is what lets three sectors (left / centre / right)
            # address three notes. A `0 <= idx` guard here silently loses the third.
            note_idx = len(self.playing_notes) - 1
        if not (0 <= note_idx < len(self.playing_notes)
                and self.pose.fingers(ev.hand).press[2]
                and self.playing_notes[note_idx] is not None):
            return
        scale = self.mapper.notes_in_scale
        old = self.playing_notes[note_idx]
        new_idx = scale.index(old) + ev.value
        if not 0 <= new_idx < len(scale):
            return                                  # already at the top/bottom of the scale
        new_note = scale[new_idx]
        self.midi_writer.trig_note(old, 0)
        self.midi_writer.trig_note(new_note, vel=self.global_volume)
        self.playing_notes[note_idx] = new_note
        self.emit(ev.hand,
                  f'NOTE STEP {ev.value:+d} (pitch tilt) [{ev.hand}, sector {note_idx}]: '
                  f'{midi_note_name(new_note)} -- all: {self._sounding_names()}',
              self._slot_names(), use_yaw=self.use_yaw)


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
    ACCEL_NORM_MAX = 15.0   # m/s^2 (~1.5g). LOWER this first if bursts are hard to trigger.
    ROLL_TRIGGER_THRESH_RANGE = 20
    ROLL_TRIGGER_HYSTERESIS = 5
    # Pitch maps +/-90 deg onto 127 units = 1.417 deg/unit. Note-step FIRES at
    # (range + hyst) = 10 units = 14.2 deg from rest -- legacy fired at 14.3 deg --
    # and RE-ARMS at `range` = 5 units = 7.1 deg.
    PITCH_TRIGGER_THRESH_RANGE = 5
    PITCH_TRIGGER_HYSTERESIS = 5
    # Yaw maps +/-180 deg onto 127 units = 2.835 deg/unit: 10 units = +/-28.3 deg.
    YAW_WINDOW = 10
    VIBRATO_ENABLED = True

    app = ChoirMovingWindow(
        root_note=root_note, scale=scale,
        num_rh_fingers=num_rh_fingers,
        avg_window_size=10,
        send_all_data=True,
        trigger_sensors=trigger_sensors,
        thresholds=thresholds,
        hysteresis=hysteresis,
        hands=sorted(active_hands),   # deterministic: fill order of trig notes depends on it
        use_ble=True,
        use_yaw=True,
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
        # diagnostics=True,   # per-hand health print every 2 s (core/hand_diag.py)
    )
    app.start()

    from ui.digital_twin import build_app_and_window   # Qt only when the twin is wanted
    qt_app, twin_window = build_app_and_window(reader=app)

    def _shutdown():
        print('Shutting down...')
        app.shutdown()

    qt_app.aboutToQuit.connect(_shutdown)
    sys.exit(qt_app.exec())
