"""
app_arabic_maqam.py -- two-glove maqam instrument: taqsim melody + darbuka iqa'.

Musical decisions only; detection, calibration and thresholds stay in core.

Ableton setup (two tracks, same MIDI input)
  * Tuning: load a 12-pitch Arabic preset and keep PRESET below in sync with it
    (tuning is set-wide). Default: "Rast 1 - Egypt mid 20th". In these presets
    the keys of a C major scale give Rast on C; E and B are the half-flats.
  * Melody track: any instrument. MIDI From = the loopMIDI port, Ch. 1, Monitor = In.
  * Percussion track: Drum Rack with pads on 36 (dum), 38 (tak), 42 (ka).
    MIDI From = the same port, Ch. 10, Monitor = In. Drum Racks bypass tuning, so
    they MUST sit on a separate track from the melody.
  * Needs core/midi_writer.py with the `channel` argument on trig_note (the
    supplied midi_writer.py). Without it drums fall back to channel 1 and a
    warning is printed.

Melody hand (right glove, 'r'; whichever glove is connected in solo mode)
  flex 1-5            -> the five notes of the current jins (mono, last note wins;
                         releasing a finger falls back to the previous held one,
                         so finger rolls give hammer-ons / pull-offs)
  
  press 1 (held)      -> vibrato, depth from roll
  press 2 + flex k    -> modulate to jins k (layer: no note sounds)
  press 3 (held)+tilt -> octave down / up (applies to the next note)
  press 4 (held)      -> trill (hazz) between the note and the next jins degree
  press 5             -> all notes off
  accel burst         -> re-pluck the sounding note (repeated-note picking)

Rhythm hand (left glove, 'l'; omitted in solo mode)
  accel burst         -> next stroke of the iqa' cycle (your hand is the clock,
                         the pattern decides dum / tak and the accents)
  flex 1 / 2 / 3      -> free dum / tak / ka (fills and ghost notes)
  press 1             -> next iqa'
  press 2             -> resync to the downbeat
  press 5             -> all notes off
"""

import math
import random
import sys
import threading
import time
from pathlib import Path

# Make sibling packages importable when this file is run by path from the
# repository root rather than as a module from inside software_python.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.music_app import MusicApp, scan_available_ble
from core.pose_provider import MotionConfig

# -- tuning data ---------------------------------------------------------------
# Key offsets are positions in the 12-key preset layout, relative to BASE_KEY.
# Note names: d = half-flat (quarter tone), b = flat. 5 notes per jins; 4-note
# ajnas get the next degree as their fifth finger.
KEYS_PER_OCTAVE = 12
BASE_KEY = 60                       # MIDI key that carries the preset's "C"

PRESETS = {
    'rast1': dict(                  # Rast 1 - Egypt mid 20th (also Rast 3 / 4 layout)
        names=['C', 'Db', 'D', 'Eb', 'Ed', 'F', 'F#', 'G', 'Ab', 'A', 'Bb', 'Bd'],
        jins=[('Rast C',       [0, 2, 4, 5, 7]),
              ('Nahawand C',   [0, 2, 3, 5, 7]),
              ('Nikriz C',     [0, 2, 3, 6, 7]),
              ('Upper Rast G', [7, 9, 11, 12, 14]),
              ('Nahawand G',   [7, 9, 10, 12, 14])]),
    'rast2': dict(                  # Rast 2 (Suznak) - Egypt mid 20th
        names=['C', 'Db', 'D', 'Eb', 'Ed', 'F', 'F#', 'G', 'Ab', 'Ad', 'Bb', 'B'],
        jins=[('Rast C',     [0, 2, 4, 5, 7]),
              ('Nahawand C', [0, 2, 3, 5, 7]),
              ('Nikriz C',   [0, 2, 3, 6, 7]),
              ('Hijaz G',    [7, 8, 11, 12, 14]),
              ('Bayati G',   [7, 9, 10, 12, 14])]),
}

# -- percussion data -----------------------------------------------------------
# Stroke -> (MIDI key, base velocity). Keys follow GM kick / snare / closed hat,
# so a default Drum Rack kit is enough to audition the app.
STROKES = {'D': (36, 112), 'T': (38, 96), 'K': (42, 60)}

# Stroke order of one cycle; rests are left to the player's timing.
IQA = {
    'maqsum': ['D', 'T', 'T', 'D', 'T'],    # D T . T D . T .
    'baladi': ['D', 'D', 'T', 'D', 'T'],    # D D . T . D T .
    'saidi':  ['D', 'T', 'D', 'D', 'T'],    # D T . D D . T .
    'malfuf': ['D', 'T', 'T'],              # D . . T . . T .  (3+3+2)
}

# The same cycles as 8 eighth-note slots, for loop mode. D dum, T tak, . rest.
IQA_SLOTS = {
    'maqsum': 'D T . T D . T .',
    'baladi': 'D D T . D T . .',
    'saidi':  'D T . D D . T .',
    'malfuf': 'D . . T . . T .',
}

class ArabicMaqam(MusicApp):
    MELODY_HAND = 'r'                # hand ids as used by core and the twin: 'l' / 'r'
    MELODY_VEL = 92
    # Vibrato: global pitch bend. 700 of +/-8192 is ~17 cents at a +/-2 semitone range.
    VIBRATO_HZ = 5.5
    VIBRATO_MAX_BEND = 700
    TRILL_HZ = 9.0
    DRUM_GATE_SEC = 0.08
    DRUM_CHANNEL = 10                            # MIDI channel for percussion (1-16)
    LOOP_BPM = 76                                # quarter notes per minute in loop mode
    FILL_STROKES = {0: 'D', 1: 'T', 2: 'K'}      # rhythm-hand flex index -> stroke

    def __init__(self, *args, preset='rast1', iqa='maqsum', percussion=True, **kwargs):
        super().__init__(*args, **kwargs)
        self.preset = PRESETS[preset]
        self.iqa_names = list(IQA)
        self.iqa_i = self.iqa_names.index(iqa)
        self.percussion = percussion

    # -- hooks -------------------------------------------------------------

    def on_start(self):
        hands = list(self.hands)
        self.melody_hand = self.MELODY_HAND if self.MELODY_HAND in hands else hands[0]
        rest = [h for h in hands if h != self.melody_hand]
        self.rhythm_hand = rest[0] if (rest and self.percussion) else None
        self.jins_i = 0
        self.octave = 0                  # -1 / 0 / +1
        self.held = []                   # melody fingers down, oldest first
        self.finger_note = {}            # finger -> (key, upper-neighbour key or None)
        self.voice = None                # key currently sounding on the melody line
        self.trill_on = self.trill_up = False
        self.trill_due = 0.0
        self.iqa_pos = 0
        self.prev_flex = {}
        self.offs = []                   # (due time, key) pending drum note-offs
        self.last_bend = None
        self.channel_warned = False
        self.loop_on = False
        self._loop_stop = threading.Event()
        threading.Thread(target=self._loop_run, daemon=True).start()
        print(f'[maqam] melody={self.melody_hand} rhythm={self.rhythm_hand} '
              f'jins={self.preset["jins"][0][0]} iqa={self.iqa_names[self.iqa_i]}')
        if self.percussion and self.rhythm_hand is None:
            print('[maqam] WARNING: only one glove discovered, so the rhythm hand is '
                  'disabled. Check the left glove (power, BLE) and restart.')

    def on_frame(self, frame, motion):
        now = time.monotonic()
        self._flush_offs(now)
        self._vibrato(now)
        if motion is None:
            return
        if motion.hand == self.melody_hand:
            self._melody_fingers(motion.hand)
            self._trill(now)
        elif motion.hand == self.rhythm_hand:
            self._rhythm_fingers(motion.hand)

    def on_event(self, event, motion):
        hand = event.hand
        if event.kind == 'press' and event.value == 1:
            if event.index == 4:
                self._all_notes_off(hand)
            elif hand == self.rhythm_hand and event.index == 0:
                self._next_iqa(hand)
            elif hand == self.rhythm_hand and event.index == 1:
                self.iqa_pos = 0
                self.emit(hand, f'IQA RESYNC [{hand}]')
            elif hand == self.rhythm_hand and event.index == 2:
                self._toggle_loop(hand)
        elif event.kind == 'accel_burst' and event.value == 1:
            if hand == self.melody_hand:
                self._repluck()
            elif hand == self.rhythm_hand:
                self._iqa_step(hand)
        elif event.kind == 'pitch_step' and hand == self.melody_hand:
            self._shift_octave(event)

    # -- helpers -----------------------------------------------------------

    @staticmethod
    def _vel(base, spread):
        return max(1, min(127, base + random.randint(-spread, spread)))

    def _edges(self, hand, states):
        now = [bool(s) for s in states]
        prev = self.prev_flex.get(hand)
        if prev is None or len(prev) != len(now):
            prev = [False] * len(now)
        self.prev_flex[hand] = now
        rises = [i for i, (a, b) in enumerate(zip(prev, now)) if b and not a]
        falls = [i for i, (a, b) in enumerate(zip(prev, now)) if a and not b]
        return rises, falls

    def _flush_offs(self, now):
        due = [k for t, k in self.offs if t <= now]
        if due:
            self.offs = [(t, k) for t, k in self.offs if t > now]
            for key in due:
                self._drum_note(key, 0)

    # -- melody ------------------------------------------------------------

    def _set_voice(self, key):
        """Mono line with legato overlap: new note-on goes out before the old note-off."""
        old = self.voice
        if key == old:
            return
        if key is not None:
            self.midi_writer.trig_note(key, self._vel(self.MELODY_VEL, 8))
        if old is not None:
            self.midi_writer.trig_note(old, 0)
        self.voice = key

    def _melody_fingers(self, hand):
        fs = self.pose.fingers(hand)
        if fs is None:
            return
        rises, falls = self._edges(hand, fs.flex)
        layer = bool(fs.press[1]) #P2 = jins layer
        for i in rises:
            if layer:
                self._set_jins(i)
            else:
                self._finger_down(i)
        for i in falls:
            self._finger_up(i)

    def _finger_down(self, i):
        keys = self.preset['jins'][self.jins_i][1]
        if i >= len(keys):
            return
        origin = BASE_KEY + self.octave * KEYS_PER_OCTAVE
        upper = origin + keys[i + 1] if i + 1 < len(keys) else None
        self.finger_note[i] = (origin + keys[i], upper)
        if i in self.held:
            self.held.remove(i)
        self.held.append(i)
        self._set_voice(origin + keys[i])

    def _finger_up(self, i):
        if i not in self.held:
            return
        self.held.remove(i)
        self._set_voice(self.finger_note[self.held[-1]][0] if self.held else None)

    def _set_jins(self, i):
        ajnas = self.preset['jins']
        if i >= len(ajnas) or i == self.jins_i:
            return
        self.jins_i = i
        name, keys = ajnas[i]
        notes = ' '.join(self.preset['names'][k % KEYS_PER_OCTAVE] for k in keys)
        self.emit(self.melody_hand, f'JINS [{self.melody_hand}]: {name} ({notes})')

    def _shift_octave(self, ev):
        if not self.pose.fingers(ev.hand).press[2]:
            return
        new = max(-1, min(1, self.octave + int(ev.value)))
        if new != self.octave:
            self.octave = new
            self.emit(ev.hand, f'OCTAVE {new:+d} [{ev.hand}]')

    def _repluck(self):
        if self.voice is None:
            return
        self.midi_writer.trig_note(self.voice, 0)
        self.midi_writer.trig_note(self.voice, self._vel(self.MELODY_VEL + 10, 6))

    def _trill(self, now):
        fs = self.pose.fingers(self.melody_hand)
        active = fs is not None and bool(fs.press[3]) and bool(self.held)
        if not active:
            if self.trill_on:
                self.trill_on = False
                if self.held:
                    self._set_voice(self.finger_note[self.held[-1]][0])
            return
        base, upper = self.finger_note[self.held[-1]]
        if upper is None:
            return                              # top finger of the jins: nothing above
        half = 0.5 / self.TRILL_HZ
        if not self.trill_on:
            self.trill_on, self.trill_up, self.trill_due = True, False, now + half
        elif now >= self.trill_due:
            self.trill_up = not self.trill_up
            self.trill_due = now + half
            self._set_voice(upper if self.trill_up else base)

    def _vibrato(self, now):
        if self.live_controls.get_vibrato_enabled():
            mh = self.pose.latest_motion(self.melody_hand)
            depth = mh.roll_depth if (mh is not None and mh.fingers.press[0]) else 0.0
            bend = (math.sin(2 * math.pi * self.VIBRATO_HZ * now)
                    * depth * self.VIBRATO_MAX_BEND) if depth > 0.0 else 0.0
            if bend != self.last_bend:
                self.midi_writer.pitch_bend(bend)
                self.last_bend = bend
        elif self.live_controls.pop_vibrato_reset_pending():
            self.midi_writer.pitch_bend(0)
            self.last_bend = 0

    # -- percussion --------------------------------------------------------

    def _drum_note(self, key, vel):
        try:
            self.midi_writer.trig_note(key, vel, channel=self.DRUM_CHANNEL)
        except TypeError:                # midi_writer without channel support
            if not self.channel_warned:
                print('[maqam] WARNING: midi_writer.trig_note has no channel argument; '
                      'drums are going out on channel 1. Use the supplied midi_writer.py.')
                self.channel_warned = True
            self.midi_writer.trig_note(key, vel)

    def _hit(self, stroke, accent=0):
        key, vel = STROKES[stroke]
        self._drum_note(key, self._vel(vel + accent, 6))
        self.offs.append((time.monotonic() + self.DRUM_GATE_SEC, key))

    def _rhythm_fingers(self, hand):
        fs = self.pose.fingers(hand)
        if fs is None:
            return
        rises, _ = self._edges(hand, fs.flex)
        for i in rises:
            if i in self.FILL_STROKES:
                self._hit(self.FILL_STROKES[i], accent=-4)
                self.emit(hand, f'FILL [{hand}]: {self.FILL_STROKES[i]}')

    def _iqa_step(self, hand):
        name = self.iqa_names[self.iqa_i]
        pattern = IQA[name]
        stroke = pattern[self.iqa_pos]
        self._hit(stroke, accent=8 if self.iqa_pos == 0 else 0)
        self.emit(hand, f'{name.upper()} {self.iqa_pos + 1}/{len(pattern)} '
                        f'[{hand}]: {stroke}')
        self.iqa_pos = (self.iqa_pos + 1) % len(pattern)

    def _toggle_loop(self, hand):
        self.loop_on = not self.loop_on
        self.emit(hand, f'LOOP {"ON" if self.loop_on else "OFF"} [{hand}]: '
                        f'{self.iqa_names[self.iqa_i]} at {self.LOOP_BPM} bpm')

    def _loop_run(self):
        """Background clock: plays the current iqa' as 8 eighth-note slots while loop_on."""
        pending, next_t, slot = [], None, 0
        while not self._loop_stop.wait(0.002):
            now = time.perf_counter()
            for t, key in [p for p in pending if p[0] <= now]:
                self._drum_note(key, 0)
            pending = [p for p in pending if p[0] > now]
            if not self.loop_on:
                next_t = None
                continue
            if next_t is None:                     # just switched on: start on the downbeat
                next_t, slot = now, 0
            if now >= next_t:
                pat = IQA_SLOTS[self.iqa_names[self.iqa_i]].split()
                s = pat[slot % len(pat)]
                if s != '.':
                    key, vel = STROKES[s]
                    accent = 8 if slot % len(pat) == 0 else 0
                    self._drum_note(key, self._vel(vel + accent, 6))
                    pending.append((now + self.DRUM_GATE_SEC, key))
                slot += 1
                next_t += 30.0 / self.LOOP_BPM

    def _next_iqa(self, hand):
        self.iqa_i = (self.iqa_i + 1) % len(self.iqa_names)
        self.iqa_pos = 0
        pattern = ' '.join(IQA[self.iqa_names[self.iqa_i]])
        self.emit(hand, f'IQA [{hand}]: {self.iqa_names[self.iqa_i]} ({pattern})')

    # -- panic -------------------------------------------------------------

    def _all_notes_off(self, hand):
        self.loop_on = False
        self.held.clear()
        self.trill_on = False
        self._set_voice(None)
        for _, key in self.offs:
            self._drum_note(key, 0)
        self.offs.clear()
        self.emit(hand, f'ALL NOTES OFF [{hand}]')


if __name__ == "__main__":
    active_hands = scan_available_ble(timeout=5.0)
    if not active_hands:
        print("[Error] No active Gluvn hardware found.")
        sys.exit(1)
    print(f"Discovered gloves: {[h.upper() for h in sorted(active_hands)]}")

    PRESET = 'rast1'          # must match the tuning loaded in Live (see PRESETS)
    IQA_START = 'maqsum'

    trigger_sensors = {h: ['press', 'flex'] for h in active_hands}
    thresholds = {'press': 20, 'flex': 120}
    hysteresis = {'press': 5, 'flex': 2}

    # Same motion thresholds as the choir. Pitch steps fire beyond +/-30 deg from
    # the calibrated rest pose and re-arm inside +/-20 deg.
    motion_config = MotionConfig(
        accel_norm_max=15.0, avg_window_size=10,
        accel_trigger_thresh=110, accel_trigger_hysteresis=10,
        roll_trigger_thresh_range=20, roll_trigger_hysteresis=5,
        pitch_trigger_thresh_range=5, pitch_trigger_hysteresis=5,
        yaw_window=10, pitch_step_mode='directional',
        pitch_trigger_angle_deg=30.0, pitch_trigger_rearm_deg=20.0,
        vibrato_enabled=True, calibration_window_sec=2.0)

    app = ArabicMaqam(
        root_note='C', scale='major',      # unused here; base constructor expects them
        preset=PRESET, iqa=IQA_START,
        send_all_data=True,
        trigger_sensors=trigger_sensors,
        thresholds=thresholds,
        hysteresis=hysteresis,
        hands=sorted(active_hands),
        use_ble=True,
        enable_motion_stream=True,
        motion_config=motion_config,
    )
    app.start()

    from ui.digital_twin import build_app_and_window   # Qt only when the twin is wanted
    qt_app, twin_window = build_app_and_window(reader=app)
    twin_window.sector_bar.hide()      # yaw-sector panel is choir-specific; unused here

    def _shutdown():
        print('Shutting down...')
        app.shutdown()

    qt_app.aboutToQuit.connect(_shutdown)
    sys.exit(qt_app.exec())