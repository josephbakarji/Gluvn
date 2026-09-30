"""
GLUVN Musical Mapper Module

Musical theory and note mapping for the sensor glove system: scale
generation, chord mapping, and note window management. Supports single-hand
and dual-hand operation. This module is the single musical-theory authority
for GLUVN -- strategy classes (TriggerStrategy, MappingStrategy,
ModulationStrategy) should delegate note/scale/chord logic here rather than
reimplementing it.

Supported scales: major, natural/harmonic/melodic minor, pentatonic
(extensible via SCALES).

Window system: the moving window lets the left hand select which scale
window is active (via finger position), while the right hand plays notes
within it -- giving access to a much larger note range than 5-10 fingers alone.

Usage:
    mapper = NoteMapper(root_note='C', scale='major')
    notes = mapper.basic_map_2hands()
    window_trigger, note_windows = mapper.moving_window()

Author: Joseph Bakarji
Last Updated: July 2026 by Helene Jabbour — M5StickC Plus / BLE adaptation
"""

import numpy as np


SCALES = {
    'major':              [2, 2, 1, 2, 2, 2, 1],
    'minor':              [2, 1, 2, 2, 1, 2, 2],  # alias for natural_minor
    'natural_minor':      [2, 1, 2, 2, 1, 2, 2],
    'harmonic_minor':     [2, 1, 2, 2, 1, 3, 1],
    'melodic_minor_asc':  [2, 1, 2, 2, 2, 2, 1],
    'melodic_minor_desc': [2, 1, 2, 2, 1, 2, 2],
    'pentatonic':         [3, 2, 2, 3, 2],
}

NOTES_SHARP = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
NOTES_FLAT  = ['C', 'Db', 'D', 'Eb', 'E', 'F', 'Gb', 'G', 'Ab', 'A', 'Bb', 'B']

MIDI_MIN = 24   # C1
MIDI_MAX = 127


class NoteMapper:
    def __init__(self, root_note='C', scale='major'):
        if scale not in SCALES:
            raise ValueError(f"Unknown scale '{scale}'. Supported: {list(SCALES.keys())}")
        self.scale      = scale
        self.root_note  = root_note
        self.note2midi, self.midi2note = self._generate_notemidi_dict()
        self.notes_in_scale = self.get_all_notes(root_note, scale)

    # -------------------------------------------------------
    # Note / MIDI dictionaries
    # -------------------------------------------------------
    def _generate_notemidi_dict(self):
        """
        Build bidirectional note<->MIDI dicts.
        MIDI 24 = C0, MIDI 127 = G10 (octave-index convention used here; not standard MIDI octave numbering).
        Both sharp and flat spellings map to the same MIDI number.
        """
        note2midi = {}
        midi2note = {}
        midi = MIDI_MIN
        for octave in range(10):
            for sharp, flat in zip(NOTES_SHARP, NOTES_FLAT):
                if midi > MIDI_MAX:
                    break
                note2midi[f"{sharp}{octave}"] = midi
                note2midi[f"{flat}{octave}"]  = midi
                midi2note[midi]               = f"{sharp}{octave}"
                midi += 1
            if midi > MIDI_MAX:
                break
        return note2midi, midi2note

    # -------------------------------------------------------
    # Scale generation
    # -------------------------------------------------------
    def get_all_notes(self, root_note='C', scale='major'):
        """Return all MIDI note numbers in the given scale from MIDI_MIN to MIDI_MAX."""
        intervals  = SCALES[scale]
        root_midi  = self.note2midi.get(root_note + '0')
        if root_midi is None:
            raise ValueError(f"Root note '{root_note}' not found in note dictionary")

        midi_notes = [root_midi]
        i = 0
        while True:
            next_note = midi_notes[-1] + intervals[i % len(intervals)]
            if next_note > MIDI_MAX:
                break
            midi_notes.append(next_note)
            i += 1
        return midi_notes

    def _get_all_notes(self, root_note='C', scale='major'):
        """Deprecated alias for get_all_notes(). Kept for backward compatibility."""
        return self.get_all_notes(root_note, scale)

    def _find_root_idx(self, octave_hint=3):
        """
        Find the index of root_note in notes_in_scale, starting near octave_hint.
        Falls back to any octave if the hinted one isn't in the scale.
        """
        for octave in range(octave_hint, -1, -1):
            key  = self.root_note + str(octave)
            midi = self.note2midi.get(key)
            if midi and midi in self.notes_in_scale:
                return self.notes_in_scale.index(midi)

        for octave in range(octave_hint + 1, 10):
            key  = self.root_note + str(octave)
            midi = self.note2midi.get(key)
            if midi and midi in self.notes_in_scale:
                return self.notes_in_scale.index(midi)

        raise ValueError(
            f"Root note '{self.root_note}' not found in scale '{self.scale}'"
        )

    # -------------------------------------------------------
    # Chord generation
    # -------------------------------------------------------
    def get_chord(self, root_idx, num_notes=3, interval=2):
        """
        Stack notes from notes_in_scale at fixed scale-degree intervals
        (diatonic tertian stacking — interval=2 gives thirds within the
        active scale, so major/minor quality follows automatically from
        the scale's own interval pattern).

        Parameters
        ----------
        root_idx : int
            Index into notes_in_scale for the chord root.
        num_notes : int
            Number of chord tones to stack (3 = triad, 4 = seventh chord).
        interval : int
            Scale-degree step between stacked tones (2 = thirds, 1 = seconds).

        Returns
        -------
        List of MIDI note numbers. Shorter than num_notes if the stack
        runs past MIDI_MAX.
        """
        return [
            self.notes_in_scale[root_idx + interval * i]
            for i in range(num_notes)
            if root_idx + interval * i < len(self.notes_in_scale)
        ]

    def get_chord_from_note(self, note_name, num_notes=3, interval=2):
        """Same as get_chord(), but takes a note name (e.g. 'C3') instead of a scale index."""
        midi = self.note2midi.get(note_name)
        if midi is None or midi not in self.notes_in_scale:
            raise ValueError(f"Note '{note_name}' not in scale '{self.scale}'")
        root_idx = self.notes_in_scale.index(midi)
        return self.get_chord(root_idx, num_notes=num_notes, interval=interval)

    # -------------------------------------------------------
    # Note mapping — single and dual hand
    # -------------------------------------------------------
    def basic_map(self, first_note='C3', num_notes=5):
        """Return the first num_notes scale notes starting from first_note (MIDI list)."""
        midi = self.note2midi.get(first_note)
        if midi is None:
            raise ValueError(f"Note '{first_note}' not in note dictionary")
        if midi not in self.notes_in_scale:
            raise ValueError(f"Note '{first_note}' not in scale '{self.scale}'")
        idx = self.notes_in_scale.index(midi)
        return self.notes_in_scale[idx: idx + num_notes]

    def basic_map_2hands(self, first_note=None, hands=None):
        """
        Map scale notes to one or both hands.

        Parameters
        ----------
        first_note : str, optional
            Starting note for the right hand (default: root_note + '3').
        hands : list, optional
            Which hands to map. Defaults to ['r', 'l'] for backward compatibility.
            Pass ['r'] when only the right glove is connected.

        Returns
        -------
        dict  {hand: [midi_note, ...]}  — only keys in `hands` are present.
        """
        if hands is None:
            hands = ['r', 'l']

        if first_note is None:
            first_note = self.root_note + '3'

        idx = self._find_root_idx(octave_hint=3)

        note_dict = {}
        if 'r' in hands:
            note_dict['r'] = self.notes_in_scale[idx: idx + 5]
        if 'l' in hands:
            # Left hand plays the 5 notes immediately below right hand, reversed
            l_start = max(0, idx - 5)
            note_dict['l'] = self.notes_in_scale[l_start: idx][::-1]

        return note_dict

    # -------------------------------------------------------
    # Moving window system
    # -------------------------------------------------------
    def moving_window(self, num_rhf=5, num_lhf=5):
        """
        Build window trigger matrix and note windows for the double-flex instrument.

        Returns
        -------
        window_trigger : np.ndarray  shape (num_windows, num_lhf)
        note_windows   : np.ndarray  shape (num_windows, num_rhf)
        """
        window_trigger = self.window_setter(mode='standard', num_fingers=num_lhf)
        note_windows   = self.generate_windows(window_trigger, num_fingers=num_rhf)
        return window_trigger, note_windows

    def generate_windows(self, window_trigger, num_fingers=5):
        """
        Generate a set of note windows aligned to the window_trigger rows.
        Each row in note_windows corresponds to a row in window_trigger.
        """
        idx_root = self._find_root_idx(octave_hint=3)
        num_windows = window_trigger.shape[0]

        window_list = []
        for i in range(20):
            idx0 = idx_root + num_fingers * i
            if idx0 + num_fingers > len(self.notes_in_scale):
                break
            window_list.append(self.notes_in_scale[idx0: idx0 + num_fingers])

        for i in range(1, 20):
            idx0 = idx_root - num_fingers * i
            if idx0 < 0:
                break
            window_list.append(self.notes_in_scale[idx0: idx0 + num_fingers])

        # Sort by lowest note ascending
        window_list = np.array(window_list)
        window_list = window_list[np.argsort(window_list[:, 0])]

        root_midi     = self.notes_in_scale[idx_root]
        root_window_idx = np.where(window_list[:, 0] == root_midi)[0]
        if len(root_window_idx) == 0:
            raise ValueError("Root window not found in generated windows")
        root_window_idx = root_window_idx[0]

        # Open-palm row = all zeros in window_trigger
        fingers_bent   = np.sum(window_trigger, axis=1)
        open_palm_rows = np.where(fingers_bent == 0)[0]
        if len(open_palm_rows) == 0:
            raise ValueError("No open-palm row (all zeros) found in window_trigger")
        open_palm_idx = open_palm_rows[0]

        # Align root window to the open-palm row
        while open_palm_idx > root_window_idx:
            window_list     = np.vstack((window_list[0, :], window_list))
            root_window_idx = np.where(window_list[:, 0] == root_midi)[0][0]

        idx0   = max(0, root_window_idx - open_palm_idx)
        idxend = min(idx0 + num_windows, window_list.shape[0])
        result = window_list[idx0: idxend]

        # Pad with last window if fewer windows than trigger rows
        if result.shape[0] < num_windows:
            pad = np.tile(result[-1], (num_windows - result.shape[0], 1))
            result = np.vstack((result, pad))

        return result

    def window_setter(self, mode='standard', num_fingers=5):
        """Return the window trigger matrix for the given mode and finger count."""
        if mode == 'standard':
            if num_fingers == 5:
                return np.array([
                    [1, 1, 1, 1, 1],
                    [0, 1, 1, 1, 1],
                    [0, 0, 1, 1, 1],
                    [0, 0, 0, 1, 1],
                    [0, 0, 0, 0, 1],
                    [0, 0, 0, 0, 0],
                    [1, 0, 0, 0, 0],
                    [1, 1, 0, 0, 0],
                    [1, 1, 1, 0, 0],
                    [1, 1, 1, 1, 0],
                ])
            elif num_fingers == 3:
                return np.array([
                    [1, 1, 1],
                    [0, 1, 1],
                    [0, 0, 1],
                    [0, 0, 0],
                    [1, 0, 0],
                    [1, 1, 0],
                ])
        raise ValueError(f"Unsupported mode '{mode}' or num_fingers={num_fingers}")

    def window_map(self, nswitch, window_trigger, note_windows):
        """
        Return the note window matching the given finger state.

        Parameters
        ----------
        nswitch        : array-like, finger state to match
        window_trigger : np.ndarray
        note_windows   : np.ndarray

        Returns
        -------
        (note_array, index) or (note_windows[open_palm_idx], open_palm_idx) as fallback
        """
        nswitch = np.asarray(nswitch)
        idx_list = np.where(np.all(window_trigger == nswitch, axis=1))[0]
        if len(idx_list) > 0:
            idx = idx_list[0]
            return note_windows[idx], idx

        # Fallback: open-palm window
        fingers_bent   = np.sum(window_trigger, axis=1)
        open_palm_rows = np.where(fingers_bent == 0)[0]
        if len(open_palm_rows) > 0:
            idx = open_palm_rows[0]
            return note_windows[idx], idx

        # Last resort: first window
        return note_windows[0], 0

