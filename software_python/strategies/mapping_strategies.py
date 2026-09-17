"""
Mapping Strategy Implementations for GLUVN System

This module contains concrete implementations of mapping strategies that convert
trigger events into MIDI note assignments.

Strategies:
- BasicMapper: Simple finger-to-note mapping for both hands
- WindowMapper: Moving window note selection (to be implemented)
- ChordMapper: Chord-based harmonization (to be implemented)

Author: Joseph Bakarji
Updated: Jul 2026 - Helene Jabbour
"""

from .base_strategies import MappingStrategy
import numpy as np
from typing import Dict, Any, List, Optional


class BasicMapper(MappingStrategy):
    """
    Simple finger-to-note mapping for both hands
    
    This strategy implements a straightforward mapping where each finger
    corresponds to a specific note in the scale. It supports both hands
    with independent note assignments.
    """
    
    def initialize(self):
        """Initialize basic note mappings for both hands"""
        # Get the basic note mapping from the note mapper
        self.note_maps = self.note_mapper.basic_map_2hands()
        
        # Track currently active notes for each hand
        self._current_notes_l = [None] * 5
        self._current_notes_r = [None] * 5
        
        # Store configuration parameters
        self.base_note = self.config.get('base_note', 'C3')
        
    def map_to_notes(self, trigger_events: np.ndarray, hand: str, **kwargs) -> List[Optional[int]]:
        """
        Map trigger events to MIDI note numbers
        
        Args:
            trigger_events: Array of trigger events (1=on, -1=off, 0=no change)
            hand: Hand identifier ('l' or 'r')
            **kwargs: Additional context (velocity, etc.)
            
        Returns:
            List of MIDI note numbers to trigger (None for no action)
        """
        if hand not in self.note_maps:
            return [None] * 5
            
        notes_to_trigger = []
        hand_notes = self.note_maps[hand]
        current_notes = getattr(self, f'_current_notes_{hand}')
        
        # Process each finger's trigger event
        for i, event in enumerate(trigger_events):
            if i >= len(hand_notes):
                notes_to_trigger.append(None)
                continue
                
            if event == 1:  # Note on
                note = hand_notes[i]
                notes_to_trigger.append(note)
                current_notes[i] = note
            elif event == -1:  # Note off
                # Return the currently playing note to turn it off
                if current_notes[i] is not None:
                    notes_to_trigger.append(current_notes[i])
                    current_notes[i] = None
                else:
                    notes_to_trigger.append(None)
            else:  # No change
                notes_to_trigger.append(None)
                
        return notes_to_trigger
    
    def update_mapping(self, **kwargs):
        """
        Update mapping parameters dynamically
        
        Args:
            **kwargs: Parameters to update (root_note, scale, etc.)
        """
        if 'root_note' in kwargs:
            self.note_mapper.root_note = kwargs['root_note']
            self.note_mapper.notes_in_scale = self.note_mapper.get_all_notes(
                kwargs['root_note'], 
                self.note_mapper.scale
            )
            self.note_maps = self.note_mapper.basic_map_2hands()
            
        if 'scale' in kwargs:
            self.note_mapper.scale = kwargs['scale']
            self.note_mapper.notes_in_scale = self.note_mapper.get_all_notes(
                self.note_mapper.root_note, 
                kwargs['scale']
            )
            self.note_maps = self.note_mapper.basic_map_2hands()
    
    def get_current_notes(self, hand: str) -> List[Optional[int]]:
        """
        Get currently active notes for a hand
        
        Args:
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            List of currently active MIDI note numbers
        """
        return getattr(self, f'_current_notes_{hand}', [None] * 5)
    
    def get_note_mapping(self, hand: str) -> List[int]:
        """
        Get the complete note mapping for a hand
        
        Args:
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            List of MIDI note numbers assigned to each finger
        """
        return self.note_maps.get(hand, [])


class WindowMapper(MappingStrategy):
    """
    Moving window note selection

    Left hand selects a note window (register) via its finger-flex pattern;
    right hand plays the 5 notes within that window. Window transposition is
    diatonic (fixed scale-degree steps), not chromatic, so scale membership
    is preserved across all windows — this comes directly from
    note_mapper.generate_windows().
    """

    def initialize(self):
        """Build the window trigger matrix and note windows from note_mapper."""
        num_rhf = self.config.get('num_rhf', 5)
        num_lhf = self.config.get('num_lhf', 5)
        self.window_trigger, self.note_windows = self.note_mapper.moving_window(
            num_rhf=num_rhf, num_lhf=num_lhf
        )
        self._current_notes_l = [None] * 5
        self._current_notes_r = [None] * 5
        self._current_window_idx = None

    def map_to_notes(self, trigger_events: np.ndarray, hand: str, **kwargs) -> List[Optional[int]]:
        """
        Map trigger events to notes within the currently selected window.

        Left hand triggers do not produce notes; they only report finger
        state via `window_switch` (passed as a kwarg, or taken directly from
        `trigger_events` for hand='l') so the right-hand call can resolve
        the active window through note_mapper.window_map().

        kwargs:
            window_switch : array-like, optional
                Left-hand finger state (matches window_trigger row width).
                Required for hand='r' to select the correct window; if
                omitted, the previously resolved window is reused, or the
                open-palm window on first call.
        """
        if hand == 'l':
            # Left hand only selects the window; it does not play notes.
            return [None] * 5

        window_switch = kwargs.get('window_switch')
        if window_switch is not None:
            hand_notes, self._current_window_idx = self.note_mapper.window_map(
                window_switch, self.window_trigger, self.note_windows
            )
        elif self._current_window_idx is not None:
            hand_notes = self.note_windows[self._current_window_idx]
        else:
            # First call, no window_switch yet: fall back to open-palm window.
            hand_notes, self._current_window_idx = self.note_mapper.window_map(
                np.zeros(self.window_trigger.shape[1]), self.window_trigger, self.note_windows
            )

        notes_to_trigger = []
        current_notes = self._current_notes_r

        for i, event in enumerate(trigger_events):
            if i >= len(hand_notes):
                notes_to_trigger.append(None)
                continue

            if event == 1:  # Note on
                note = int(hand_notes[i])
                notes_to_trigger.append(note)
                current_notes[i] = note
            elif event == -1:  # Note off
                if current_notes[i] is not None:
                    notes_to_trigger.append(current_notes[i])
                    current_notes[i] = None
                else:
                    notes_to_trigger.append(None)
            else:
                notes_to_trigger.append(None)

        return notes_to_trigger

    def update_mapping(self, **kwargs):
        """Update root_note/scale, then regenerate window_trigger/note_windows."""
        if 'root_note' in kwargs:
            self.note_mapper.root_note = kwargs['root_note']
            self.note_mapper.notes_in_scale = self.note_mapper.get_all_notes(
                kwargs['root_note'], self.note_mapper.scale
            )
        if 'scale' in kwargs:
            self.note_mapper.scale = kwargs['scale']
            self.note_mapper.notes_in_scale = self.note_mapper.get_all_notes(
                self.note_mapper.root_note, kwargs['scale']
            )
        if 'root_note' in kwargs or 'scale' in kwargs:
            num_rhf = self.config.get('num_rhf', 5)
            num_lhf = self.config.get('num_lhf', 5)
            self.window_trigger, self.note_windows = self.note_mapper.moving_window(
                num_rhf=num_rhf, num_lhf=num_lhf
            )
            self._current_window_idx = None


class ChordMapper(MappingStrategy):
    """
    Chord-based harmonization mapping

    Each finger triggers a diatonic chord (default: triad) rooted at that
    finger's scale note, rather than a single note. Chord quality (major/
    minor/diminished) follows automatically from the active scale's own
    interval pattern via note_mapper.get_chord() — no separate chord-quality
    table needed.

    map_to_notes() returns the chord ROOT per finger (for interface
    compatibility with other MappingStrategy implementations, e.g. simple
    note-on/off counting). The full chord (all stacked tones) for each
    currently-sounding finger is available via get_current_chord(hand).
    """

    def initialize(self):
        """Initialize chord mapping: base note-per-finger map + chord config."""
        self.note_maps = self.note_mapper.basic_map_2hands()
        self.chord_size = self.config.get('chord_size', 3)       # 3 = triad, 4 = seventh chord
        self.chord_interval = self.config.get('chord_interval', 2)  # 2 = thirds

        self._current_notes_l = [None] * 5
        self._current_notes_r = [None] * 5
        self._current_chords_l = [None] * 5
        self._current_chords_r = [None] * 5

    def map_to_notes(self, trigger_events: np.ndarray, hand: str, **kwargs) -> List[Optional[int]]:
        """Map trigger events to chord roots, building the full chord internally."""
        if hand not in self.note_maps:
            return [None] * 5

        notes_to_trigger = []
        hand_notes = self.note_maps[hand]
        current_notes = getattr(self, f'_current_notes_{hand}')
        current_chords = getattr(self, f'_current_chords_{hand}')

        for i, event in enumerate(trigger_events):
            if i >= len(hand_notes):
                notes_to_trigger.append(None)
                continue

            if event == 1:  # Note on
                root_note = hand_notes[i]
                notes_to_trigger.append(root_note)
                current_notes[i] = root_note
                root_idx = self.note_mapper.notes_in_scale.index(root_note)
                current_chords[i] = self.note_mapper.get_chord(
                    root_idx, num_notes=self.chord_size, interval=self.chord_interval
                )
            elif event == -1:  # Note off
                if current_notes[i] is not None:
                    notes_to_trigger.append(current_notes[i])
                    current_notes[i] = None
                    current_chords[i] = None
                else:
                    notes_to_trigger.append(None)
            else:
                notes_to_trigger.append(None)

        return notes_to_trigger

    def get_current_chord(self, hand: str, finger: int) -> Optional[List[int]]:
        """Return the full chord (list of MIDI notes) currently sounding at `finger`, or None."""
        chords = getattr(self, f'_current_chords_{hand}', [None] * 5)
        if finger >= len(chords):
            return None
        return chords[finger]

    def update_mapping(self, **kwargs):
        """Update root_note/scale/chord_size/chord_interval."""
        if 'root_note' in kwargs:
            self.note_mapper.root_note = kwargs['root_note']
            self.note_mapper.notes_in_scale = self.note_mapper.get_all_notes(
                kwargs['root_note'], self.note_mapper.scale
            )
            self.note_maps = self.note_mapper.basic_map_2hands()

        if 'scale' in kwargs:
            self.note_mapper.scale = kwargs['scale']
            self.note_mapper.notes_in_scale = self.note_mapper.get_all_notes(
                self.note_mapper.root_note, kwargs['scale']
            )
            self.note_maps = self.note_mapper.basic_map_2hands()

        if 'chord_size' in kwargs:
            self.chord_size = kwargs['chord_size']

        if 'chord_interval' in kwargs:
            self.chord_interval = kwargs['chord_interval']