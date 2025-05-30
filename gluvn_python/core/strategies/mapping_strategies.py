"""
Mapping Strategy Implementations for GLUVN System

This module contains concrete implementations of mapping strategies that convert
trigger events into MIDI note assignments.

Strategies:
- BasicMapper: Simple finger-to-note mapping for both hands
- WindowMapper: Moving window note selection (to be implemented)
- ChordMapper: Chord-based harmonization (to be implemented)

Author: Joseph Bakarji
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
    Moving window note selection (placeholder for future implementation)
    
    This strategy will implement the moving window system where the left hand
    selects note windows and the right hand plays notes within the window.
    """
    
    def initialize(self):
        """Initialize window mapping system"""
        # Placeholder implementation
        self.note_maps = self.note_mapper.basic_map_2hands()
        self._current_notes_l = [None] * 5
        self._current_notes_r = [None] * 5
    
    def map_to_notes(self, trigger_events: np.ndarray, hand: str, **kwargs) -> List[Optional[int]]:
        """Placeholder implementation - delegates to basic mapping for now"""
        # TODO: Implement window selection logic
        basic_mapper = BasicMapper(self.note_mapper, self.config)
        return basic_mapper.map_to_notes(trigger_events, hand, **kwargs)
    
    def update_mapping(self, **kwargs):
        """Placeholder for window mapping updates"""
        pass


class ChordMapper(MappingStrategy):
    """
    Chord-based harmonization mapping (placeholder for future implementation)
    
    This strategy will implement chord generation and harmonization based on
    trigger events and musical context.
    """
    
    def initialize(self):
        """Initialize chord mapping system"""
        # Placeholder implementation
        self.note_maps = self.note_mapper.basic_map_2hands()
        self._current_notes_l = [None] * 5
        self._current_notes_r = [None] * 5
    
    def map_to_notes(self, trigger_events: np.ndarray, hand: str, **kwargs) -> List[Optional[int]]:
        """Placeholder implementation - delegates to basic mapping for now"""
        # TODO: Implement chord generation logic
        basic_mapper = BasicMapper(self.note_mapper, self.config)
        return basic_mapper.map_to_notes(trigger_events, hand, **kwargs)
    
    def update_mapping(self, **kwargs):
        """Placeholder for chord mapping updates"""
        pass 