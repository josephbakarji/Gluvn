"""
Base Strategy Classes for GLUVN System

This module defines the abstract base classes for all strategy implementations
in the GLUVN sensor-to-MIDI system. These classes establish the interfaces
that concrete strategies must implement.

Strategy Types:
- TriggerStrategy: Converts sensor data to discrete trigger events
- MappingStrategy: Maps trigger events to MIDI note numbers
- ModulationStrategy: Processes continuous sensor data for MIDI control

Author: Joseph Bakarji
Updated: Jul 2026 - Helene Jabbour
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, List, Optional, Union
import numpy as np


class TriggerStrategy(ABC):
    """
    Base class for trigger logic implementations
    
    Trigger strategies are responsible for converting raw sensor data into
    discrete trigger events (note on/off). They handle hysteresis, thresholds,
    and state management to provide stable triggering behavior.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the trigger strategy
        
        Args:
            config: Configuration dictionary containing strategy-specific parameters
        """
        self.config = config
        self.trigger_states = {}
        self.initialize()
    
    @abstractmethod
    def initialize(self):
        """Initialize strategy-specific state and parameters"""
        pass
    
    @abstractmethod
    def process_triggers(self, sensor_data: Dict[str, Any], hand: str) -> np.ndarray:
        """
        Process sensor data and return trigger events
        
        Args:
            sensor_data: Dictionary containing sensor readings
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Array of trigger events: 1 for note on, -1 for note off, 0 for no change
        """
        pass
    
    def get_trigger_state(self, hand: str) -> np.ndarray:
        """
        Get current trigger state for hand
        
        Args:
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Boolean array indicating which fingers are currently triggered
        """
        return self.trigger_states.get(hand, np.zeros(5, dtype=bool))
    
    def reset_state(self, hand: Optional[str] = None):
        """
        Reset trigger state
        
        Args:
            hand: Hand to reset, or None to reset all hands
        """
        if hand is None:
            self.trigger_states.clear()
        else:
            self.trigger_states[hand] = np.zeros(5, dtype=bool)


class MappingStrategy(ABC):
    """
    Base class for sensor-to-note mapping strategies
    
    Mapping strategies determine which MIDI notes are triggered based on
    sensor events. They can implement simple finger-to-note mappings,
    complex harmonic relationships, or dynamic note selection systems.
    """
    
    def __init__(self, note_mapper, config: Dict[str, Any]):
        """
        Initialize the mapping strategy
        
        Args:
            note_mapper: NoteMapper instance for musical theory operations
            config: Configuration dictionary containing strategy-specific parameters
        """
        self.note_mapper = note_mapper
        self.config = config
        self.initialize()
    
    @abstractmethod
    def initialize(self):
        """Initialize strategy-specific state and note mappings"""
        pass
    
    @abstractmethod
    def map_to_notes(self, trigger_events: np.ndarray, hand: str, **kwargs) -> List[Optional[int]]:
        """
        Map trigger events to MIDI note numbers
        
        Args:
            trigger_events: Array of trigger events from TriggerStrategy
            hand: Hand identifier ('l' or 'r')
            **kwargs: Additional context (e.g., current velocity, modulation state)
            
        Returns:
            List of MIDI note numbers (or None for no note)
        """
        pass
    
    @abstractmethod
    def update_mapping(self, **kwargs) -> None:
        """
        Update mapping parameters dynamically
        
        Args:
            **kwargs: Parameters to update (strategy-specific)
        """
        pass
    
    def get_current_notes(self, hand: str) -> List[Optional[int]]:
        """
        Get currently active notes for a hand
        
        Args:
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            List of currently active MIDI note numbers
        """
        return getattr(self, f'_current_notes_{hand}', [])


class ModulationStrategy(ABC):
    """
    Base class for continuous control strategies
    
    Modulation strategies process continuous sensor data to generate
    MIDI control messages like volume, pitch bend, aftertouch, and
    other continuous controllers.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the modulation strategy
        
        Args:
            config: Configuration dictionary containing strategy-specific parameters
        """
        self.config = config
        self.modulation_state = {}
        self.initialize()
    
    @abstractmethod
    def initialize(self):
        """Initialize strategy-specific state and parameters"""
        pass
    
    @abstractmethod
    def process_modulation(self, sensor_data: Dict[str, Any], hand: str) -> Dict[str, float]:
        """
        Process sensor data for continuous controls
        
        Args:
            sensor_data: Dictionary containing sensor readings
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Dictionary of modulation values:
            - 'volume': Volume/velocity (0-127)
            - 'pitch_bend': Pitch bend (-8192 to 8192)
            - 'aftertouch': Channel aftertouch (0-127)
            - 'cc_<number>': Continuous controller values (0-127)
        """
        pass
    
    def get_modulation_state(self, hand: str) -> Dict[str, float]:
        """
        Get current modulation state for hand
        
        Args:
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Dictionary of current modulation values
        """
        return self.modulation_state.get(hand, {})
    
    def reset_state(self, hand: Optional[str] = None):
        """
        Reset modulation state
        
        Args:
            hand: Hand to reset, or None to reset all hands
        """
        if hand is None:
            self.modulation_state.clear()
        else:
            self.modulation_state[hand] = {}

    def _input_scaling(self, input_val, min_output=0, max_output=127, shift=0,
                        min_input=0, max_input=255):
        """
        Linear scaling of a raw input value into an output range, clamped
        to [min_output, max_output]. Shared by all ModulationStrategy
        subclasses — do not redefine per-strategy.

        Args:
            input_val: Raw sensor value to scale
            min_output, max_output: Target output range
            shift: Offset applied to input_val before scaling
            min_input, max_input: Expected raw input range

        Returns:
            Scaled and clamped output value
        """
        if max_input == min_input:
            return min_output

        output = min_output + (max_output - min_output) * (
            (input_val + shift) - min_input
        ) / (max_input - min_input)
        return max(min_output, min(max_output, output))


class CompositeStrategy(ABC):
    """
    Base class for strategies that combine multiple sub-strategies
    
    This allows for complex behaviors that might need multiple
    processing approaches working together.
    """
    
    def __init__(self, strategies: List[Union[TriggerStrategy, MappingStrategy, ModulationStrategy]], 
                 config: Dict[str, Any]):
        """
        Initialize the composite strategy
        
        Args:
            strategies: List of sub-strategies to combine
            config: Configuration dictionary
        """
        self.strategies = strategies
        self.config = config
        self.initialize()
    
    @abstractmethod
    def initialize(self):
        """Initialize composite strategy state"""
        pass
    
    @abstractmethod
    def combine_results(self, results: List[Any]) -> Any:
        """
        Combine results from multiple sub-strategies
        
        Args:
            results: List of results from sub-strategies
            
        Returns:
            Combined result
        """
        pass