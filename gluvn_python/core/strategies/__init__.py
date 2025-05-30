"""
Strategy pattern implementations for GLUVN system

This module contains all strategy implementations for the GLUVN sensor-to-MIDI system:
- Trigger strategies: How sensor data is converted to discrete events
- Mapping strategies: How trigger events are mapped to musical notes
- Modulation strategies: How continuous sensor data controls MIDI parameters

The strategy pattern allows for flexible combination of different processing approaches
while maintaining a clean, extensible architecture.
"""

from .base_strategies import TriggerStrategy, MappingStrategy, ModulationStrategy

# Import concrete trigger strategies
from .trigger_strategies import HysteresisTrigger, MultiSensorTrigger, IMUDirectionalTrigger

# Import concrete mapping strategies
from .mapping_strategies import BasicMapper, WindowMapper, ChordMapper

# Import concrete implementations when they're created
try:
    from .modulation_strategies import VolumeModulation, AccelVolumeModulation, PitchBendModulation, IMUPitchBend
except ImportError:
    # Strategies not yet implemented
    pass

__all__ = [
    'TriggerStrategy', 'MappingStrategy', 'ModulationStrategy',
    'HysteresisTrigger', 'MultiSensorTrigger', 'IMUDirectionalTrigger',
    'BasicMapper', 'WindowMapper', 'ChordMapper'
] 