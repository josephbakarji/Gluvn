"""
Strategy pattern implementations for GLUVN system

This module contains all strategy implementations for the GLUVN sensor-to-MIDI system:
- Trigger strategies: How sensor data is converted to discrete events
- Mapping strategies: How trigger events are mapped to musical notes
- Modulation strategies: How continuous sensor data controls MIDI parameters

The strategy pattern allows for flexible combination of different processing approaches
while maintaining a clean, extensible architecture.
"""

from .base_strategies import TriggerStrategy, MappingStrategy, ModulationStrategy, CompositeStrategy

# Import concrete trigger strategies
from .trigger_strategies import HysteresisTrigger, MultiSensorTrigger, IMUDirectionalTrigger

# Import concrete mapping strategies
from .mapping_strategies import BasicMapper, WindowMapper, ChordMapper

# Import concrete modulation strategies
from .modulation_strategies import (
    AccelVolumeModulation,
    IMUPitchBendModulation,
    MovingWindowModulation,
    ChoirModulation,
    CompositeModulation,
)

__all__ = [
    'TriggerStrategy', 'MappingStrategy', 'ModulationStrategy', 'CompositeStrategy',
    'HysteresisTrigger', 'MultiSensorTrigger', 'IMUDirectionalTrigger',
    'BasicMapper', 'WindowMapper', 'ChordMapper',
    'AccelVolumeModulation', 'IMUPitchBendModulation', 'MovingWindowModulation',
    'ChoirModulation', 'CompositeModulation',
]