"""
GLUVN Visualization Components

This module contains PyQt-based visualization components that can be reused
across different GLUVN applications.

Components:
- FingerSensorWidget: Individual finger sensor display with real-time values
- SensorProcessingThread: Thread for real-time sensor data processing
- TenFingerDisplay: Complete ten-finger hand display widget
- PianoKeyboard: Piano keyboard display with note triggering visualization
- CompactPianoKeyboard: Compact piano keyboard for tight layouts

Modulation Strategies (re-exported for convenience):
- AccelVolumeModulation: Accelerometer-based volume control
- IMUPitchBendModulation: IMU-based pitch bend control
- MovingWindowModulation: Dynamic window averaging control
- ChoirModulation: Multi-voice IMU control

Author: Joseph Bakarji
"""

from .finger_widgets import FingerSensorWidget, TenFingerDisplay, EnhancedFingerSensorWidget, EnhancedTenFingerDisplay
from .sensor_threads import SensorProcessingThread
from .piano_keyboard import PianoKeyboard, CompactPianoKeyboard, PianoKey

# Re-export modulation strategies for convenient access
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from core.strategies.modulation_strategies import (
    AccelVolumeModulation, 
    IMUPitchBendModulation, 
    MovingWindowModulation, 
    ChoirModulation,
    CompositeModulation
)

__all__ = [
    'FingerSensorWidget', 
    'TenFingerDisplay', 
    'EnhancedFingerSensorWidget',
    'EnhancedTenFingerDisplay',
    'SensorProcessingThread',
    'PianoKeyboard',
    'CompactPianoKeyboard',
    'PianoKey',
    'AccelVolumeModulation',
    'IMUPitchBendModulation', 
    'MovingWindowModulation',
    'ChoirModulation',
    'CompositeModulation'
] 