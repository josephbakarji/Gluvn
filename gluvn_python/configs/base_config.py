"""
Base configuration file for gluvn applications
Contains defaults and common settings
"""
import json
import os
from typing import Dict, Any, Optional

# Default sensor settings
sensor_defaults = {
    'thresholds': {
        'flex': 200,
        'press': 15,
        'imu': 32767
    },
    'hysteresis': {
        'flex': 5,
        'press': 5,
        'imu': 1000
    }
}

# Constants
TWO_BYTE = 65535
BYTE = 255
ZERO_GYRIN = 32767.0
MAX_BEND = 2000
ZERO_ACCEL = TWO_BYTE / 4.0 - 680.0

# Default scales and keys
SCALES = {
    'major': [0, 2, 4, 5, 7, 9, 11],
    'minor': [0, 2, 3, 5, 7, 8, 10],
    'pentatonic': [0, 2, 4, 7, 9],
    'blues': [0, 3, 5, 6, 7, 10],
    'chromatic': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
}

# Note to MIDI number mapping (C3 = 60)
NOTES = {
    'C': 60,
    'C#': 61,
    'Db': 61,
    'D': 62,
    'D#': 63,
    'Eb': 63,
    'E': 64,
    'F': 65,
    'F#': 66,
    'Gb': 66,
    'G': 67,
    'G#': 68,
    'Ab': 68,
    'A': 69,
    'A#': 70,
    'Bb': 70,
    'B': 71
}

# Default sensor configuration
default_sensor_config = {
    'l': {'flex': True, 'press': False, 'imu': False},
    'r': {'flex': True, 'press': False, 'imu': False}
}

# Base chord configurations
CHORDS = {
    'major': [0, 4, 7],
    'minor': [0, 3, 7],
    'diminished': [0, 3, 6],
    'augmented': [0, 4, 8],
    'sus2': [0, 2, 7],
    'sus4': [0, 5, 7],
    'major7': [0, 4, 7, 11],
    'minor7': [0, 3, 7, 10],
    'dominant7': [0, 4, 7, 10]
}

# Base class for all configurations
class BaseConfig:
    def __init__(self):
        # Default values
        self.root_note = 'C'
        self.scale = 'major'
        self.trigger_sensors = {'l': 'flex', 'r': 'flex'}
        self.thresholds = sensor_defaults['thresholds']
        self.hysteresis = sensor_defaults['hysteresis']
        self.mod_sensors = {'r': [None], 'l': [None]}
        self.mod_idx = {'r': [None], 'l': [None]}
        self.hands = ['l', 'r']
        self.sensor_config = default_sensor_config
        
    def to_file(self, config_path: str) -> Dict[str, Any]:
        """Save configuration to a JSON file"""
        os.makedirs(os.path.dirname(os.path.abspath(config_path)), exist_ok=True)
        
        # Create a dictionary of all configuration attributes
        config_dict = {}
        for key, value in self.__dict__.items():
            if not key.startswith('_'):  # Skip private attributes
                config_dict[key] = value
                
        # Write the dictionary to a JSON file
        with open(config_path, 'w') as f:
            json.dump(config_dict, f, indent=4)
            
        return config_dict
    
    @classmethod
    def from_file(cls, config_path: str) -> 'BaseConfig':
        """Load configuration from a JSON file"""
        config = cls()
        
        with open(config_path, 'r') as f:
            config_dict = json.load(f)
            
        # Update configuration attributes from the dictionary
        for key, value in config_dict.items():
            if hasattr(config, key):
                setattr(config, key, value)
                
        return config 