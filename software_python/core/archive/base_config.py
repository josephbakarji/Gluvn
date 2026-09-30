"""
Base configuration for gluvn applications — defaults and common settings.
Adapted for M5Stick C Plus 1.1 + CD74HC4067 multiplexer + built-in IMU (MPU6886).
"""
import json
import os
from copy import deepcopy
from typing import Dict, Any

# Sensor thresholds/hysteresis: M5Stick 12-bit ADC values (4x the original 10-bit Arduino values)
sensor_defaults = {
    'thresholds': {
        'flex': 800,
        'press': 60,
        'imu': 16384,
    },
    'hysteresis': {
        'flex': 20,
        'press': 20,
        'imu': 500,
    }
}

TWO_BYTE = 65535
BYTE = 255

ZERO_ACCEL = 0.0   # M5Unified auto-calibrates / 0-centered

default_sensor_config = {
    'l': {'flex': True, 'press': True, 'imu': True},
    'r': {'flex': True, 'press': True, 'imu': True}
}

class BaseConfig:
    def __init__(self):
        self.root_note = 'C'
        self.scale = 'major'
        self.trigger_sensors = {'l': 'flex', 'r': 'flex'}
        # deepcopy: these instance attributes must not alias the shared
        # module-level dicts above, or mutating one BaseConfig's thresholds
        # (e.g. config.thresholds['flex'] = 900) would silently change the
        # default for every other instance and every other config that
        # never explicitly set its own value.
        self.thresholds = deepcopy(sensor_defaults['thresholds'])
        self.hysteresis = deepcopy(sensor_defaults['hysteresis'])
        self.mod_sensors = {'r': [None], 'l': [None]}
        self.mod_idx = {'r': [None], 'l': [None]}
        self.hands = ['l', 'r']
        self.sensor_config = deepcopy(default_sensor_config)

        self.mcu_target = "M5Stick_C_Plus_1.1"
        self.adc_resolution = 12
        self.imu_source = "Internal_M5Unified"
        self.mux_enabled = True

    def to_file(self, config_path: str) -> Dict[str, Any]:
        """Save configuration to a JSON file."""
        os.makedirs(os.path.dirname(os.path.abspath(config_path)), exist_ok=True)

        config_dict = {k: v for k, v in self.__dict__.items() if not k.startswith('_')}

        with open(config_path, 'w') as f:
            json.dump(config_dict, f, indent=4)

        return config_dict

    @classmethod
    def from_file(cls, config_path: str) -> 'BaseConfig':
        """Load configuration from a JSON file."""
        config = cls()

        with open(config_path, 'r') as f:
            config_dict = json.load(f)

        for key, value in config_dict.items():
            if hasattr(config, key):
                setattr(config, key, value)

        return config
