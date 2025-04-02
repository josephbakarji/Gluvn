"""
Moving Window configuration for gluvn applications
Allows for controlling notes based on finger positions with a moving window approach
"""

from configs.base_config import BaseConfig

class MovingWindowConfig(BaseConfig):
    def __init__(self):
        super().__init__()
        
        # Basic settings
        self.root_note = 'D'
        self.scale = 'minor'
        
        # Sensor configuration
        self.trigger_sensors = {'l': 'flex', 'r': 'flex'}
        self.mod_sensors = {'r': ['imu', 'imu', 'imu'], 'l': [None]}
        self.mod_idx = {'r': [3, 4, 5], 'l': None}  # [yaw, pitch, roll]
        
        # Controller settings
        self.volume_controller = 'accel_mag'  # Volume based on accelerometer magnitude
        self.pitch_bender = None
        self.averaging_window_controller = 'flex0'  # Left thumb controls averaging window
        
        # Window settings
        self.averaging_window_size = 10
        self.base_volume = 20
        
        # Instrument settings
        self.instrument = 'double_flex'  # Options: 'double_flex' or 'ten_finger'
        self.num_lh_fingers = 5
        self.num_rh_fingers = 5
        
        # Override specific thresholds if needed
        self.thresholds.update({
            'flex': 200,
            'press': 15
        })
        
        # Configure the sensor setup
        self._setup_sensor_config()
        
    def _setup_sensor_config(self):
        """Configure which sensors are enabled based on the app's needs"""
        self.sensor_config = {'l': {'flex': False, 'press': False, 'imu': False},
                             'r': {'flex': False, 'press': False, 'imu': False}}
        
        # Enable required sensors
        for hand in ['l', 'r']:
            # Enable trigger sensors
            if hand in self.trigger_sensors:
                sensor_type = self.trigger_sensors[hand]
                if sensor_type:
                    self.sensor_config[hand][sensor_type] = True
            
            # Enable modulation sensors
            if hand in self.mod_sensors:
                for sensor in self.mod_sensors[hand]:
                    if sensor is not None and sensor in self.sensor_config[hand]:
                        self.sensor_config[hand][sensor] = True


# Create a default config instance for import
config = MovingWindowConfig() 