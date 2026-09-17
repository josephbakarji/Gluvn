"""
Trigger Strategy Implementations for GLUVN System

This module contains concrete implementations of trigger strategies that convert
raw sensor data into discrete trigger events for note on/off control.

Strategies:
- HysteresisTrigger: Hysteresis-based triggering with configurable thresholds
- MultiSensorTrigger: Trigger based on multiple sensor fusion
- IMUDirectionalTrigger: IMU-based directional triggering

Author: Joseph Bakarji
Updated: Jul 2026 - Helene Jabbour
"""

from .base_strategies import TriggerStrategy
import numpy as np
from typing import Dict, Any


class HysteresisTrigger(TriggerStrategy):
    """
    Hysteresis-based triggering with configurable thresholds
    
    This strategy implements the core trigger logic from the original SensorProcess
    class, providing stable triggering with hysteresis to prevent false triggers
    from sensor noise.
    """
    
    def initialize(self):
        """Initialize hysteresis trigger parameters and state"""
        # Get configuration parameters
        self.thresholds = self.config.get('thresholds', {'flex': 200, 'press': 15})
        self.hysteresis = self.config.get('hysteresis', {'flex': 5, 'press': 5})
        self.trigger_sensors = self.config.get('trigger_sensors', {'l': 'flex', 'r': 'flex'})
        
        # Initialize state for both hands
        for hand in ['l', 'r']:
            self.trigger_states[hand] = np.zeros(5, dtype=bool)
            
        # Additional state tracking for hysteresis logic
        self.trig_on = {'l': np.zeros(5, dtype=bool), 'r': np.zeros(5, dtype=bool)}
        self.trig_off = {'l': np.zeros(5, dtype=bool), 'r': np.zeros(5, dtype=bool)}
    
    def process_triggers(self, sensor_data: Dict[str, Any], hand: str) -> np.ndarray:
        """
        Implement hysteresis-based triggering logic
        
        This is a direct port of the trigger_logic method from the original
        SensorProcess class, adapted to work with the new strategy interface.
        
        Args:
            sensor_data: Dictionary containing sensor readings
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Array of trigger events: 1 for note on, -1 for note off, 0 for no change
        """
        # Get the trigger sensor type for this hand
        if hand not in self.trigger_sensors:
            return np.zeros(5, dtype=int)
            
        trigger_sensor = self.trigger_sensors[hand]
        
        # Check if sensor data is available
        if trigger_sensor not in sensor_data:
            return np.zeros(5, dtype=int)
            
        # Get sensor values and convert to numpy array
        sensor_values = sensor_data[trigger_sensor]
        sensarr = np.asarray(sensor_values)
        
        # Ensure we have exactly 5 values (one per finger)
        if len(sensarr) != 5:
            return np.zeros(5, dtype=int)
        
        # Get threshold and hysteresis values for this sensor type
        threshold = self.thresholds.get(trigger_sensor, 200)
        hysteresis_val = self.hysteresis.get(trigger_sensor, 5)
        
        # Get previous trigger states
        trigon_prev = self.trig_on[hand].copy()
        trigoff_prev = self.trig_off[hand].copy()
        
        # Calculate trigger conditions
        sdiff = sensarr - threshold  # subtract threshold from readings
        trigon = sdiff - hysteresis_val > 0
        trigoff = sdiff + hysteresis_val < 0
        
        # Determine turn on/off events
        turnon = np.logical_and(
            np.logical_and(trigon, np.logical_not(trigon_prev)), 
            np.logical_not(self.trigger_states[hand])
        )
        turnoff = np.logical_and(
            np.logical_and(trigoff, np.logical_not(trigoff_prev)), 
            self.trigger_states[hand]
        )
        
        # Calculate switch events: 1 for turn on, -1 for turn off, 0 for no change
        n_switch = turnon.astype(int) - turnoff.astype(int)
        
        # Update trigger state
        self.trigger_states[hand] = n_switch + self.trigger_states[hand]
        
        # Update trigger on/off states for next iteration
        self.trig_on[hand] = trigon
        self.trig_off[hand] = trigoff
        
        return n_switch
    
    def set_threshold(self, sensor_type: str, threshold: float):
        """
        Update threshold for a specific sensor type
        
        Args:
            sensor_type: Type of sensor ('flex', 'press', etc.)
            threshold: New threshold value
        """
        self.thresholds[sensor_type] = threshold
    
    def set_hysteresis(self, sensor_type: str, hysteresis: float):
        """
        Update hysteresis for a specific sensor type
        
        Args:
            sensor_type: Type of sensor ('flex', 'press', etc.)
            hysteresis: New hysteresis value
        """
        self.hysteresis[sensor_type] = hysteresis
    
    def get_sensor_readings_above_threshold(self, sensor_data: Dict[str, Any], hand: str) -> np.ndarray:
        """
        Get which sensors are currently above threshold (for debugging)
        
        Args:
            sensor_data: Dictionary containing sensor readings
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Boolean array indicating which sensors are above threshold
        """
        if hand not in self.trigger_sensors:
            return np.zeros(5, dtype=bool)
            
        trigger_sensor = self.trigger_sensors[hand]
        if trigger_sensor not in sensor_data:
            return np.zeros(5, dtype=bool)
            
        sensor_values = np.asarray(sensor_data[trigger_sensor])
        threshold = self.thresholds.get(trigger_sensor, 200)
        
        return sensor_values > threshold


class MultiSensorTrigger(TriggerStrategy):
    """
    Trigger based on multiple sensor fusion
    
    This strategy can combine multiple sensor types (e.g., flex + pressure)
    to create more sophisticated triggering conditions.
    """
    
    def initialize(self):
        """Initialize multi-sensor trigger parameters"""
        # Configuration for multiple sensors per hand
        self.sensor_configs = self.config.get('sensor_configs', {
            'l': [{'type': 'flex', 'weight': 1.0}],
            'r': [{'type': 'flex', 'weight': 1.0}]
        })
        
        self.fusion_method = self.config.get('fusion_method', 'weighted_sum')  # 'weighted_sum', 'logical_and', 'logical_or'
        self.thresholds = self.config.get('thresholds', {'combined': 200})
        self.hysteresis = self.config.get('hysteresis', {'combined': 5})
        
        # Initialize state
        for hand in ['l', 'r']:
            self.trigger_states[hand] = np.zeros(5, dtype=bool)
        
        self.trig_on = {'l': np.zeros(5, dtype=bool), 'r': np.zeros(5, dtype=bool)}
        self.trig_off = {'l': np.zeros(5, dtype=bool), 'r': np.zeros(5, dtype=bool)}
    
    def process_triggers(self, sensor_data: Dict[str, Any], hand: str) -> np.ndarray:
        """
        Process multiple sensors and fuse them for triggering
        
        Args:
            sensor_data: Dictionary containing sensor readings
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Array of trigger events
        """
        if hand not in self.sensor_configs:
            return np.zeros(5, dtype=int)
        
        # Collect sensor values for fusion
        sensor_values = []
        weights = []
        
        for sensor_config in self.sensor_configs[hand]:
            sensor_type = sensor_config['type']
            weight = sensor_config.get('weight', 1.0)
            
            if sensor_type in sensor_data:
                values = np.asarray(sensor_data[sensor_type])
                if len(values) == 5:
                    sensor_values.append(values)
                    weights.append(weight)
        
        if not sensor_values:
            return np.zeros(5, dtype=int)
        
        # Fuse sensor values
        if self.fusion_method == 'weighted_sum':
            fused_values = np.zeros(5)
            total_weight = sum(weights)
            for values, weight in zip(sensor_values, weights):
                fused_values += values * (weight / total_weight)
        elif self.fusion_method == 'logical_and':
            # All sensors must be above their individual thresholds
            fused_values = np.ones(5) * 1000  # High value if all conditions met
            for values in sensor_values:
                # Convert to boolean based on individual thresholds, then back to values
                threshold = self.thresholds.get('combined', 200)
                above_threshold = values > threshold
                fused_values = fused_values * above_threshold.astype(float) * threshold
        elif self.fusion_method == 'logical_or':
            # Any sensor above threshold triggers
            fused_values = np.zeros(5)
            for values in sensor_values:
                fused_values = np.maximum(fused_values, values)
        else:
            # Default to first sensor
            fused_values = sensor_values[0]
        
        # Apply hysteresis logic to fused values
        threshold = self.thresholds.get('combined', 200)
        hysteresis_val = self.hysteresis.get('combined', 5)
        
        trigon_prev = self.trig_on[hand].copy()
        trigoff_prev = self.trig_off[hand].copy()
        
        sdiff = fused_values - threshold
        trigon = sdiff - hysteresis_val > 0
        trigoff = sdiff + hysteresis_val < 0
        
        turnon = np.logical_and(
            np.logical_and(trigon, np.logical_not(trigon_prev)), 
            np.logical_not(self.trigger_states[hand])
        )
        turnoff = np.logical_and(
            np.logical_and(trigoff, np.logical_not(trigoff_prev)), 
            self.trigger_states[hand]
        )
        
        n_switch = turnon.astype(int) - turnoff.astype(int)
        self.trigger_states[hand] = n_switch + self.trigger_states[hand]
        self.trig_on[hand] = trigon
        self.trig_off[hand] = trigoff
        
        return n_switch


class IMUDirectionalTrigger(TriggerStrategy):
    """
    IMU-based directional triggering (Jacob Choir style)
    
    This strategy uses IMU orientation data to trigger notes based on
    hand direction and movement, as implemented in the Jacob Choir application.
    """
    
    def initialize(self):
        """Initialize IMU directional trigger parameters"""
        # IMU-based triggering parameters
        self.accel_trigger_thresh = self.config.get('accel_trigger_thresh', 110)
        self.accel_trigger_hysteresis = self.config.get('accel_trigger_hysteresis', 10)
        self.pitch_trigger_hysteresis = self.config.get('pitch_trigger_hysteresis', 5)
        self.yaw_window = self.config.get('yaw_window', 10)
        
        # Initialize state tracking
        for hand in ['l', 'r']:
            self.trigger_states[hand] = np.zeros(5, dtype=bool)
        
        # IMU-specific state
        self.accel_trig_on = {'r': False, 'l': False}
        self.accel_trig_off = {'r': False, 'l': False}
        self.accel_turn_state = {'r': False, 'l': False}
        
        self.pitch_trig_on = {'r': False, 'l': False}
        self.pitch_trig_off = {'r': False, 'l': False}
        self.pitch_turn_state = {'r': False, 'l': False}
        self.pitch_trigger_thresh = {'r': 0, 'l': 0}
        
        # Reference orientations (will be calibrated)
        self.yaw0 = {'r': 0, 'l': 0}
        self.pitch0 = {'r': 0, 'l': 0}
        self.roll0 = {'r': 0, 'l': 0}
        self.yaw_trigger_thresh = {'r': [0, 0], 'l': [0, 0]}
        
        self.calibrated = False
    
    def process_triggers(self, sensor_data: Dict[str, Any], hand: str) -> np.ndarray:
        """
        Process IMU data for directional triggering
        
        Args:
            sensor_data: Dictionary containing sensor readings including IMU data
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Array of trigger events based on IMU orientation and movement
        """
        # This is a simplified version - full implementation would need
        # the complete IMU processing logic from app_jacob_choir.py
        
        if 'imu' not in sensor_data:
            return np.zeros(5, dtype=int)
        
        # For now, return basic trigger based on acceleration magnitude
        # Full implementation would include the complex orientation tracking
        # and directional note triggering from the Jacob Choir app
        
        imu_data = sensor_data['imu']
        if len(imu_data) < 6:  # Need at least 6 IMU values
            return np.zeros(5, dtype=int)
        
        # Simple acceleration-based triggering as placeholder
        accel_x, accel_y, accel_z = imu_data[3:6]
        accel_magnitude = np.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        
        # Trigger if acceleration exceeds threshold
        if accel_magnitude > self.accel_trigger_thresh:
            # Trigger first finger for demonstration
            trigger_events = np.zeros(5, dtype=int)
            if not self.trigger_states[hand][0]:  # If not already triggered
                trigger_events[0] = 1
                self.trigger_states[hand][0] = True
            return trigger_events
        else:
            # Turn off if below threshold
            trigger_events = np.zeros(5, dtype=int)
            if self.trigger_states[hand][0]:  # If currently triggered
                trigger_events[0] = -1
                self.trigger_states[hand][0] = False
            return trigger_events
    
    def calibrate_reference_orientation(self, sensor_data_list: list):
        """
        Calibrate reference orientation from a series of sensor readings
        
        Args:
            sensor_data_list: List of sensor data dictionaries for calibration
        """
        # Implementation would calculate reference orientations
        # from multiple sensor readings, similar to the initialization
        # phase in app_jacob_choir.py
        pass 