"""
Modulation Strategy Implementations for GLUVN System

This module contains concrete implementations of modulation strategies that convert
continuous sensor data into MIDI control messages for expressive musical control.

Strategies:
- AccelVolumeModulation: Volume control based on accelerometer magnitude
- IMUPitchBendModulation: Pitch bend control based on IMU orientation data
- MovingWindowModulation: Dynamic window averaging for smooth control
- ChoirModulation: Multi-voice control using IMU data

Author: Joseph Bakarji
"""

from .base_strategies import ModulationStrategy
import numpy as np
from typing import Dict, Any
from collections import deque
from itertools import repeat, islice


class AccelVolumeModulation(ModulationStrategy):
    """
    Volume control based on accelerometer magnitude
    
    This strategy implements the accelerometer-based volume control from the original
    app_10fig_accel.py, providing smooth volume changes based on hand movement.
    """
    
    def initialize(self):
        """Initialize accelerometer volume modulation parameters"""
        # Configuration parameters
        self.base_volume = self.config.get('base_volume', 20)
        self.max_volume = self.config.get('max_volume', 127)
        self.accel_sensors = self.config.get('accel_sensors', ['imu3', 'imu4', 'imu5'])  # x, y, z accel
        self.smoothing_window = self.config.get('smoothing_window', 10)
        self.scaling_factor = self.config.get('scaling_factor', 15000)
        
        # Constants from original code
        self.TWO_BYTE = 65535
        self.ZERO_ACCEL = self.TWO_BYTE / 4.0 - 680.0
        
        # Smoothing queues for each hand
        for hand in ['l', 'r']:
            self.modulation_state[hand] = {
                'averaging_queue': deque(repeat(0, self.smoothing_window), maxlen=self.smoothing_window),
                'current_volume': self.base_volume
            }
    
    def process_modulation(self, sensor_data: Dict[str, Any], hand: str) -> Dict[str, float]:
        """
        Process accelerometer data for volume control
        
        Args:
            sensor_data: Dictionary containing IMU sensor readings
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Dictionary with 'volume' and 'aftertouch' values
        """
        if hand not in self.modulation_state:
            return {}
        
        # Check if we have accelerometer data
        if not all(sensor in sensor_data for sensor in self.accel_sensors):
            return {'volume': self.modulation_state[hand]['current_volume']}
        
        # Calculate accelerometer magnitude
        if len(self.accel_sensors) == 3:
            # Traditional 3-axis accelerometer magnitude (original implementation)
            accel_x = sensor_data[self.accel_sensors[0]] - self.TWO_BYTE / 2.0
            accel_y = sensor_data[self.accel_sensors[1]] - self.TWO_BYTE / 2.0
            accel_z = sensor_data[self.accel_sensors[2]] - self.TWO_BYTE / 2.0
            
            norm = np.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
            volume_raw = max(norm - self.ZERO_ACCEL, 0)
        else:
            # Single sensor or custom configuration
            # Just use the raw sensor value with appropriate scaling
            sensor_value = sensor_data[self.accel_sensors[0]]
            if isinstance(sensor_value, (list, tuple)):
                sensor_value = sensor_value[0] if len(sensor_value) > 0 else 0
            
            # Scale single sensor value appropriately
            volume_raw = max(abs(sensor_value - self.TWO_BYTE / 2.0), 0)
        
        # Add to smoothing queue
        averaging_queue = self.modulation_state[hand]['averaging_queue']
        averaging_queue.append(volume_raw)
        
        # Calculate smoothed volume
        mean_volume = sum(averaging_queue) / len(averaging_queue)
        
        # Scale to MIDI range
        scaled_volume = self._input_scaling(
            mean_volume, 
            min_output=0, 
            max_output=self.max_volume - self.base_volume,
            max_input=self.scaling_factor
        )
        
        final_volume = self.base_volume + scaled_volume
        final_volume = max(0, min(127, int(final_volume)))
        
        # Update state
        self.modulation_state[hand]['current_volume'] = final_volume
        
        return {
            'volume': final_volume,
            'aftertouch': final_volume  # Use same value for aftertouch
        }
    
    def _input_scaling(self, input_val, min_output=0, max_output=127, shift=0, min_input=0, max_input=255):
        """Scale input value to output range (from original code)"""
        if max_input == min_input:
            return min_output
        
        output = min_output + (max_output - min_output) * ((input_val + shift) - min_input) / (max_input - min_input)
        return max(min_output, min(max_output, output))


class IMUPitchBendModulation(ModulationStrategy):
    """
    Pitch bend control based on IMU orientation data
    
    This strategy implements IMU-based pitch bending for expressive musical control.
    """
    
    def initialize(self):
        """Initialize IMU pitch bend modulation parameters"""
        # Configuration parameters
        self.pitch_bend_sensor = self.config.get('pitch_bend_sensor', 'imu2')
        self.pitch_bend_range = self.config.get('pitch_bend_range', 8192)  # Standard MIDI pitch bend range
        smoothing_factor = self.config.get('smoothing_factor', 0.5)  # Convert to window size
        self.smoothing_window_size = max(1, int(smoothing_factor * 10))  # Convert 0.1-2.0 to 1-20 window size
        self.modulation_mode = self.config.get('modulation_mode', 'modulo')  # 'modulo' or 'linear'
        
        # Constants
        self.TWO_BYTE = 65535
        
        # State for each hand
        for hand in ['l', 'r']:
            self.modulation_state[hand] = {
                'last_pitch_bend': 0,
                'pitch_bend_history': deque(maxlen=self.smoothing_window_size)
            }
    
    def process_modulation(self, sensor_data: Dict[str, Any], hand: str) -> Dict[str, float]:
        """
        Process IMU data for pitch bend control
        
        Args:
            sensor_data: Dictionary containing IMU sensor readings
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Dictionary with 'pitch_bend' value
        """
        if hand not in self.modulation_state:
            return {}
        
        # Check if we have pitch bend sensor data
        if self.pitch_bend_sensor not in sensor_data:
            return {'pitch_bend': self.modulation_state[hand]['last_pitch_bend']}
        
        # Get raw sensor value
        raw_value = sensor_data[self.pitch_bend_sensor]
        
        # Calculate pitch bend based on modulation mode
        if self.modulation_mode == 'modulo':
            pitch_bend = self._scaling_modulo(
                raw_value,
                min_output=-self.pitch_bend_range,
                max_output=self.pitch_bend_range,
                min_input=0,
                max_input=self.TWO_BYTE
            )
        else:  # linear mode
            pitch_bend = self._input_scaling(
                raw_value,
                min_output=-self.pitch_bend_range,
                max_output=self.pitch_bend_range,
                min_input=0,
                max_input=self.TWO_BYTE
            )
        
        # Apply smoothing
        history = self.modulation_state[hand]['pitch_bend_history']
        history.append(pitch_bend)
        
        if len(history) > 1:
            smoothed_pitch_bend = np.mean(list(history))
        else:
            smoothed_pitch_bend = pitch_bend
        
        # Update state
        self.modulation_state[hand]['last_pitch_bend'] = int(smoothed_pitch_bend)
        
        return {
            'pitch_bend': int(smoothed_pitch_bend)
        }
    
    def _input_scaling(self, input_val, min_output=0, max_output=127, shift=0, min_input=0, max_input=255):
        """Scale input value to output range"""
        if max_input == min_input:
            return min_output
        
        output = min_output + (max_output - min_output) * ((input_val + shift) - min_input) / (max_input - min_input)
        return max(min_output, min(max_output, output))
    
    def _scaling_modulo(self, input_val, min_output=0, max_output=127, min_input=0, max_input=255):
        """Modulo-based scaling (from original code)"""
        if max_input == min_input:
            return min_output
        
        scaled = max_output + (max_output - min_output) * (input_val - min_input) / (max_input - min_input)
        modulo_result = (scaled % (max_output - min_output)) - max_output
        return int(modulo_result)


class MovingWindowModulation(ModulationStrategy):
    """
    Dynamic window averaging control for sophisticated parameter adjustment
    
    This strategy allows IMU data to control the averaging window size for
    other modulation strategies, enabling dynamic response characteristics.
    """
    
    def initialize(self):
        """Initialize moving window modulation parameters"""
        # Configuration parameters
        self.window_controller = self.config.get('window_controller', 'imu1')
        self.min_window_size = self.config.get('min_window_size', 1)
        self.max_window_size = self.config.get('max_window_size', 50)
        self.control_parameter = self.config.get('control_parameter', 'averaging_window')
        
        # Constants
        self.TWO_BYTE = 65535
        
        # State for each hand
        for hand in ['l', 'r']:
            self.modulation_state[hand] = {
                'current_window_size': self.max_window_size // 2,
                'window_history': deque(maxlen=10)
            }
    
    def process_modulation(self, sensor_data: Dict[str, Any], hand: str) -> Dict[str, float]:
        """
        Process IMU data for window size control
        
        Args:
            sensor_data: Dictionary containing IMU sensor readings
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Dictionary with window control parameters
        """
        if hand not in self.modulation_state:
            return {}
        
        # Check if we have window controller data
        if self.window_controller not in sensor_data:
            return {self.control_parameter: self.modulation_state[hand]['current_window_size']}
        
        # Get raw sensor value and scale to window size range
        raw_value = sensor_data[self.window_controller]
        window_size = self._input_scaling(
            raw_value,
            min_output=self.min_window_size,
            max_output=self.max_window_size,
            min_input=0,
            max_input=self.TWO_BYTE
        )
        
        # Update state
        self.modulation_state[hand]['current_window_size'] = int(window_size)
        
        return {
            self.control_parameter: int(window_size)
        }
    
    def _input_scaling(self, input_val, min_output=0, max_output=127, shift=0, min_input=0, max_input=255):
        """Scale input value to output range"""
        if max_input == min_input:
            return min_output
        
        output = min_output + (max_output - min_output) * ((input_val + shift) - min_input) / (max_input - min_input)
        return max(min_output, min(max_output, output))


class ChoirModulation(ModulationStrategy):
    """
    Multi-voice control using IMU data for choir-like effects
    
    This strategy enables complex multi-voice control where different IMU
    sensors control different aspects of a choir-like arrangement.
    """
    
    def initialize(self):
        """Initialize choir modulation parameters"""
        # Configuration parameters
        self.voice_controllers = self.config.get('voice_controllers', {
            'voice1_volume': 'imu1',
            'voice2_volume': 'imu2', 
            'voice3_volume': 'imu3',
            'master_pitch': 'imu4'
        })
        self.voice_ranges = self.config.get('voice_ranges', {
            'voice1_volume': (0, 127),
            'voice2_volume': (0, 127),
            'voice3_volume': (0, 127),
            'master_pitch': (-8192, 8192)
        })
        self.smoothing_windows = self.config.get('smoothing_windows', {
            'voice1_volume': 5,
            'voice2_volume': 5,
            'voice3_volume': 5,
            'master_pitch': 3
        })
        
        # Constants
        self.TWO_BYTE = 65535
        
        # State for each hand
        for hand in ['l', 'r']:
            self.modulation_state[hand] = {}
            for param in self.voice_controllers:
                window_size = self.smoothing_windows.get(param, 5)
                self.modulation_state[hand][param] = {
                    'history': deque(maxlen=window_size),
                    'current_value': 0
                }
    
    def process_modulation(self, sensor_data: Dict[str, Any], hand: str) -> Dict[str, float]:
        """
        Process IMU data for choir control
        
        Args:
            sensor_data: Dictionary containing IMU sensor readings
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Dictionary with choir control parameters
        """
        if hand not in self.modulation_state:
            return {}
        
        results = {}
        
        for param_name, sensor_name in self.voice_controllers.items():
            if sensor_name not in sensor_data:
                # Return last known value
                results[param_name] = self.modulation_state[hand][param_name]['current_value']
                continue
            
            # Get raw sensor value
            raw_value = sensor_data[sensor_name]
            
            # Get scaling range for this parameter
            min_output, max_output = self.voice_ranges.get(param_name, (0, 127))
            
            # Scale the value
            scaled_value = self._input_scaling(
                raw_value,
                min_output=min_output,
                max_output=max_output,
                min_input=0,
                max_input=self.TWO_BYTE
            )
            
            # Apply smoothing
            history = self.modulation_state[hand][param_name]['history']
            history.append(scaled_value)
            
            if len(history) > 1:
                smoothed_value = np.mean(list(history))
            else:
                smoothed_value = scaled_value
            
            # Update state and results
            self.modulation_state[hand][param_name]['current_value'] = int(smoothed_value)
            results[param_name] = int(smoothed_value)
        
        return results
    
    def _input_scaling(self, input_val, min_output=0, max_output=127, shift=0, min_input=0, max_input=255):
        """Scale input value to output range"""
        if max_input == min_input:
            return min_output
        
        output = min_output + (max_output - min_output) * ((input_val + shift) - min_input) / (max_input - min_input)
        return max(min_output, min(max_output, output))


class CompositeModulation(ModulationStrategy):
    """
    Composite modulation strategy that combines multiple modulation strategies
    
    This allows for complex modulation behaviors by combining different
    modulation strategies and their outputs.
    """
    
    def __init__(self, modulation_strategies: list, config: Dict[str, Any]):
        """
        Initialize composite modulation strategy
        
        Args:
            modulation_strategies: List of ModulationStrategy instances
            config: Configuration dictionary
        """
        self.modulation_strategies = modulation_strategies
        super().__init__(config)
    
    def initialize(self):
        """Initialize composite modulation parameters"""
        # How to combine results: 'merge', 'priority', 'weighted'
        self.combination_mode = self.config.get('combination_mode', 'merge')
        self.strategy_weights = self.config.get('strategy_weights', {})
        
        # Initialize state for each hand
        for hand in ['l', 'r']:
            self.modulation_state[hand] = {
                'combined_results': {},
                'strategy_outputs': {}
            }
    
    def process_modulation(self, sensor_data: Dict[str, Any], hand: str) -> Dict[str, float]:
        """
        Process sensor data through all sub-strategies and combine results
        
        Args:
            sensor_data: Dictionary containing sensor readings
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Dictionary with combined modulation values
        """
        if hand not in self.modulation_state:
            return {}
        
        # Process each strategy
        strategy_results = []
        for i, strategy in enumerate(self.modulation_strategies):
            result = strategy.process_modulation(sensor_data, hand)
            strategy_results.append(result)
            self.modulation_state[hand]['strategy_outputs'][f'strategy_{i}'] = result
        
        # Combine results based on combination mode
        if self.combination_mode == 'merge':
            combined = self._merge_results(strategy_results)
        elif self.combination_mode == 'weighted':
            combined = self._weighted_combine(strategy_results)
        else:  # priority mode - later strategies override earlier ones
            combined = self._priority_combine(strategy_results)
        
        # Update state
        self.modulation_state[hand]['combined_results'] = combined
        
        return combined
    
    def _merge_results(self, results: list) -> Dict[str, float]:
        """Merge results by taking the latest value for each parameter"""
        combined = {}
        for result in results:
            combined.update(result)
        return combined
    
    def _weighted_combine(self, results: list) -> Dict[str, float]:
        """Combine results using weighted averaging"""
        combined = {}
        param_counts = {}
        
        for i, result in enumerate(results):
            weight = self.strategy_weights.get(f'strategy_{i}', 1.0)
            for param, value in result.items():
                if param not in combined:
                    combined[param] = 0
                    param_counts[param] = 0
                combined[param] += value * weight
                param_counts[param] += weight
        
        # Normalize by total weights
        for param in combined:
            if param_counts[param] > 0:
                combined[param] /= param_counts[param]
        
        return combined
    
    def _priority_combine(self, results: list) -> Dict[str, float]:
        """Combine results with priority (later strategies override)"""
        combined = {}
        for result in results:
            combined.update(result)  # Later results override earlier ones
        return combined 