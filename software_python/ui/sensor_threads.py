"""
Sensor Processing Threads for GLUVN Visualization

This module provides threading components for real-time sensor data processing
that integrate with the GLUVN strategy system for proper trigger logic and
modulation control.

Author: Joseph Bakarji
"""

import sys
import os
import numpy as np
import queue
from PyQt5.QtCore import QThread, pyqtSignal

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from strategies.trigger_strategies import HysteresisTrigger


class SensorProcessingThread(QThread):
    """
    Thread for processing real sensor data using GLUVN trigger strategies
    
    This thread integrates with the hardware reader and uses the proper
    trigger strategy classes from core.strategies for consistent behavior
    across all GLUVN applications.
    """
    
    sensor_update = pyqtSignal(str, int, int, bool, int)  # hand, finger_idx, sensor_value, triggered, switch_event
    modulation_update = pyqtSignal(str, dict)  # hand, modulation_dict
    raw_sensor_update = pyqtSignal(str, dict)  # hand, raw_sensor_dict (for quantification displays)
    error_signal = pyqtSignal(str)
    
    def __init__(self, reader, trigger_strategy_config, sensor_type='flex', modulation_strategies=None, debug_printing=False):
        super().__init__()
        self.reader = reader
        self.sensor_type = sensor_type
        self.running = False
        self.debug_printing = debug_printing
        
        # Initialize trigger strategy using the proper GLUVN strategy system
        self.trigger_strategy = HysteresisTrigger(trigger_strategy_config)
        self.trigger_strategy.initialize()
        
        # Initialize modulation strategies if provided
        self.modulation_strategies = modulation_strategies or []
        if self.modulation_strategies:
            for strategy in self.modulation_strategies:
                strategy.initialize()
        
        if self.debug_printing:
            print(f"✅ Initialized trigger strategy with config: {trigger_strategy_config}")
            if self.modulation_strategies:
                print(f"✅ Initialized {len(self.modulation_strategies)} modulation strategies")
    
    def run(self):
        """Main sensor processing loop"""
        try:
            if not self.reader:
                self.error_signal.emit("No hardware reader available")
                return
                
            self.reader.start_readers()
            if self.debug_printing:
                print("🔧 Sensor processing started")
                
                # Debug: Print available hands and threads
                print(f"📡 Available reader threads: {list(self.reader.threads.keys())}")
                for hand in self.reader.threads:
                    print(f"👋 Hand {hand} threads: {list(self.reader.threads[hand].keys())}")
            
            while self.running:
                try:
                    # Process each hand
                    for hand in ['l', 'r']:
                        if hand not in self.reader.threads or 'parser' not in self.reader.threads[hand]:
                            continue
                        
                        parser_queue = self.reader.threads[hand]['parser'].getQ()
                        
                        # Get latest data by draining the queue
                        latest_data = None
                        data_count = 0
                        
                        while True:
                            try:
                                sensor_data = parser_queue.get_nowait()
                                latest_data = sensor_data
                                data_count += 1
                            except queue.Empty:
                                break
                        
                        # Process the latest sensor data if we got any
                        if latest_data and self.sensor_type in latest_data:
                            # Use the proper trigger strategy for processing
                            switch_events = self.trigger_strategy.process_triggers(latest_data, hand)
                            trigger_states = self.trigger_strategy.get_trigger_state(hand)
                            sensor_values = latest_data[self.sensor_type]
                            
                            # Debug: Print what data we received (less frequently)
                            if data_count > 0 and self.debug_printing:
                                print(f"📊 {hand.upper()}: {self.sensor_type} = {sensor_values[:5]} (processed {data_count} samples)")
                                print(f"🔍 {hand.upper()} sensor data keys: {list(latest_data.keys())}")
                                if 'imu' in latest_data:
                                    print(f"🧭 {hand.upper()} IMU array: {latest_data['imu']}")
                            
                            # Process each finger for trigger events
                            for finger_idx in range(min(5, len(sensor_values))):
                                value = sensor_values[finger_idx]
                                switch_event = switch_events[finger_idx] if finger_idx < len(switch_events) else 0
                                currently_triggered = trigger_states[finger_idx] if finger_idx < len(trigger_states) else False
                                
                                # Debug switch events
                                if switch_event == 1 and self.debug_printing:
                                    print(f"🔥 TRIGGER ON: {hand.upper()} finger {finger_idx}, value={value}")
                                elif switch_event == -1 and self.debug_printing:
                                    print(f"⚪ TRIGGER OFF: {hand.upper()} finger {finger_idx}, value={value}")
                                
                                # Emit sensor update signal with switch event
                                self.sensor_update.emit(hand, finger_idx, int(value), currently_triggered, switch_event)
                            
                            # Emit raw sensor data for quantification displays
                            self.raw_sensor_update.emit(hand, latest_data)
                            
                            # Process modulation strategies if available
                            if self.modulation_strategies:
                                # Transform sensor data for modulation strategies
                                # Convert IMU array to individual sensor keys
                                transformed_data = self._transform_sensor_data(latest_data)
                                
                                # Debug: Show transformed data occasionally
                                if data_count > 0 and self.debug_printing:
                                    print(f"🔧 {hand.upper()} transformed data keys: {list(transformed_data.keys())}")
                                    if any(key.startswith('imu') for key in transformed_data.keys()):
                                        imu_values = {k: v for k, v in transformed_data.items() if k.startswith('imu')}
                                        print(f"🧭 {hand.upper()} transformed IMU: {imu_values}")
                                
                                combined_modulation = {}
                                
                                for strategy in self.modulation_strategies:
                                    modulation_result = strategy.process_modulation(transformed_data, hand)
                                    combined_modulation.update(modulation_result)
                                
                                # Emit modulation update signal
                                if combined_modulation:
                                    self.modulation_update.emit(hand, combined_modulation)
                                    
                                    # Debug modulation values (less frequently)
                                    if data_count > 0 and any(combined_modulation.values()) and self.debug_printing:
                                        print(f"🎛️ {hand.upper()} modulation: {combined_modulation}")
                    
                    # Small delay to prevent excessive CPU usage
                    self.msleep(50)  # 50ms = 20 FPS
                    
                except Exception as e:
                    self.error_signal.emit(f"Error processing sensors: {e}")
                    if self.debug_printing:
                        print(f"❌ Exception in sensor processing: {e}")
                    self.msleep(100)
                    
        except Exception as e:
            self.error_signal.emit(f"Failed to start sensor reader: {e}")
            if self.debug_printing:
                print(f"❌ Exception starting sensor reader: {e}")
        finally:
            if self.reader:
                try:
                    self.reader.stop_readers()
                except:
                    pass
    
    def start_processing(self):
        """Start the processing thread"""
        self.running = True
        self.start()
    
    def stop_processing(self):
        """Stop the processing thread"""
        self.running = False
        if self.reader:
            try:
                self.reader.stop_readers()
            except:
                pass
        self.wait(2000)
    
    def update_threshold(self, sensor_type, threshold):
        """Update threshold value in the trigger strategy"""
        if hasattr(self.trigger_strategy, 'set_threshold'):
            self.trigger_strategy.set_threshold(sensor_type, threshold)
            if self.debug_printing:
                print(f"🎛️ Updated {sensor_type} threshold to {threshold}")
    
    def update_hysteresis(self, sensor_type, hysteresis):
        """Update hysteresis value in the trigger strategy"""
        if hasattr(self.trigger_strategy, 'set_hysteresis'):
            self.trigger_strategy.set_hysteresis(sensor_type, hysteresis)
            if self.debug_printing:
                print(f"🎛️ Updated {sensor_type} hysteresis to {hysteresis}")
    
    def add_modulation_strategy(self, strategy):
        """Add a modulation strategy at runtime"""
        if strategy not in self.modulation_strategies:
            strategy.initialize()
            self.modulation_strategies.append(strategy)
            if self.debug_printing:
                print(f"🎛️ Added modulation strategy: {strategy.__class__.__name__}")
    
    def remove_modulation_strategy(self, strategy_class):
        """Remove a modulation strategy by class type"""
        self.modulation_strategies = [s for s in self.modulation_strategies if not isinstance(s, strategy_class)]
        if self.debug_printing:
            print(f"🎛️ Removed modulation strategy: {strategy_class.__name__}")
    
    def get_modulation_state(self, hand):
        """Get current modulation state for debugging"""
        states = {}
        for i, strategy in enumerate(self.modulation_strategies):
            strategy_name = strategy.__class__.__name__
            states[f"{strategy_name}_{i}"] = strategy.get_modulation_state(hand)
        return states

    def _transform_sensor_data(self, data):
        """Transform sensor data for modulation strategies"""
        transformed_data = data.copy()
        
        # Handle IMU data conversion - based on volume_pitch_control_gui.py
        # Convert IMU array [yaw, pitch, roll, gx, gy, gz] to individual sensor keys
        if 'imu' in data and isinstance(data['imu'], (list, tuple)):
            imu_array = data['imu']
            if len(imu_array) >= 6:
                # Map array indices to IMU sensor names based on original app structure
                # Based on data_analysis.py: [yaw, pitch, roll, gx, gy, gz] = indices [0, 1, 2, 3, 4, 5]
                # For original app compatibility: imu1=yaw, imu2=pitch, imu3=gx, imu4=gy, imu5=gz
                transformed_data['imu1'] = int(imu_array[0])  # yaw
                transformed_data['imu2'] = int(imu_array[1])  # pitch
                transformed_data['imu3'] = int(imu_array[3])  # gx (accelerometer X)
                transformed_data['imu4'] = int(imu_array[4])  # gy (accelerometer Y)
                transformed_data['imu5'] = int(imu_array[5])  # gz (accelerometer Z)
        
        return transformed_data 