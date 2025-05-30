"""
Sensor Processing Threads for GLUVN Visualization

This module provides threading components for real-time sensor data processing
that integrate with the GLUVN strategy system for proper trigger logic.

Author: Joseph Bakarji
"""

import sys
import os
import numpy as np
import queue
from PyQt5.QtCore import QThread, pyqtSignal

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from core.strategies.trigger_strategies import HysteresisTrigger


class SensorProcessingThread(QThread):
    """
    Thread for processing real sensor data using GLUVN trigger strategies
    
    This thread integrates with the hardware reader and uses the proper
    trigger strategy classes from core.strategies for consistent behavior
    across all GLUVN applications.
    """
    
    sensor_update = pyqtSignal(str, int, int, bool, int)  # hand, finger_idx, sensor_value, triggered, switch_event
    error_signal = pyqtSignal(str)
    
    def __init__(self, reader, trigger_strategy_config, sensor_type='flex'):
        super().__init__()
        self.reader = reader
        self.sensor_type = sensor_type
        self.running = False
        
        # Initialize trigger strategy using the proper GLUVN strategy system
        self.trigger_strategy = HysteresisTrigger(trigger_strategy_config)
        self.trigger_strategy.initialize()
        
        print(f"✅ Initialized trigger strategy with config: {trigger_strategy_config}")
    
    def run(self):
        """Main sensor processing loop"""
        try:
            if not self.reader:
                self.error_signal.emit("No hardware reader available")
                return
                
            self.reader.start_readers()
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
                            if data_count > 0:
                                print(f"📊 {hand.upper()}: {self.sensor_type} = {sensor_values[:5]} (processed {data_count} samples)")
                            
                            # Process each finger
                            for finger_idx in range(min(5, len(sensor_values))):
                                value = sensor_values[finger_idx]
                                switch_event = switch_events[finger_idx] if finger_idx < len(switch_events) else 0
                                currently_triggered = trigger_states[finger_idx] if finger_idx < len(trigger_states) else False
                                
                                # Debug switch events
                                if switch_event == 1:
                                    print(f"🔥 TRIGGER ON: {hand.upper()} finger {finger_idx}, value={value}")
                                elif switch_event == -1:
                                    print(f"⚪ TRIGGER OFF: {hand.upper()} finger {finger_idx}, value={value}")
                                
                                # Emit sensor update signal with switch event
                                self.sensor_update.emit(hand, finger_idx, int(value), currently_triggered, switch_event)
                    
                    # Small delay to prevent excessive CPU usage
                    self.msleep(50)  # 50ms = 20 FPS
                    
                except Exception as e:
                    self.error_signal.emit(f"Error processing sensors: {e}")
                    print(f"❌ Exception in sensor processing: {e}")
                    self.msleep(100)
                    
        except Exception as e:
            self.error_signal.emit(f"Failed to start sensor reader: {e}")
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
            print(f"🎛️ Updated {sensor_type} threshold to {threshold}")
    
    def update_hysteresis(self, sensor_type, hysteresis):
        """Update hysteresis value in the trigger strategy"""
        if hasattr(self.trigger_strategy, 'set_hysteresis'):
            self.trigger_strategy.set_hysteresis(sensor_type, hysteresis)
            print(f"🎛️ Updated {sensor_type} hysteresis to {hysteresis}") 