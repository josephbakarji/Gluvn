#!/usr/bin/env python3
"""
Ten Finger Demo Application (PyQt5 Version - No Hardware Required)

This is a PyQt5 demo version of the ten finger GUI that simulates sensor data
for testing the interface and functionality without requiring actual hardware.

Features:
- Simulated flex sensor data with realistic patterns
- Click-to-trigger buttons for manual testing
- All GUI features functional
- MIDI output still works (if MIDI device available)
- PyQt5 interface matching existing sensor plotting applications

Author: Joseph Bakarji
Usage: python ten_finger_demo_pyqt.py
"""

import sys
import os
import numpy as np
import threading
import queue
import time
import random

# PyQt5 imports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGridLayout, QLabel, QPushButton, 
                            QGroupBox, QSpinBox, QComboBox, QFrame, QStatusBar,
                            QMessageBox, QSizePolicy)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QPalette, QColor, QPaintEvent, QPainter, QBrush

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import GLUVN components
from core.strategies.trigger_strategies import HysteresisTrigger
from core.strategies.mapping_strategies import BasicMapper
from mapper import NoteMapper
from midi_writer import MidiWriter


class MockSensorData:
    """
    Mock sensor data generator for demo purposes
    
    This class simulates realistic flex sensor data patterns including
    noise, drift, and finger movement patterns.
    """
    
    def __init__(self):
        """Initialize mock sensor data generator"""
        # Base sensor values (at rest)
        self.base_values = {
            'l': [100, 110, 105, 115, 120],  # Left hand base values
            'r': [95, 108, 112, 98, 125]     # Right hand base values
        }
        
        # Current trigger states for simulation
        self.trigger_states = {
            'l': [False] * 5,
            'r': [False] * 5
        }
        
        # Animation parameters
        self.time_offset = 0
        self.noise_amplitude = 5  # Sensor noise level
        
    def get_sensor_data(self, hand: str) -> dict:
        """
        Generate simulated sensor data for a hand
        
        Args:
            hand: Hand identifier ('l' or 'r')
            
        Returns:
            Dictionary with simulated sensor data
        """
        # Generate base flex values with some drift and noise
        flex_values = []
        for i, base_val in enumerate(self.base_values[hand]):
            # Add slow drift
            drift = 10 * np.sin(self.time_offset * 0.1 + i)
            
            # Add noise
            noise = random.uniform(-self.noise_amplitude, self.noise_amplitude)
            
            # Add trigger simulation (if finger is "pressed")
            if self.trigger_states[hand][i]:
                trigger_value = 300 + random.uniform(-20, 20)  # High value when triggered
            else:
                trigger_value = 0
            
            final_value = base_val + drift + noise + trigger_value
            flex_values.append(max(0, min(1023, final_value)))  # Clamp to sensor range
        
        self.time_offset += 1
        
        return {
            'flex': flex_values,
            'press': [0] * 5,  # No pressure simulation in this demo
            'imu': [512] * 6   # Neutral IMU values
        }
    
    def set_finger_trigger(self, hand: str, finger: int, triggered: bool):
        """
        Manually set a finger trigger state for testing
        
        Args:
            hand: Hand identifier ('l' or 'r')
            finger: Finger index (0-4)
            triggered: Whether finger should be triggered
        """
        if hand in self.trigger_states and 0 <= finger < 5:
            self.trigger_states[hand][finger] = triggered


class FingerLight(QWidget):
    """
    PyQt5 Visual representation of a single finger trigger state with click interaction
    """
    
    def __init__(self, finger_name, note_name, click_callback=None):
        """
        Initialize a finger light widget
        
        Args:
            finger_name: Name of the finger (e.g., "Thumb", "Index")
            note_name: MIDI note name for this finger
            click_callback: Function to call when light is clicked
        """
        super().__init__()
        self.finger_name = finger_name
        self.note_name = note_name
        self.is_active = False
        self.click_callback = click_callback
        
        # Set up the widget
        self.setFixedSize(80, 100)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        
        # Create layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(2)
        
        # Create labels
        self.finger_label = QLabel(finger_name)
        self.finger_label.setAlignment(Qt.AlignCenter)
        self.finger_label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(self.finger_label)
        
        # Light area (will be drawn in paintEvent)
        self.light_area = QWidget()
        self.light_area.setFixedSize(60, 60)
        layout.addWidget(self.light_area, 0, Qt.AlignCenter)
        
        self.note_label = QLabel(note_name)
        self.note_label.setAlignment(Qt.AlignCenter)
        self.note_label.setFont(QFont("Arial", 9))
        layout.addWidget(self.note_label)
        
        # Add instruction for demo
        if click_callback:
            self.instruction_label = QLabel("(Click)")
            self.instruction_label.setAlignment(Qt.AlignCenter)
            self.instruction_label.setFont(QFont("Arial", 8))
            self.instruction_label.setStyleSheet("color: gray;")
            layout.addWidget(self.instruction_label)
        
        layout.addStretch()
    
    def mousePressEvent(self, event):
        """Handle mouse press"""
        if self.click_callback and event.button() == Qt.LeftButton:
            self.click_callback(True)
    
    def mouseReleaseEvent(self, event):
        """Handle mouse release"""
        if self.click_callback and event.button() == Qt.LeftButton:
            self.click_callback(False)
    
    def paintEvent(self, event):
        """Custom paint event to draw the light"""
        super().paintEvent(event)
        
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Calculate light position
        light_rect = self.light_area.geometry()
        center_x = light_rect.x() + light_rect.width() // 2
        center_y = light_rect.y() + light_rect.height() // 2
        radius = 25
        
        # Draw light circle
        if self.is_active:
            color = QColor(0, 255, 0)  # Green when active
        else:
            color = QColor(139, 0, 0)  # Dark red when inactive
        
        painter.setBrush(QBrush(color))
        painter.setPen(QColor(0, 0, 0))
        painter.drawEllipse(center_x - radius, center_y - radius, radius * 2, radius * 2)
    
    def set_active(self, active):
        """
        Set the active state of the finger light
        
        Args:
            active: Boolean indicating if finger is triggered
        """
        if active != self.is_active:
            self.is_active = active
            self.update()  # Trigger repaint
    
    def update_note(self, note_name):
        """
        Update the note name displayed for this finger
        
        Args:
            note_name: New MIDI note name to display
        """
        self.note_name = note_name
        self.note_label.setText(note_name)


class SensorProcessingThread(QThread):
    """
    Thread for processing mock sensor data
    """
    trigger_update = pyqtSignal(str, list, list)  # hand, states, events
    
    def __init__(self, mock_sensor, trigger_strategy, mapping_strategy, midi_writer):
        super().__init__()
        self.mock_sensor = mock_sensor
        self.trigger_strategy = trigger_strategy
        self.mapping_strategy = mapping_strategy
        self.midi_writer = midi_writer
        self.running = False
    
    def run(self):
        """Main sensor processing loop"""
        while self.running:
            try:
                # Process each hand
                for hand in ['l', 'r']:
                    # Get simulated sensor data
                    sensor_data = self.mock_sensor.get_sensor_data(hand)
                    
                    # Process triggers
                    trigger_events = self.trigger_strategy.process_triggers(sensor_data, hand)
                    
                    # Get trigger states
                    trigger_states = self.trigger_strategy.get_trigger_state(hand)
                    
                    # Process mapping if there are trigger events
                    if np.any(trigger_events != 0):
                        notes = self.mapping_strategy.map_to_notes(trigger_events, hand)
                        
                        # Send MIDI output for note events (if available)
                        if self.midi_writer:
                            for i, note in enumerate(notes):
                                if note is not None and trigger_events[i] == 1:
                                    # Note on
                                    self.midi_writer.trig_note(note, vel=80)
                                elif note is not None and trigger_events[i] == -1:
                                    # Note off
                                    self.midi_writer.trig_note(note, vel=0)
                    
                    # Emit signal for GUI update
                    self.trigger_update.emit(hand, trigger_states.tolist(), trigger_events.tolist())
                
                # Small delay to prevent excessive CPU usage
                self.msleep(50)  # 50ms delay = 20 FPS
                
            except Exception as e:
                print(f"Error in sensor processing thread: {e}")
                self.msleep(100)
    
    def start_processing(self):
        """Start the processing thread"""
        self.running = True
        self.start()
    
    def stop_processing(self):
        """Stop the processing thread"""
        self.running = False
        self.wait(1000)  # Wait up to 1 second for thread to finish


class TenFingerDemoQt(QMainWindow):
    """
    PyQt5 Demo version of the ten finger GUI with simulated sensor data
    """
    
    def __init__(self):
        """Initialize the demo application"""
        super().__init__()
        
        # Set window properties
        self.setWindowTitle("GLUVN Ten Finger Demo (PyQt5 - No Hardware Required)")
        self.setGeometry(100, 100, 1000, 700)
        
        # Application state
        self.running = False
        self.sensor_thread = None
        
        # Mock sensor data generator
        self.mock_sensor = MockSensorData()
        
        # Initialize GLUVN components
        self.setup_gluvn_components()
        
        # Create GUI elements
        self.setup_ui()
        
        # Auto demo variables
        self.auto_demo_timer = QTimer()
        self.auto_demo_timer.timeout.connect(self.auto_demo_step)
        self.auto_demo_step_counter = 0
        self.auto_demo_pattern = []
    
    def setup_gluvn_components(self):
        """
        Initialize GLUVN sensor processing components
        """
        # Configuration for trigger strategy
        self.trigger_config = {
            'thresholds': {'flex': 200},  # Adjustable via GUI
            'hysteresis': {'flex': 10},   # Adjustable via GUI
            'trigger_sensors': {'l': 'flex', 'r': 'flex'}
        }
        
        # Initialize trigger strategy
        self.trigger_strategy = HysteresisTrigger(self.trigger_config)
        
        # Initialize note mapper and mapping strategy
        self.note_mapper = NoteMapper(root_note='C', scale='major')
        self.mapping_strategy = BasicMapper(self.note_mapper, {})
        
        # Initialize MIDI writer (for actual note output)
        try:
            self.midi_writer = MidiWriter()
            self.midi_available = True
        except Exception as e:
            print(f"Warning: MIDI not available: {e}")
            self.midi_writer = None
            self.midi_available = False
        
        # Initialize sensor processing thread
        self.sensor_thread = SensorProcessingThread(
            self.mock_sensor, 
            self.trigger_strategy, 
            self.mapping_strategy, 
            self.midi_writer
        )
        self.sensor_thread.trigger_update.connect(self.update_finger_lights)
    
    def setup_ui(self):
        """Create and layout all GUI widgets"""
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # Title
        title_label = QLabel("GLUVN Ten Finger Demo")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        main_layout.addWidget(title_label)
        
        # Demo info
        info_label = QLabel("Demo Mode: Click on finger lights to manually trigger them, or use the Auto Demo button")
        info_label.setAlignment(Qt.AlignCenter)
        info_label.setFont(QFont("Arial", 10))
        info_label.setStyleSheet("background-color: lightblue; padding: 8px; border-radius: 4px;")
        main_layout.addWidget(info_label)
        
        # Control panel
        self.create_control_panel(main_layout)
        
        # Hands display
        self.create_hands_display(main_layout)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.status_bar.showMessage("Demo Ready - Click 'Start Demo' or click on finger lights")
        self.setStatusBar(self.status_bar)
    
    def create_control_panel(self, main_layout):
        """Create the control panel with demo controls and configuration options"""
        control_group = QGroupBox("Controls")
        main_layout.addWidget(control_group)
        
        control_layout = QHBoxLayout(control_group)
        
        # Demo control buttons
        button_layout = QVBoxLayout()
        
        button_row1 = QHBoxLayout()
        self.start_button = QPushButton("Start Demo")
        self.start_button.clicked.connect(self.start_demo)
        button_row1.addWidget(self.start_button)
        
        self.stop_button = QPushButton("Stop Demo")
        self.stop_button.clicked.connect(self.stop_demo)
        self.stop_button.setEnabled(False)
        button_row1.addWidget(self.stop_button)
        
        self.auto_demo_button = QPushButton("Auto Demo")
        self.auto_demo_button.clicked.connect(self.start_auto_demo)
        button_row1.addWidget(self.auto_demo_button)
        
        button_layout.addLayout(button_row1)
        control_layout.addLayout(button_layout)
        
        # Configuration controls
        config_layout = QGridLayout()
        
        # Threshold adjustment
        config_layout.addWidget(QLabel("Threshold:"), 0, 0)
        self.threshold_spinbox = QSpinBox()
        self.threshold_spinbox.setRange(50, 500)
        self.threshold_spinbox.setValue(200)
        self.threshold_spinbox.valueChanged.connect(self.update_threshold)
        config_layout.addWidget(self.threshold_spinbox, 0, 1)
        
        # Hysteresis adjustment
        config_layout.addWidget(QLabel("Hysteresis:"), 0, 2)
        self.hysteresis_spinbox = QSpinBox()
        self.hysteresis_spinbox.setRange(1, 50)
        self.hysteresis_spinbox.setValue(10)
        self.hysteresis_spinbox.valueChanged.connect(self.update_hysteresis)
        config_layout.addWidget(self.hysteresis_spinbox, 0, 3)
        
        # Scale selection
        config_layout.addWidget(QLabel("Scale:"), 1, 0)
        self.scale_combo = QComboBox()
        self.scale_combo.addItems(['major', 'minor', 'pentatonic'])
        self.scale_combo.currentTextChanged.connect(self.update_scale)
        config_layout.addWidget(self.scale_combo, 1, 1)
        
        # Root note selection
        config_layout.addWidget(QLabel("Root Note:"), 1, 2)
        self.root_note_combo = QComboBox()
        self.root_note_combo.addItems(['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B'])
        self.root_note_combo.currentTextChanged.connect(self.update_root_note)
        config_layout.addWidget(self.root_note_combo, 1, 3)
        
        control_layout.addLayout(config_layout)
    
    def create_hands_display(self, main_layout):
        """Create the visual display of both hands with clickable finger lights"""
        hands_group = QGroupBox("Finger Lights")
        main_layout.addWidget(hands_group)
        
        hands_layout = QHBoxLayout(hands_group)
        
        # Finger names for display
        finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        
        # Left hand
        left_hand_group = QGroupBox("Left Hand")
        left_hand_layout = QHBoxLayout(left_hand_group)
        
        self.left_lights = []
        left_notes = self.mapping_strategy.get_note_mapping('l')
        
        for i, (finger_name, note_midi) in enumerate(zip(finger_names, left_notes)):
            if note_midi is not None:
                note_name = self.note_mapper.midi2note.get(note_midi, f"Note{note_midi}")
            else:
                note_name = "N/A"
            
            # Create click callback for this finger
            click_callback = lambda triggered, h='l', f=i: self.on_finger_click(h, f, triggered)
            
            light = FingerLight(finger_name, note_name, click_callback)
            left_hand_layout.addWidget(light)
            self.left_lights.append(light)
        
        # Right hand
        right_hand_group = QGroupBox("Right Hand")
        right_hand_layout = QHBoxLayout(right_hand_group)
        
        self.right_lights = []
        right_notes = self.mapping_strategy.get_note_mapping('r')
        
        for i, (finger_name, note_midi) in enumerate(zip(finger_names, right_notes)):
            if note_midi is not None:
                note_name = self.note_mapper.midi2note.get(note_midi, f"Note{note_midi}")
            else:
                note_name = "N/A"
            
            # Create click callback for this finger
            click_callback = lambda triggered, h='r', f=i: self.on_finger_click(h, f, triggered)
            
            light = FingerLight(finger_name, note_name, click_callback)
            right_hand_layout.addWidget(light)
            self.right_lights.append(light)
        
        hands_layout.addWidget(left_hand_group)
        hands_layout.addWidget(right_hand_group)
    
    def on_finger_click(self, hand: str, finger: int, triggered: bool):
        """
        Handle finger light click events
        
        Args:
            hand: Hand identifier ('l' or 'r')
            finger: Finger index (0-4)
            triggered: Whether finger is being triggered (True) or released (False)
        """
        # Update mock sensor data
        self.mock_sensor.set_finger_trigger(hand, finger, triggered)
        
        # Update status
        finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        action = "Triggered" if triggered else "Released"
        self.status_bar.showMessage(f"{action} {hand.upper()} {finger_names[finger]}")
    
    def update_threshold(self, value):
        """Update the trigger threshold when user changes the value"""
        self.trigger_strategy.set_threshold('flex', float(value))
        self.status_bar.showMessage(f"Updated threshold to {value}")
    
    def update_hysteresis(self, value):
        """Update the hysteresis value when user changes it"""
        self.trigger_strategy.set_hysteresis('flex', float(value))
        self.status_bar.showMessage(f"Updated hysteresis to {value}")
    
    def update_scale(self, scale):
        """Update the musical scale and refresh note mappings"""
        self.mapping_strategy.update_mapping(scale=scale)
        self.update_note_displays()
        self.status_bar.showMessage(f"Updated scale to {scale}")
    
    def update_root_note(self, root_note):
        """Update the root note and refresh note mappings"""
        self.mapping_strategy.update_mapping(root_note=root_note)
        self.update_note_displays()
        self.status_bar.showMessage(f"Updated root note to {root_note}")
    
    def update_note_displays(self):
        """Update the note names displayed on each finger light"""
        # Update left hand
        left_notes = self.mapping_strategy.get_note_mapping('l')
        for i, light in enumerate(self.left_lights):
            if i < len(left_notes) and left_notes[i] is not None:
                note_name = self.note_mapper.midi2note.get(left_notes[i], f"Note{left_notes[i]}")
                light.update_note(note_name)
        
        # Update right hand
        right_notes = self.mapping_strategy.get_note_mapping('r')
        for i, light in enumerate(self.right_lights):
            if i < len(right_notes) and right_notes[i] is not None:
                note_name = self.note_mapper.midi2note.get(right_notes[i], f"Note{right_notes[i]}")
                light.update_note(note_name)
    
    def start_demo(self):
        """Start the demo sensor processing"""
        self.sensor_thread.start_processing()
        
        # Update UI
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.status_bar.showMessage("Demo running - Click on finger lights to trigger them")
    
    def stop_demo(self):
        """Stop the demo processing"""
        # Stop processing
        self.sensor_thread.stop_processing()
        
        # Stop auto demo if running
        self.auto_demo_timer.stop()
        
        # Update UI
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_bar.showMessage("Demo stopped")
        
        # Clear all lights
        for light in self.left_lights + self.right_lights:
            light.set_active(False)
        
        # Reset mock sensor triggers
        for hand in ['l', 'r']:
            for finger in range(5):
                self.mock_sensor.set_finger_trigger(hand, finger, False)
    
    def start_auto_demo(self):
        """Start automatic demo sequence"""
        if not self.sensor_thread.running:
            self.start_demo()
        
        # Set up demo patterns
        demo_patterns = [
            # Pattern 1: Left hand scales
            [('l', 0), ('l', 1), ('l', 2), ('l', 3), ('l', 4)],
            # Pattern 2: Right hand scales  
            [('r', 0), ('r', 1), ('r', 2), ('r', 3), ('r', 4)],
            # Pattern 3: Alternating hands
            [('l', 0), ('r', 0), ('l', 1), ('r', 1), ('l', 2), ('r', 2)],
            # Pattern 4: Chords
            [('l', 0), ('l', 2), ('l', 4), ('r', 1), ('r', 3)],
        ]
        
        # Flatten patterns into a sequence
        self.auto_demo_pattern = []
        for pattern in demo_patterns:
            # Trigger sequence
            for hand, finger in pattern:
                self.auto_demo_pattern.append(('trigger', hand, finger))
            self.auto_demo_pattern.append(('wait', 0.5))
            # Release sequence
            for hand, finger in pattern:
                self.auto_demo_pattern.append(('release', hand, finger))
            self.auto_demo_pattern.append(('wait', 0.8))
        
        self.auto_demo_step_counter = 0
        self.auto_demo_timer.start(300)  # Execute every 300ms
        self.status_bar.showMessage("Auto demo running - watch the lights!")
    
    def auto_demo_step(self):
        """Execute one step of the auto demo"""
        if self.auto_demo_step_counter >= len(self.auto_demo_pattern):
            # Demo complete, restart
            self.auto_demo_step_counter = 0
            return
        
        step = self.auto_demo_pattern[self.auto_demo_step_counter]
        
        if step[0] == 'trigger':
            hand, finger = step[1], step[2]
            self.mock_sensor.set_finger_trigger(hand, finger, True)
        elif step[0] == 'release':
            hand, finger = step[1], step[2]
            self.mock_sensor.set_finger_trigger(hand, finger, False)
        elif step[0] == 'wait':
            # For wait steps, we just continue to the next step
            pass
        
        self.auto_demo_step_counter += 1
    
    def update_finger_lights(self, hand, states, events):
        """
        Update GUI finger lights based on trigger states
        
        Args:
            hand: Hand identifier ('l' or 'r')
            states: List of trigger states for each finger
            events: List of trigger events for each finger
        """
        # Update lights for this hand
        if hand == 'l':
            lights = self.left_lights
        elif hand == 'r':
            lights = self.right_lights
        else:
            return
        
        # Update each finger light
        for i, light in enumerate(lights):
            if i < len(states):
                light.set_active(bool(states[i]))
    
    def closeEvent(self, event):
        """Handle application shutdown"""
        if self.sensor_thread.running:
            self.stop_demo()
        
        # Make sure thread is stopped
        self.sensor_thread.stop_processing()
        
        event.accept()


def main():
    """Main entry point for the PyQt5 ten finger demo application"""
    app = QApplication(sys.argv)
    
    print("Starting GLUVN Ten Finger Demo Application (PyQt5)")
    print("=" * 50)
    print("This is a demo version that doesn't require hardware.")
    print("You can:")
    print("- Click on finger lights to manually trigger them")
    print("- Use the Auto Demo button for automated sequences")
    print("- Adjust thresholds and musical settings in real-time")
    print("- Test MIDI output (if MIDI device is available)")
    print("=" * 50)
    
    try:
        window = TenFingerDemoQt()
        window.show()
        
        sys.exit(app.exec_())
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"Error starting demo: {e}")
        QMessageBox.critical(None, "Error", f"Failed to start demo: {e}")


if __name__ == "__main__":
    main() 