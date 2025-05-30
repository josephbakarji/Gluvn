#!/usr/bin/env python3
"""
Ten Finger Hardware GUI Application (PyQt5 Version)

This is a PyQt5 version of the ten finger GUI that works with actual GLUVN hardware.
It provides real-time sensor data processing and visual feedback for trigger states.

Features:
- Real-time sensor data processing from GLUVN hardware
- Visual finger lights showing trigger states
- Configurable thresholds and hysteresis
- Musical scale and root note selection
- MIDI output for triggered notes
- PyQt5 interface matching existing sensor plotting applications

Author: Joseph Bakarji
Usage: python ten_finger_gui_pyqt.py
"""

import sys
import os
import numpy as np
import threading
import queue
import time

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

# Try to import hardware reader
try:
    from port_read import Reader
    HARDWARE_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Hardware reader not available: {e}")
    HARDWARE_AVAILABLE = False
    Reader = None


class FingerLight(QWidget):
    """
    PyQt5 Visual representation of a single finger trigger state
    """
    
    def __init__(self, finger_name, note_name):
        """
        Initialize a finger light widget
        
        Args:
            finger_name: Name of the finger (e.g., "Thumb", "Index")
            note_name: MIDI note name for this finger
        """
        super().__init__()
        self.finger_name = finger_name
        self.note_name = note_name
        self.is_active = False
        
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
        
        layout.addStretch()
    
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
    Thread for processing real sensor data from hardware
    """
    trigger_update = pyqtSignal(str, list, list)  # hand, states, events
    error_signal = pyqtSignal(str)  # error message
    
    def __init__(self, reader, trigger_strategy, mapping_strategy, midi_writer):
        super().__init__()
        self.reader = reader
        self.trigger_strategy = trigger_strategy
        self.mapping_strategy = mapping_strategy
        self.midi_writer = midi_writer
        self.running = False
    
    def run(self):
        """Main sensor processing loop"""
        try:
            # Start the hardware reader
            self.reader.start_readers()
            
            while self.running:
                try:
                    # Process each hand
                    for hand in ['l', 'r']:
                        # Check if we have a parser for this hand
                        if hand not in self.reader.threads or 'parser' not in self.reader.threads[hand]:
                            continue
                        
                        parser_queue = self.reader.threads[hand]['parser'].getQ()
                        
                        # Try to get sensor data (non-blocking)
                        try:
                            sensor_data = parser_queue.get_nowait()
                        except queue.Empty:
                            continue
                        
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
                    self.msleep(20)  # 20ms delay = 50 FPS
                    
                except Exception as e:
                    self.error_signal.emit(f"Error in sensor processing: {e}")
                    self.msleep(100)
                    
        except Exception as e:
            self.error_signal.emit(f"Failed to start sensor reader: {e}")
        finally:
            # Clean up
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
        self.wait(2000)  # Wait up to 2 seconds for thread to finish


class TenFingerGUIQt(QMainWindow):
    """
    PyQt5 Hardware version of the ten finger GUI with real sensor data
    """
    
    def __init__(self):
        """Initialize the hardware GUI application"""
        super().__init__()
        
        # Set window properties
        self.setWindowTitle("GLUVN Ten Finger Hardware GUI (PyQt5)")
        self.setGeometry(100, 100, 1000, 700)
        
        # Application state
        self.running = False
        self.sensor_thread = None
        self.reader = None
        
        # Initialize GLUVN components
        self.setup_gluvn_components()
        
        # Create GUI elements
        self.setup_ui()
    
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
        
        # Initialize hardware reader
        if HARDWARE_AVAILABLE:
            try:
                # Configure sensor reading for both hands with flex sensors
                sensor_config = {
                    'l': {'flex': True, 'press': False, 'imu': False},
                    'r': {'flex': True, 'press': False, 'imu': False}
                }
                self.reader = Reader(sensor_config=sensor_config)
                self.hardware_available = True
                print("Hardware reader initialized successfully")
            except Exception as e:
                print(f"Warning: Could not initialize sensor reader: {e}")
                self.reader = None
                self.hardware_available = False
        else:
            self.hardware_available = False
    
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
        title_label = QLabel("GLUVN Ten Finger Hardware GUI")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        main_layout.addWidget(title_label)
        
        # Hardware status info
        if self.hardware_available:
            info_text = "Hardware Mode: Connect your GLUVN sensor glove and click 'Start Sensors'"
            info_color = "lightgreen"
        else:
            info_text = "Hardware Not Available: Check connection and try restarting the application"
            info_color = "lightcoral"
        
        info_label = QLabel(info_text)
        info_label.setAlignment(Qt.AlignCenter)
        info_label.setFont(QFont("Arial", 10))
        info_label.setStyleSheet(f"background-color: {info_color}; padding: 8px; border-radius: 4px;")
        main_layout.addWidget(info_label)
        
        # Control panel
        self.create_control_panel(main_layout)
        
        # Hands display
        self.create_hands_display(main_layout)
        
        # Status bar
        self.status_bar = QStatusBar()
        if self.hardware_available:
            self.status_bar.showMessage("Ready - Click 'Start Sensors' to begin")
        else:
            self.status_bar.showMessage("Hardware not available - Check connection")
        self.setStatusBar(self.status_bar)
    
    def create_control_panel(self, main_layout):
        """Create the control panel with start/stop and configuration options"""
        control_group = QGroupBox("Controls")
        main_layout.addWidget(control_group)
        
        control_layout = QHBoxLayout(control_group)
        
        # Sensor control buttons
        button_layout = QVBoxLayout()
        
        button_row1 = QHBoxLayout()
        self.start_button = QPushButton("Start Sensors")
        self.start_button.clicked.connect(self.start_sensors)
        self.start_button.setEnabled(self.hardware_available)
        button_row1.addWidget(self.start_button)
        
        self.stop_button = QPushButton("Stop Sensors")
        self.stop_button.clicked.connect(self.stop_sensors)
        self.stop_button.setEnabled(False)
        button_row1.addWidget(self.stop_button)
        
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
        """Create the visual display of both hands with finger lights"""
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
            
            light = FingerLight(finger_name, note_name)
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
            
            light = FingerLight(finger_name, note_name)
            right_hand_layout.addWidget(light)
            self.right_lights.append(light)
        
        hands_layout.addWidget(left_hand_group)
        hands_layout.addWidget(right_hand_group)
    
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
    
    def start_sensors(self):
        """Start sensor reading and processing"""
        if not self.hardware_available:
            QMessageBox.critical(self, "Error", "Hardware not available. Check connection and restart application.")
            return
        
        try:
            # Initialize sensor processing thread
            self.sensor_thread = SensorProcessingThread(
                self.reader, 
                self.trigger_strategy, 
                self.mapping_strategy, 
                self.midi_writer
            )
            self.sensor_thread.trigger_update.connect(self.update_finger_lights)
            self.sensor_thread.error_signal.connect(self.handle_sensor_error)
            
            # Start processing
            self.sensor_thread.start_processing()
            
            # Update UI
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.status_bar.showMessage("Sensors running - Move fingers to see triggers")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start sensors: {e}")
    
    def stop_sensors(self):
        """Stop sensor reading and processing"""
        # Stop processing
        if self.sensor_thread:
            self.sensor_thread.stop_processing()
            self.sensor_thread = None
        
        # Update UI
        self.start_button.setEnabled(self.hardware_available)
        self.stop_button.setEnabled(False)
        self.status_bar.showMessage("Sensors stopped")
        
        # Clear all lights
        for light in self.left_lights + self.right_lights:
            light.set_active(False)
    
    def handle_sensor_error(self, error_message):
        """Handle errors from the sensor processing thread"""
        self.status_bar.showMessage(f"Sensor error: {error_message}")
        QMessageBox.warning(self, "Sensor Error", error_message)
    
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
        if self.sensor_thread and self.sensor_thread.running:
            self.stop_sensors()
        
        event.accept()


def main():
    """Main entry point for the PyQt5 ten finger hardware GUI application"""
    app = QApplication(sys.argv)
    
    print("Starting GLUVN Ten Finger Hardware GUI (PyQt5)")
    print("=" * 50)
    
    if not HARDWARE_AVAILABLE:
        print("Warning: Hardware reader not available")
        print("Make sure your GLUVN hardware is connected")
    
    print("Features:")
    print("- Real-time sensor data processing")
    print("- Visual finger trigger feedback")
    print("- Configurable thresholds and musical settings")
    print("- MIDI output for triggered notes")
    print("=" * 50)
    
    try:
        window = TenFingerGUIQt()
        window.show()
        
        sys.exit(app.exec_())
        
    except KeyboardInterrupt:
        print("\nApplication interrupted by user")
    except Exception as e:
        print(f"Error starting application: {e}")
        QMessageBox.critical(None, "Error", f"Failed to start application: {e}")


if __name__ == "__main__":
    main() 