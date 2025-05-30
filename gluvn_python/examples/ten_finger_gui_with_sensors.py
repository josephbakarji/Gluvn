#!/usr/bin/env python3
"""
Ten Finger Hardware GUI with Real-Time Sensor Values

Enhanced version of the ten finger GUI that displays actual sensor readings
in real-time above each trigger indicator. This version uses the proper
GLUVN architecture with strategies and reusable visualization components.

Features:
- Real-time sensor value display (flex/pressure values)
- Visual finger lights showing trigger states
- Configurable thresholds with visual threshold lines
- Live sensor value sliders/progress bars
- MIDI output for triggered notes
- Proper integration with GLUVN trigger strategies
- Reusable visualization components

Author: Joseph Bakarji
Usage: python ten_finger_gui_with_sensors.py
"""

import sys
import os

# PyQt5 imports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGroupBox, QSpinBox, QComboBox, 
                            QPushButton, QLabel, QStatusBar, QMessageBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import GLUVN components
from visualization import TenFingerDisplay, SensorProcessingThread
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


class TenFingerSensorGUI(QMainWindow):
    """Main GUI application with real-time sensor value display using GLUVN architecture"""
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("GLUVN Ten Finger GUI - Real-Time Sensor Values (Clean Architecture)")
        self.setGeometry(100, 100, 1200, 800)
        
        # Application state
        self.running = False
        self.sensor_thread = None
        self.reader = None
        
        # Configuration
        self.threshold = 120
        self.hysteresis = 10
        self.sensor_type = 'flex'
        
        # Initialize MIDI and mapping
        self._initialize_midi_system()
        
        # Initialize hardware
        self._initialize_hardware()
        
        # Setup UI
        self._setup_ui()
    
    def _initialize_midi_system(self):
        """Initialize MIDI writer and note mapping"""
        try:
            self.midi_writer = MidiWriter()
            self.note_mapper = NoteMapper(root_note='C', scale='major')
            self.note_maps = self.note_mapper.basic_map_2hands()
            self.midi_available = True
            print("🎹 MIDI system initialized successfully")
        except Exception as e:
            print(f"⚠️ Warning: MIDI not available: {e}")
            self.midi_writer = None
            self.note_maps = {'l': [], 'r': []}
            self.midi_available = False
    
    def _initialize_hardware(self):
        """Initialize hardware reader"""
        if HARDWARE_AVAILABLE:
            try:
                sensor_config = {
                    'l': {'flex': True, 'press': True, 'imu': False},
                    'r': {'flex': True, 'press': True, 'imu': False}
                }
                self.reader = Reader(sensor_config=sensor_config)
                self.hardware_available = True
                print("🔌 Hardware reader initialized successfully")
            except Exception as e:
                print(f"⚠️ Warning: Could not initialize sensor reader: {e}")
                self.reader = None
                self.hardware_available = False
        else:
            self.hardware_available = False
    
    def _setup_ui(self):
        """Create the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # Title
        title = QLabel("GLUVN Ten Finger GUI - Clean Architecture with Strategies")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Bold))
        main_layout.addWidget(title)
        
        # Status info
        status_text, status_color = self._get_status_info()
        status_label = QLabel(status_text)
        status_label.setAlignment(Qt.AlignCenter)
        status_label.setStyleSheet(f"background-color: {status_color}; padding: 10px; border-radius: 5px;")
        main_layout.addWidget(status_label)
        
        # Control panel
        self._create_control_panel(main_layout)
        
        # Ten finger display using reusable visualization component
        self.finger_display = TenFingerDisplay(
            note_maps=self.note_maps, 
            sensor_type=self.sensor_type
        )
        self.finger_display.update_note_maps(self.note_maps, self.note_mapper)
        self.finger_display.set_threshold(self.threshold)
        
        display_group = QGroupBox("Finger Sensors - Real-Time Values")
        display_layout = QVBoxLayout(display_group)
        display_layout.addWidget(self.finger_display)
        main_layout.addWidget(display_group)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.status_bar.showMessage("Ready - Click 'Start Sensors' to begin")
        self.setStatusBar(self.status_bar)
    
    def _get_status_info(self):
        """Get status text and color based on system state"""
        if self.hardware_available:
            if self.midi_available:
                return "✅ Hardware + MIDI Connected - Full functionality available", "lightgreen"
            else:
                return "⚠️ Hardware Connected, MIDI Unavailable - Sensor display only", "lightyellow"
        else:
            return "❌ Hardware Not Available - Check connection and restart", "lightcoral"
    
    def _create_control_panel(self, main_layout):
        """Create control panel with configuration options"""
        control_group = QGroupBox("Controls & Configuration")
        main_layout.addWidget(control_group)
        
        control_layout = QHBoxLayout(control_group)
        
        # Buttons
        button_layout = QVBoxLayout()
        
        button_row = QHBoxLayout()
        self.start_button = QPushButton("Start Sensors")
        self.start_button.clicked.connect(self.start_sensors)
        self.start_button.setEnabled(self.hardware_available)
        button_row.addWidget(self.start_button)
        
        self.stop_button = QPushButton("Stop Sensors")
        self.stop_button.clicked.connect(self.stop_sensors)
        self.stop_button.setEnabled(False)
        button_row.addWidget(self.stop_button)
        
        button_layout.addLayout(button_row)
        control_layout.addLayout(button_layout)
        
        # Configuration controls
        config_layout = QVBoxLayout()
        
        # Row 1: Threshold and Hysteresis
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Threshold:"))
        self.threshold_spinbox = QSpinBox()
        self.threshold_spinbox.setRange(50, 500)
        self.threshold_spinbox.setValue(self.threshold)
        self.threshold_spinbox.valueChanged.connect(self.update_threshold)
        row1.addWidget(self.threshold_spinbox)
        
        row1.addWidget(QLabel("Hysteresis:"))
        self.hysteresis_spinbox = QSpinBox()
        self.hysteresis_spinbox.setRange(1, 50)
        self.hysteresis_spinbox.setValue(self.hysteresis)
        self.hysteresis_spinbox.valueChanged.connect(self.update_hysteresis)
        row1.addWidget(self.hysteresis_spinbox)
        
        config_layout.addLayout(row1)
        
        # Row 2: Sensor type
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Sensor Type:"))
        self.sensor_combo = QComboBox()
        self.sensor_combo.addItems(['flex', 'press'])
        self.sensor_combo.currentTextChanged.connect(self.update_sensor_type)
        row2.addWidget(self.sensor_combo)
        
        config_layout.addLayout(row2)
        control_layout.addLayout(config_layout)
    
    def start_sensors(self):
        """Start sensor processing using proper trigger strategy"""
        if not self.hardware_available:
            QMessageBox.warning(self, "Hardware Error", "No hardware available!")
            return
        
        try:
            # Create trigger strategy configuration
            trigger_config = {
                'thresholds': {self.sensor_type: self.threshold},
                'hysteresis': {self.sensor_type: self.hysteresis},
                'trigger_sensors': {'l': self.sensor_type, 'r': self.sensor_type}
            }
            
            # Initialize sensor processing thread with proper strategy
            self.sensor_thread = SensorProcessingThread(
                self.reader, trigger_config, self.sensor_type
            )
            self.sensor_thread.sensor_update.connect(self.on_sensor_update)
            self.sensor_thread.error_signal.connect(self.on_error)
            self.sensor_thread.start_processing()
            
            self.running = True
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.status_bar.showMessage("🔥 Sensors running with GLUVN trigger strategies - Flex your fingers!")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start sensors: {e}")
            print(f"❌ Error starting sensors: {e}")
    
    def stop_sensors(self):
        """Stop sensor processing"""
        if self.sensor_thread:
            self.sensor_thread.stop_processing()
            self.sensor_thread = None
        
        self.running = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_bar.showMessage("⏹️ Sensors stopped")
    
    def on_sensor_update(self, hand, finger_idx, sensor_value, triggered, switch_event):
        """Handle sensor data updates from the processing thread"""
        # Update visualization using reusable component
        self.finger_display.update_sensor_value(hand, finger_idx, sensor_value)
        self.finger_display.set_triggered(hand, finger_idx, triggered)
        
        # Play MIDI note only on switch events (edge transitions)
        if self.midi_available and self.midi_writer and switch_event != 0:
            if hand in self.note_maps and finger_idx < len(self.note_maps[hand]):
                note = self.note_maps[hand][finger_idx]
                
                if switch_event == 1:
                    # Turn note ON
                    self.midi_writer.trig_note(note, vel=80)
                    print(f"🎵 MIDI ON: {hand.upper()} finger {finger_idx}, note {note}")
                elif switch_event == -1:
                    # Turn note OFF  
                    self.midi_writer.trig_note(note, vel=0)
                    print(f"🎵 MIDI OFF: {hand.upper()} finger {finger_idx}, note {note}")
    
    def on_error(self, error_message):
        """Handle errors from sensor thread"""
        print(f"❌ Sensor error: {error_message}")
        self.status_bar.showMessage(f"Error: {error_message}")
    
    def update_threshold(self, value):
        """Update threshold value"""
        self.threshold = value
        self.finger_display.set_threshold(value)
        
        if self.sensor_thread:
            self.sensor_thread.update_threshold(self.sensor_type, value)
    
    def update_hysteresis(self, value):
        """Update hysteresis value"""
        self.hysteresis = value
        if self.sensor_thread:
            self.sensor_thread.update_hysteresis(self.sensor_type, value)
    
    def update_sensor_type(self, sensor_type):
        """Update sensor type"""
        self.sensor_type = sensor_type
        self.status_bar.showMessage(f"🔄 Sensor type changed to {sensor_type} - Restart sensors to apply")
    
    def closeEvent(self, event):
        """Handle application close"""
        if self.running:
            self.stop_sensors()
        event.accept()


def main():
    """Main application entry point"""
    print("🚀 Starting GLUVN Ten Finger GUI with Clean Architecture")
    print("=" * 70)
    print("🏗️  This version uses the proper GLUVN strategy system:")
    print("   • Trigger logic in core.strategies.trigger_strategies")
    print("   • Reusable visualization components")
    print("   • Clean separation of concerns")
    print("   • Consistent behavior across applications")
    print("=" * 70)
    
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = TenFingerSensorGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 