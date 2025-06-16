#!/usr/bin/env python3
"""
Ten Finger Hardware GUI with Real-Time Sensor Values and Pitch Bend

Enhanced version of the ten finger GUI that displays actual sensor readings
in real-time above each trigger indicator, and adds pitch bend control using IMU2.

Features:
- Real-time sensor value display (flex/pressure values)
- Visual finger lights showing trigger states
- Configurable thresholds with visual threshold lines
- Live sensor value sliders/progress bars
- MIDI output for triggered notes
- Pitch bend using IMU2 (pitch)
- Live IMU data plotting using pyqtgraph
- Proper integration with GLUVN trigger strategies
- Reusable visualization components

Author: Joseph Bakarji
Usage: python ten_finger_gui_with_pitchbend.py
"""

import sys
import os
import time
import numpy as np
from collections import deque

# PyQt5 imports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGroupBox, QSpinBox, QComboBox, 
                            QPushButton, QLabel, QStatusBar, QMessageBox, QCheckBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

# PyQtGraph imports
import pyqtgraph as pg

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

class IMUPlotWidget(QWidget):
    """Widget for displaying live IMU data plots using pyqtgraph"""
    def __init__(self, max_points=1000):
        super().__init__()
        self.max_points = max_points
        
        # Create data buffers using numpy arrays
        self.data_buffer = {
            'l': np.zeros((max_points, 3), dtype=np.float32),  # yaw, pitch, roll
            'r': np.zeros((max_points, 3), dtype=np.float32)
        }
        self.buffer_index = {'l': 0, 'r': 0}
        
        # Create layout
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Create plot widgets
        self.plot_widgets = {
            'l': pg.PlotWidget(title='Left Hand IMU'),
            'r': pg.PlotWidget(title='Right Hand IMU')
        }
        
        # Setup plots
        for hand in ['l', 'r']:
            plot = self.plot_widgets[hand]
            plot.showGrid(x=True, y=True)
            plot.setLabel('left', 'Value')
            plot.setLabel('bottom', 'Sample')
            plot.setYRange(0, 65535)  # IMU range now 0-65535
            plot.addLegend(offset=(10, 10))
            layout.addWidget(plot)
        
        # Create curves with different colors
        self.curves = {
            'l': [
                self.plot_widgets['l'].plot(pen=pg.mkPen(color=(255,0,0), width=2), name='Yaw'),
                self.plot_widgets['l'].plot(pen=pg.mkPen(color=(0,255,0), width=2), name='Pitch'),
                self.plot_widgets['l'].plot(pen=pg.mkPen(color=(0,0,255), width=2), name='Roll')
            ],
            'r': [
                self.plot_widgets['r'].plot(pen=pg.mkPen(color=(255,0,0), width=2), name='Yaw'),
                self.plot_widgets['r'].plot(pen=pg.mkPen(color=(0,255,0), width=2), name='Pitch'),
                self.plot_widgets['r'].plot(pen=pg.mkPen(color=(0,0,255), width=2), name='Roll')
            ]
        }
    
    def update_plot(self, hand, imu_array):
        """Update the plot with new IMU data"""
        if not isinstance(imu_array, (list, tuple)) or len(imu_array) < 3:
            return
            
        # Get yaw, pitch, roll values
        ypr_data = np.array(imu_array[:3], dtype=np.float32)
        
        # Apply smoothing if we have previous data
        if self.buffer_index[hand] > 0:
            ypr_data = 0.9 * self.data_buffer[hand][self.buffer_index[hand]-1] + 0.1 * ypr_data
        
        # Handle buffer rollover
        if self.buffer_index[hand] >= self.max_points:
            self.data_buffer[hand] = np.roll(self.data_buffer[hand], -1, axis=0)
            self.buffer_index[hand] = self.max_points - 1
        
        # Store new data
        self.data_buffer[hand][self.buffer_index[hand]] = ypr_data
        self.buffer_index[hand] += 1
        
        # Update plot
        if self.buffer_index[hand] > 0:
            valid_data = self.data_buffer[hand][:self.buffer_index[hand]]
            x = np.arange(len(valid_data))
            for i in range(3):
                self.curves[hand][i].setData(x, valid_data[:, i])

class TenFingerPitchBendGUI(QMainWindow):
    """Main GUI application with real-time sensor value display and pitch bend using IMU2"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GLUVN Ten Finger GUI with Pitch Bend (IMU2)")
        self.setGeometry(100, 100, 1200, 1000)  # Increased height for IMU plot
        # Application state
        self.running = False
        self.sensor_thread = None
        self.reader = None
        # Configuration
        self.threshold = 120
        self.hysteresis = 10
        self.sensor_type = 'flex'
        # Debug settings
        self.debug_printing = False
        # Initialize MIDI and mapping
        self._initialize_midi_system()
        # Initialize hardware
        self._initialize_hardware()
        # Setup UI
        self.pitch_bend_imu_channel = 1  # Default to IMU2 (index 1)
        self._setup_ui()
        # For pitch bend throttling
        self.last_pitch_update = 0
        self.midi_update_interval = 0.05  # 50ms
    def _initialize_midi_system(self):
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
        if HARDWARE_AVAILABLE:
            try:
                sensor_config = {
                    'l': {'flex': True, 'press': True, 'imu': True},
                    'r': {'flex': True, 'press': True, 'imu': True}
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
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        title = QLabel("GLUVN Ten Finger GUI with Pitch Bend (IMU2)")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Bold))
        main_layout.addWidget(title)
        status_text, status_color = self._get_status_info()
        status_label = QLabel(status_text)
        status_label.setAlignment(Qt.AlignCenter)
        status_label.setStyleSheet(f"background-color: {status_color}; padding: 10px; border-radius: 5px;")
        main_layout.addWidget(status_label)
        self._create_control_panel(main_layout)
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
        # Add IMU plot
        self.imu_plot = IMUPlotWidget()
        imu_group = QGroupBox("IMU Data Plot")
        imu_layout = QVBoxLayout(imu_group)
        imu_layout.addWidget(self.imu_plot)
        main_layout.addWidget(imu_group)
        # Add pitch bend source selector
        pitch_bend_group = QGroupBox('Pitch Bend Source')
        pitch_bend_layout = QHBoxLayout(pitch_bend_group)
        pitch_bend_label = QLabel('Pitch Bend Source (IMU channel):')
        self.pitch_bend_combo = QComboBox()
        self.pitch_bend_combo.addItems([
            'IMU0 (Yaw)',
            'IMU1 (Pitch)',
            'IMU2 (Roll)',
            'IMU3 (Accel X)',
            'IMU4 (Accel Y)',
            'IMU5 (Accel Z)'])
        self.pitch_bend_combo.setCurrentIndex(self.pitch_bend_imu_channel)
        self.pitch_bend_combo.currentIndexChanged.connect(self._on_pitch_bend_channel_changed)
        pitch_bend_layout.addWidget(pitch_bend_label)
        pitch_bend_layout.addWidget(self.pitch_bend_combo)
        main_layout.addWidget(pitch_bend_group)
        self.status_bar = QStatusBar()
        self.status_bar.showMessage("Ready - Click 'Start Sensors' to begin")
        self.setStatusBar(self.status_bar)
    def _get_status_info(self):
        if self.hardware_available:
            if self.midi_available:
                return "✅ Hardware + MIDI Connected - Full functionality available", "lightgreen"
            else:
                return "⚠️ Hardware Connected, MIDI Unavailable - Sensor display only", "lightyellow"
        else:
            return "❌ Hardware Not Available - Check connection and restart", "lightcoral"
    def _create_control_panel(self, main_layout):
        control_group = QGroupBox("Controls & Configuration")
        main_layout.addWidget(control_group)
        control_layout = QHBoxLayout(control_group)
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
        config_layout = QVBoxLayout()
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
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Sensor Type:"))
        self.sensor_combo = QComboBox()
        self.sensor_combo.addItems(['flex', 'press'])
        self.sensor_combo.currentTextChanged.connect(self.update_sensor_type)
        row2.addWidget(self.sensor_combo)
        row3 = QHBoxLayout()
        self.debug_checkbox = QCheckBox("Enable Debug Printing")
        self.debug_checkbox.setChecked(self.debug_printing)
        self.debug_checkbox.stateChanged.connect(self.toggle_debug_printing)
        row3.addWidget(self.debug_checkbox)
        config_layout.addLayout(row2)
        config_layout.addLayout(row3)
        control_layout.addLayout(config_layout)
    def start_sensors(self):
        if not self.hardware_available:
            QMessageBox.warning(self, "Hardware Error", "No hardware available!")
            return
        try:
            trigger_config = {
                'thresholds': {self.sensor_type: self.threshold},
                'hysteresis': {self.sensor_type: self.hysteresis},
                'trigger_sensors': {'l': self.sensor_type, 'r': self.sensor_type}
            }
            self.sensor_thread = SensorProcessingThread(
                self.reader, trigger_config, self.sensor_type, None, self.debug_printing
            )
            self.sensor_thread.sensor_update.connect(self.on_sensor_update)
            self.sensor_thread.raw_sensor_update.connect(self.on_raw_sensor_update)
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
        if self.sensor_thread:
            self.sensor_thread.stop_processing()
            self.sensor_thread = None
        self.running = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_bar.showMessage("⏹️ Sensors stopped")
    def on_sensor_update(self, hand, finger_idx, sensor_value, triggered, switch_event):
        self.finger_display.update_sensor_value(hand, finger_idx, sensor_value)
        self.finger_display.set_triggered(hand, finger_idx, triggered)
        if self.midi_available and self.midi_writer and switch_event != 0:
            if hand in self.note_maps and finger_idx < len(self.note_maps[hand]):
                note = self.note_maps[hand][finger_idx]
                if switch_event == 1:
                    self.midi_writer.trig_note(note, vel=80)
                    if self.debug_printing:
                        print(f"🎵 MIDI ON: {hand.upper()} finger {finger_idx}, note {note}")
                elif switch_event == -1:
                    self.midi_writer.trig_note(note, vel=0)
                    if self.debug_printing:
                        print(f"🎵 MIDI OFF: {hand.upper()} finger {finger_idx}, note {note}")
    def on_raw_sensor_update(self, hand, raw_sensor_dict):
        # Update IMU plot
        if 'imu' in raw_sensor_dict:
            self.imu_plot.update_plot(hand, raw_sensor_dict['imu'])
        
        # Pitch bend (right hand only)
        if hand == 'r' and 'imu' in raw_sensor_dict:
            imu = raw_sensor_dict['imu']
            if len(imu) > self.pitch_bend_imu_channel:
                pitch_val = imu[self.pitch_bend_imu_channel]
                # Map 0-65535 to -8192 to 8192
                pitch_bend = int((pitch_val / 65535.0) * 16384 - 8192)
                self.midi_writer.pitch_bend(pitch_bend)
    def on_error(self, error_message):
        if self.debug_printing:
            print(f"❌ Sensor error: {error_message}")
        self.status_bar.showMessage(f"Error: {error_message}")
    def update_threshold(self, value):
        self.threshold = value
        self.finger_display.set_threshold(value)
        if self.sensor_thread:
            self.sensor_thread.update_threshold(self.sensor_type, value)
    def update_hysteresis(self, value):
        self.hysteresis = value
        if self.sensor_thread:
            self.sensor_thread.update_hysteresis(self.sensor_type, value)
    def update_sensor_type(self, sensor_type):
        old_sensor_type = self.sensor_type
        self.sensor_type = sensor_type
        self.finger_display.set_sensor_type(sensor_type)
        if sensor_type == 'press':
            self.threshold_spinbox.setValue(25)
            self.hysteresis_spinbox.setValue(5)
        else:
            self.threshold_spinbox.setValue(120)
            self.hysteresis_spinbox.setValue(10)
        if self.running:
            self.status_bar.showMessage(f"🔄 Sensor type changed to {sensor_type} - Restart sensors to apply changes")
        else:
            self.status_bar.showMessage(f"✅ Sensor type changed to {sensor_type}")
        if self.debug_printing:
            print(f"🔄 Sensor type changed from {old_sensor_type} to {sensor_type}")
    def toggle_debug_printing(self, state):
        self.debug_printing = state == Qt.Checked
    def _on_pitch_bend_channel_changed(self, idx):
        self.pitch_bend_imu_channel = idx
    def closeEvent(self, event):
        if self.running:
            self.stop_sensors()
        event.accept()

def main():
    print("🚀 Starting GLUVN Ten Finger GUI with Pitch Bend (IMU2)")
    print("=" * 70)
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = TenFingerPitchBendGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main() 