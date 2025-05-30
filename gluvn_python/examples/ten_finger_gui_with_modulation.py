#!/usr/bin/env python3
"""
Ten Finger Hardware GUI with IMU Modulation Control

Enhanced version of the ten finger GUI that includes IMU-based volume and pitch
control using the GLUVN modulation strategy system. This demonstrates accelerometer
volume control, pitch bend, and advanced modulation features.

Features:
- Real-time sensor value display (flex/pressure values)
- Visual finger lights showing trigger states
- IMU-based volume control using accelerometer magnitude
- IMU-based pitch bend control
- Moving window averaging control
- Choir control for multi-voice effects
- Configurable modulation strategies
- MIDI output for triggered notes and continuous control

Author: Joseph Bakarji
Usage: python ten_finger_gui_with_modulation.py
"""

import sys
import os
import json

# PyQt5 imports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGroupBox, QSpinBox, QComboBox, 
                            QPushButton, QLabel, QStatusBar, QMessageBox,
                            QCheckBox, QSlider, QGridLayout, QFileDialog)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import GLUVN components
from visualization import (TenFingerDisplay, SensorProcessingThread,
                          AccelVolumeModulation, IMUPitchBendModulation,
                          MovingWindowModulation, ChoirModulation,
                          CompositeModulation)
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


class TenFingerModulationGUI(QMainWindow):
    """Main GUI application with real-time sensor display and IMU modulation control"""
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("GLUVN Ten Finger GUI - IMU Modulation Control")
        self.setGeometry(100, 100, 1400, 900)
        
        # Application state
        self.running = False
        self.sensor_thread = None
        self.reader = None
        
        # Configuration
        self.threshold = 120
        self.hysteresis = 10
        self.sensor_type = 'flex'
        
        # Modulation configuration
        self.volume_enabled = True
        self.pitch_bend_enabled = True
        self.window_control_enabled = False
        self.choir_control_enabled = False
        
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
        """Create the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # Title
        title = QLabel("GLUVN Ten Finger GUI - IMU Modulation Control")
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
        
        # Modulation control panel
        self._create_modulation_panel(main_layout)
        
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
        self.status_bar.showMessage("Ready - Configure modulation and click 'Start Sensors'")
        self.setStatusBar(self.status_bar)
    
    def _get_status_info(self):
        """Get status text and color based on system state"""
        if self.hardware_available:
            if self.midi_available:
                return "✅ Hardware + MIDI Connected - Full functionality with modulation", "lightgreen"
            else:
                return "⚠️ Hardware Connected, MIDI Unavailable - Sensor display only", "lightyellow"
        else:
            return "❌ Hardware Not Available - Check connection and restart", "lightcoral"
    
    def _create_control_panel(self, main_layout):
        """Create control panel with configuration options"""
        control_group = QGroupBox("Trigger Controls & Configuration")
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
        
        # Configuration save/load buttons
        config_row = QHBoxLayout()
        self.save_config_button = QPushButton("Save Config")
        self.save_config_button.clicked.connect(self.save_configuration)
        config_row.addWidget(self.save_config_button)
        
        self.load_config_button = QPushButton("Load Config")
        self.load_config_button.clicked.connect(self.load_configuration)
        config_row.addWidget(self.load_config_button)
        
        # Preset buttons
        self.preset_basic_button = QPushButton("Basic Preset")
        self.preset_basic_button.clicked.connect(lambda: self.load_preset('basic'))
        config_row.addWidget(self.preset_basic_button)
        
        self.preset_advanced_button = QPushButton("Advanced Preset")
        self.preset_advanced_button.clicked.connect(lambda: self.load_preset('advanced'))
        config_row.addWidget(self.preset_advanced_button)
        
        button_layout.addLayout(config_row)
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
    
    def _create_modulation_panel(self, main_layout):
        """Create modulation control panel"""
        modulation_group = QGroupBox("IMU Modulation Controls")
        main_layout.addWidget(modulation_group)
        
        mod_layout = QGridLayout(modulation_group)
        
        # Volume control (accelerometer)
        self.volume_checkbox = QCheckBox("Accelerometer Volume Control")
        self.volume_checkbox.setChecked(self.volume_enabled)
        self.volume_checkbox.toggled.connect(self.toggle_volume_control)
        mod_layout.addWidget(self.volume_checkbox, 0, 0, 1, 3)
        
        mod_layout.addWidget(QLabel("Base Volume:"), 1, 0)
        self.base_volume_slider = QSlider(Qt.Horizontal)
        self.base_volume_slider.setRange(0, 100)
        self.base_volume_slider.setValue(20)
        self.base_volume_label = QLabel("20")
        mod_layout.addWidget(self.base_volume_slider, 1, 1)
        mod_layout.addWidget(self.base_volume_label, 1, 2)
        self.base_volume_slider.valueChanged.connect(lambda v: self.base_volume_label.setText(str(v)))
        
        mod_layout.addWidget(QLabel("Volume Smoothing:"), 2, 0)
        self.volume_smoothing_slider = QSlider(Qt.Horizontal)
        self.volume_smoothing_slider.setRange(1, 50)
        self.volume_smoothing_slider.setValue(10)
        self.volume_smoothing_label = QLabel("10")
        mod_layout.addWidget(self.volume_smoothing_slider, 2, 1)
        mod_layout.addWidget(self.volume_smoothing_label, 2, 2)
        self.volume_smoothing_slider.valueChanged.connect(lambda v: self.volume_smoothing_label.setText(str(v)))
        
        mod_layout.addWidget(QLabel("Volume Sensitivity:"), 3, 0)
        self.volume_sensitivity_slider = QSlider(Qt.Horizontal)
        self.volume_sensitivity_slider.setRange(5000, 30000)
        self.volume_sensitivity_slider.setValue(15000)
        self.volume_sensitivity_label = QLabel("15000")
        mod_layout.addWidget(self.volume_sensitivity_slider, 3, 1)
        mod_layout.addWidget(self.volume_sensitivity_label, 3, 2)
        self.volume_sensitivity_slider.valueChanged.connect(lambda v: self.volume_sensitivity_label.setText(str(v)))
        
        # Pitch bend control (IMU)
        self.pitch_bend_checkbox = QCheckBox("IMU Pitch Bend Control")
        self.pitch_bend_checkbox.setChecked(self.pitch_bend_enabled)
        self.pitch_bend_checkbox.toggled.connect(self.toggle_pitch_bend_control)
        mod_layout.addWidget(self.pitch_bend_checkbox, 4, 0, 1, 3)
        
        mod_layout.addWidget(QLabel("Pitch Bend Range:"), 5, 0)
        self.pitch_range_slider = QSlider(Qt.Horizontal)
        self.pitch_range_slider.setRange(1000, 8192)
        self.pitch_range_slider.setValue(8192)
        self.pitch_range_label = QLabel("8192")
        mod_layout.addWidget(self.pitch_range_slider, 5, 1)
        mod_layout.addWidget(self.pitch_range_label, 5, 2)
        self.pitch_range_slider.valueChanged.connect(lambda v: self.pitch_range_label.setText(str(v)))
        
        mod_layout.addWidget(QLabel("Pitch Smoothing:"), 6, 0)
        self.pitch_smoothing_slider = QSlider(Qt.Horizontal)
        self.pitch_smoothing_slider.setRange(1, 20)
        self.pitch_smoothing_slider.setValue(5)
        self.pitch_smoothing_label = QLabel("5")
        mod_layout.addWidget(self.pitch_smoothing_slider, 6, 1)
        mod_layout.addWidget(self.pitch_smoothing_label, 6, 2)
        self.pitch_smoothing_slider.valueChanged.connect(lambda v: self.pitch_smoothing_label.setText(str(v)))
        
        mod_layout.addWidget(QLabel("Pitch Mode:"), 7, 0)
        self.pitch_mode_combo = QComboBox()
        self.pitch_mode_combo.addItems(['modulo', 'linear'])
        self.pitch_mode_combo.setCurrentText('modulo')
        mod_layout.addWidget(self.pitch_mode_combo, 7, 1, 1, 2)
        
        mod_layout.addWidget(QLabel("Pitch Sensor:"), 8, 0)
        self.pitch_sensor_combo = QComboBox()
        self.pitch_sensor_combo.addItems(['imu1', 'imu2', 'imu3', 'imu4', 'imu5'])
        self.pitch_sensor_combo.setCurrentText('imu2')
        mod_layout.addWidget(self.pitch_sensor_combo, 8, 1, 1, 2)
        
        # Advanced modulation options
        self.window_control_checkbox = QCheckBox("Moving Window Control")
        self.window_control_checkbox.setChecked(self.window_control_enabled)
        self.window_control_checkbox.toggled.connect(self.toggle_window_control)
        mod_layout.addWidget(self.window_control_checkbox, 9, 0, 1, 2)
        
        mod_layout.addWidget(QLabel("Window Range:"), 10, 0)
        window_layout = QHBoxLayout()
        self.min_window_spin = QSpinBox()
        self.min_window_spin.setRange(1, 20)
        self.min_window_spin.setValue(1)
        self.max_window_spin = QSpinBox()
        self.max_window_spin.setRange(10, 100)
        self.max_window_spin.setValue(50)
        window_layout.addWidget(QLabel("Min:"))
        window_layout.addWidget(self.min_window_spin)
        window_layout.addWidget(QLabel("Max:"))
        window_layout.addWidget(self.max_window_spin)
        window_widget = QWidget()
        window_widget.setLayout(window_layout)
        mod_layout.addWidget(window_widget, 10, 1, 1, 2)
        
        mod_layout.addWidget(QLabel("Window Sensor:"), 11, 0)
        self.window_sensor_combo = QComboBox()
        self.window_sensor_combo.addItems(['imu1', 'imu2', 'imu3', 'imu4', 'imu5'])
        self.window_sensor_combo.setCurrentText('imu1')
        mod_layout.addWidget(self.window_sensor_combo, 11, 1, 1, 2)
        
        self.choir_control_checkbox = QCheckBox("Choir Control (Multi-Voice)")
        self.choir_control_checkbox.setChecked(self.choir_control_enabled)
        self.choir_control_checkbox.toggled.connect(self.toggle_choir_control)
        mod_layout.addWidget(self.choir_control_checkbox, 12, 0, 1, 3)
        
        # Modulation status
        self.modulation_status_label = QLabel("Modulation: Volume=OFF, Pitch=OFF")
        self.modulation_status_label.setStyleSheet("color: blue; font-weight: bold;")
        mod_layout.addWidget(self.modulation_status_label, 13, 0, 1, 3)
    
    def _create_modulation_strategies(self):
        """Create modulation strategies based on current configuration"""
        strategies = []
        
        if self.volume_enabled:
            volume_config = {
                'base_volume': self.base_volume_slider.value(),
                'max_volume': 127,
                'accel_sensors': ['imu3', 'imu4', 'imu5'],
                'smoothing_window': self.volume_smoothing_slider.value(),
                'scaling_factor': self.volume_sensitivity_slider.value()
            }
            strategies.append(AccelVolumeModulation(volume_config))
        
        if self.pitch_bend_enabled:
            pitch_config = {
                'pitch_bend_sensor': self.pitch_sensor_combo.currentText(),
                'pitch_bend_range': self.pitch_range_slider.value(),
                'smoothing_factor': self.pitch_smoothing_slider.value() / 10.0,  # Convert to 0.1-2.0 range
                'modulation_mode': self.pitch_mode_combo.currentText()
            }
            strategies.append(IMUPitchBendModulation(pitch_config))
        
        if self.window_control_enabled:
            window_config = {
                'window_controller': self.window_sensor_combo.currentText(),
                'min_window_size': self.min_window_spin.value(),
                'max_window_size': self.max_window_spin.value(),
                'control_parameter': 'averaging_window'
            }
            strategies.append(MovingWindowModulation(window_config))
        
        if self.choir_control_enabled:
            choir_config = {
                'voice_controllers': {
                    'voice1_volume': 'imu1',
                    'voice2_volume': 'imu2',
                    'voice3_volume': 'imu3',
                    'master_pitch': 'imu4'
                },
                'voice_ranges': {
                    'voice1_volume': (0, 127),
                    'voice2_volume': (0, 127),
                    'voice3_volume': (0, 127),
                    'master_pitch': (-8192, 8192)
                }
            }
            strategies.append(ChoirModulation(choir_config))
        
        return strategies
    
    def start_sensors(self):
        """Start sensor processing using proper trigger strategy and modulation"""
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
            
            # Create modulation strategies
            modulation_strategies = self._create_modulation_strategies()
            
            # Initialize sensor processing thread with trigger and modulation strategies
            self.sensor_thread = SensorProcessingThread(
                self.reader, trigger_config, self.sensor_type, modulation_strategies
            )
            
            # Connect signals
            self.sensor_thread.sensor_update.connect(self.on_sensor_update)
            self.sensor_thread.modulation_update.connect(self.on_modulation_update)
            self.sensor_thread.error_signal.connect(self.on_error)
            self.sensor_thread.start_processing()
            
            self.running = True
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            
            status_text = "🔥 Sensors running with IMU modulation - Flex your fingers and move your hands!"
            self.status_bar.showMessage(status_text)
            
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
        self.modulation_status_label.setText("Modulation: Stopped")
    
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
    
    def on_modulation_update(self, hand, modulation_dict):
        """Handle modulation updates from IMU sensors"""
        if not self.midi_available or not self.midi_writer:
            return
        
        # Apply volume control
        if 'volume' in modulation_dict:
            volume = modulation_dict['volume']
            self.midi_writer.aftertouch(int(volume))
        
        # Apply pitch bend
        if 'pitch_bend' in modulation_dict:
            pitch_bend = modulation_dict['pitch_bend']
            self.midi_writer.pitch_bend(int(pitch_bend))
        
        # Update status display
        status_parts = []
        if 'volume' in modulation_dict:
            status_parts.append(f"Vol={int(modulation_dict['volume'])}")
        if 'pitch_bend' in modulation_dict:
            status_parts.append(f"Pitch={int(modulation_dict['pitch_bend'])}")
        if 'averaging_window' in modulation_dict:
            status_parts.append(f"Window={int(modulation_dict['averaging_window'])}")
        
        if status_parts:
            self.modulation_status_label.setText(f"🎛️ {hand.upper()}: {', '.join(status_parts)}")
    
    def on_error(self, error_message):
        """Handle errors from sensor thread"""
        print(f"❌ Sensor error: {error_message}")
        self.status_bar.showMessage(f"Error: {error_message}")
    
    def toggle_volume_control(self, enabled):
        """Toggle accelerometer volume control"""
        self.volume_enabled = enabled
        print(f"🎛️ Volume control: {'enabled' if enabled else 'disabled'}")
    
    def toggle_pitch_bend_control(self, enabled):
        """Toggle IMU pitch bend control"""
        self.pitch_bend_enabled = enabled
        print(f"🎛️ Pitch bend control: {'enabled' if enabled else 'disabled'}")
    
    def toggle_window_control(self, enabled):
        """Toggle moving window control"""
        self.window_control_enabled = enabled
        print(f"🎛️ Window control: {'enabled' if enabled else 'disabled'}")
    
    def toggle_choir_control(self, enabled):
        """Toggle choir control"""
        self.choir_control_enabled = enabled
        print(f"🎛️ Choir control: {'enabled' if enabled else 'disabled'}")
    
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
    
    def save_configuration(self):
        """Save current configuration to file"""
        config = {
            'threshold': self.threshold_spinbox.value(),
            'hysteresis': self.hysteresis_spinbox.value(),
            'sensor_type': self.sensor_combo.currentText(),
            'volume_enabled': self.volume_checkbox.isChecked(),
            'base_volume': self.base_volume_slider.value(),
            'volume_smoothing': self.volume_smoothing_slider.value(),
            'volume_sensitivity': self.volume_sensitivity_slider.value(),
            'pitch_bend_enabled': self.pitch_bend_checkbox.isChecked(),
            'pitch_range': self.pitch_range_slider.value(),
            'pitch_smoothing': self.pitch_smoothing_slider.value(),
            'pitch_mode': self.pitch_mode_combo.currentText(),
            'pitch_sensor': self.pitch_sensor_combo.currentText(),
            'window_control_enabled': self.window_control_checkbox.isChecked(),
            'min_window_size': self.min_window_spin.value(),
            'max_window_size': self.max_window_spin.value(),
            'window_sensor': self.window_sensor_combo.currentText(),
            'choir_control_enabled': self.choir_control_checkbox.isChecked()
        }
        
        filename, _ = QFileDialog.getSaveFileName(
            self, 'Save Configuration', 'gluvn_config.json', 'JSON files (*.json)'
        )
        
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(config, f, indent=2)
                QMessageBox.information(self, "Success", f"Configuration saved to {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save configuration: {e}")
    
    def load_configuration(self):
        """Load configuration from file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, 'Load Configuration', '', 'JSON files (*.json)'
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    config = json.load(f)
                
                self._apply_configuration(config)
                QMessageBox.information(self, "Success", f"Configuration loaded from {filename}")
                
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load configuration: {e}")
    
    def load_preset(self, preset_name):
        """Load a predefined preset configuration"""
        if preset_name == 'basic':
            config = {
                'volume_enabled': True,
                'base_volume': 20,
                'volume_smoothing': 10,
                'volume_sensitivity': 15000,
                'pitch_bend_enabled': True,
                'pitch_range': 4096,
                'pitch_smoothing': 5,
                'pitch_mode': 'modulo',
                'pitch_sensor': 'imu2',
                'window_control_enabled': False,
                'choir_control_enabled': False
            }
        elif preset_name == 'advanced':
            config = {
                'volume_enabled': True,
                'base_volume': 10,
                'volume_smoothing': 5,
                'volume_sensitivity': 20000,
                'pitch_bend_enabled': True,
                'pitch_range': 8192,
                'pitch_smoothing': 3,
                'pitch_mode': 'linear',
                'pitch_sensor': 'imu1',
                'window_control_enabled': True,
                'min_window_size': 2,
                'max_window_size': 30,
                'window_sensor': 'imu3',
                'choir_control_enabled': True
            }
        else:
            QMessageBox.warning(self, "Error", f"Unknown preset: {preset_name}")
            return
        
        self._apply_configuration(config)
        QMessageBox.information(self, "Preset Loaded", f"Applied {preset_name} preset configuration")
    
    def _apply_configuration(self, config):
        """Apply configuration to GUI controls"""
        if 'threshold' in config:
            self.threshold_spinbox.setValue(config['threshold'])
        if 'hysteresis' in config:
            self.hysteresis_spinbox.setValue(config['hysteresis'])
        if 'sensor_type' in config:
            self.sensor_combo.setCurrentText(config['sensor_type'])
        
        if 'volume_enabled' in config:
            self.volume_checkbox.setChecked(config['volume_enabled'])
        if 'base_volume' in config:
            self.base_volume_slider.setValue(config['base_volume'])
        if 'volume_smoothing' in config:
            self.volume_smoothing_slider.setValue(config['volume_smoothing'])
        if 'volume_sensitivity' in config:
            self.volume_sensitivity_slider.setValue(config['volume_sensitivity'])
        
        if 'pitch_bend_enabled' in config:
            self.pitch_bend_checkbox.setChecked(config['pitch_bend_enabled'])
        if 'pitch_range' in config:
            self.pitch_range_slider.setValue(config['pitch_range'])
        if 'pitch_smoothing' in config:
            self.pitch_smoothing_slider.setValue(config['pitch_smoothing'])
        if 'pitch_mode' in config:
            self.pitch_mode_combo.setCurrentText(config['pitch_mode'])
        if 'pitch_sensor' in config:
            self.pitch_sensor_combo.setCurrentText(config['pitch_sensor'])
        
        if 'window_control_enabled' in config:
            self.window_control_checkbox.setChecked(config['window_control_enabled'])
        if 'min_window_size' in config:
            self.min_window_spin.setValue(config['min_window_size'])
        if 'max_window_size' in config:
            self.max_window_spin.setValue(config['max_window_size'])
        if 'window_sensor' in config:
            self.window_sensor_combo.setCurrentText(config['window_sensor'])
        
        if 'choir_control_enabled' in config:
            self.choir_control_checkbox.setChecked(config['choir_control_enabled'])
    
    def closeEvent(self, event):
        """Handle application close"""
        if self.running:
            self.stop_sensors()
        event.accept()


def main():
    """Main application entry point"""
    print("🚀 Starting GLUVN Ten Finger GUI with IMU Modulation Control")
    print("=" * 80)
    print("🎛️  This version includes:")
    print("   • Accelerometer-based volume control")
    print("   • IMU-based pitch bend control")
    print("   • Moving window averaging control")
    print("   • Choir control for multi-voice effects")
    print("   • Real-time modulation with proper GLUVN strategy system")
    print("=" * 80)
    
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = TenFingerModulationGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 