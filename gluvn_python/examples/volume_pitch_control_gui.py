#!/usr/bin/env python3
"""
GLUVN Volume/Pitch Control GUI

This application provides flexible volume and pitch control using IMU sensors,
similar to the original app_10fig_accel.py and app_10fig_pitch.py but with
enhanced parameter control and sensor assignment flexibility.

Features:
- Choose which sensor controls volume (accelerometer magnitude, IMU axes)
- Choose which sensor controls pitch bend (any IMU sensor)
- Real-time quantification display of acceleration and pitch values
- Full parameter control for sensitivity, smoothing, ranges
- Finger triggering with configurable sensors
- Save/load configurations

Author: Joseph Bakarji
Usage: python volume_pitch_control_gui.py
"""

import sys
import os
import json

# PyQt5 imports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGroupBox, QSpinBox, QComboBox, 
                            QPushButton, QLabel, QStatusBar, QMessageBox,
                            QCheckBox, QSlider, QGridLayout, QFileDialog,
                            QProgressBar, QLCDNumber, QFrame)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPalette

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import GLUVN components
from visualization import (TenFingerDisplay, SensorProcessingThread,
                          AccelVolumeModulation, IMUPitchBendModulation,
                          MovingWindowModulation)
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


class VolumePitchControlGUI(QMainWindow):
    """Main GUI for volume/pitch control with flexible sensor assignment"""
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("GLUVN Volume/Pitch Control - Enhanced Parameter Control")
        self.setGeometry(100, 100, 1600, 1000)
        
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
        
        # Current sensor values for display
        self.current_volume = 0
        self.current_pitch_bend = 0
        self.current_accel_magnitude = 0.0
        self.current_imu_values = {'imu1': 0, 'imu2': 0, 'imu3': 0, 'imu4': 0, 'imu5': 0}
        self.raw_sensor_update_count = 0
        
        # Initialize MIDI and mapping
        self._initialize_midi_system()
        
        # Initialize hardware
        self._initialize_hardware()
        
        # Setup UI
        self._setup_ui()
        
        # Timer for updating displays
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_displays)
        self.update_timer.start(100)  # Update every 100ms
    
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
        
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(15)
        
        # Left panel: Controls
        left_panel = QVBoxLayout()
        
        # Title
        title = QLabel("GLUVN Volume/Pitch Control")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Bold))
        left_panel.addWidget(title)
        
        # Status info
        status_text, status_color = self._get_status_info()
        status_label = QLabel(status_text)
        status_label.setAlignment(Qt.AlignCenter)
        status_label.setStyleSheet(f"background-color: {status_color}; padding: 8px; border-radius: 4px; font-weight: bold;")
        left_panel.addWidget(status_label)
        
        # Control panels
        self._create_trigger_control_panel(left_panel)
        self._create_volume_control_panel(left_panel)
        self._create_pitch_control_panel(left_panel)
        self._create_advanced_control_panel(left_panel)
        self._create_action_buttons(left_panel)
        
        main_layout.addLayout(left_panel, 1)
        
        # Middle panel: Real-time displays
        middle_panel = QVBoxLayout()
        self._create_realtime_displays(middle_panel)
        main_layout.addLayout(middle_panel, 1)
        
        # Right panel: Finger display
        right_panel = QVBoxLayout()
        self._create_finger_display(right_panel)
        main_layout.addLayout(right_panel, 1)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.status_bar.showMessage("Ready - Configure sensors and click 'Start System'")
        self.setStatusBar(self.status_bar)
    
    def _get_status_info(self):
        """Get status text and color based on system state"""
        if self.hardware_available:
            if self.midi_available:
                return "✅ Hardware + MIDI Ready", "lightgreen"
            else:
                return "⚠️ Hardware Ready, MIDI Unavailable", "lightyellow"
        else:
            return "❌ Hardware Not Available", "lightcoral"
    
    def _create_trigger_control_panel(self, parent_layout):
        """Create trigger control panel"""
        trigger_group = QGroupBox("Finger Trigger Controls")
        parent_layout.addWidget(trigger_group)
        
        layout = QGridLayout(trigger_group)
        
        layout.addWidget(QLabel("Trigger Sensor:"), 0, 0)
        self.trigger_sensor_combo = QComboBox()
        self.trigger_sensor_combo.addItems(['flex', 'press'])
        self.trigger_sensor_combo.setCurrentText(self.sensor_type)
        layout.addWidget(self.trigger_sensor_combo, 0, 1)
        
        layout.addWidget(QLabel("Threshold:"), 1, 0)
        self.threshold_spinbox = QSpinBox()
        self.threshold_spinbox.setRange(50, 500)
        self.threshold_spinbox.setValue(self.threshold)
        layout.addWidget(self.threshold_spinbox, 1, 1)
        
        layout.addWidget(QLabel("Hysteresis:"), 2, 0)
        self.hysteresis_spinbox = QSpinBox()
        self.hysteresis_spinbox.setRange(1, 50)
        self.hysteresis_spinbox.setValue(self.hysteresis)
        layout.addWidget(self.hysteresis_spinbox, 2, 1)
    
    def _create_volume_control_panel(self, parent_layout):
        """Create volume control panel"""
        volume_group = QGroupBox("Volume Control Configuration")
        parent_layout.addWidget(volume_group)
        
        layout = QGridLayout(volume_group)
        
        # Enable volume control
        self.volume_enabled = QCheckBox("Enable Volume Control")
        self.volume_enabled.setChecked(True)
        layout.addWidget(self.volume_enabled, 0, 0, 1, 2)
        
        # Volume controller selection
        layout.addWidget(QLabel("Volume Controller:"), 1, 0)
        self.volume_controller_combo = QComboBox()
        self.volume_controller_combo.addItems([
            'accel_mag',  # Accelerometer magnitude (like original)
            'imu1', 'imu2', 'imu3', 'imu4', 'imu5'  # Individual IMU sensors
        ])
        self.volume_controller_combo.setCurrentText('accel_mag')
        layout.addWidget(self.volume_controller_combo, 1, 1)
        
        # Base volume
        layout.addWidget(QLabel("Base Volume:"), 2, 0)
        self.base_volume_slider = QSlider(Qt.Horizontal)
        self.base_volume_slider.setRange(0, 100)
        self.base_volume_slider.setValue(20)
        self.base_volume_label = QLabel("20")
        layout.addWidget(self.base_volume_slider, 2, 1)
        layout.addWidget(self.base_volume_label, 2, 2)
        self.base_volume_slider.valueChanged.connect(lambda v: self.base_volume_label.setText(str(v)))
        
        # Volume sensitivity
        layout.addWidget(QLabel("Volume Sensitivity:"), 3, 0)
        self.volume_sensitivity_slider = QSlider(Qt.Horizontal)
        self.volume_sensitivity_slider.setRange(5000, 30000)
        self.volume_sensitivity_slider.setValue(15000)
        self.volume_sensitivity_label = QLabel("15000")
        layout.addWidget(self.volume_sensitivity_slider, 3, 1)
        layout.addWidget(self.volume_sensitivity_label, 3, 2)
        self.volume_sensitivity_slider.valueChanged.connect(lambda v: self.volume_sensitivity_label.setText(str(v)))
        
        # Volume smoothing
        layout.addWidget(QLabel("Volume Smoothing:"), 4, 0)
        self.volume_smoothing_slider = QSlider(Qt.Horizontal)
        self.volume_smoothing_slider.setRange(1, 50)
        self.volume_smoothing_slider.setValue(10)
        self.volume_smoothing_label = QLabel("10")
        layout.addWidget(self.volume_smoothing_slider, 4, 1)
        layout.addWidget(self.volume_smoothing_label, 4, 2)
        self.volume_smoothing_slider.valueChanged.connect(lambda v: self.volume_smoothing_label.setText(str(v)))
    
    def _create_pitch_control_panel(self, parent_layout):
        """Create pitch control panel"""
        pitch_group = QGroupBox("Pitch Bend Configuration")
        parent_layout.addWidget(pitch_group)
        
        layout = QGridLayout(pitch_group)
        
        # Enable pitch control
        self.pitch_enabled = QCheckBox("Enable Pitch Bend Control")
        self.pitch_enabled.setChecked(True)
        layout.addWidget(self.pitch_enabled, 0, 0, 1, 2)
        
        # Pitch controller selection
        layout.addWidget(QLabel("Pitch Controller:"), 1, 0)
        self.pitch_controller_combo = QComboBox()
        self.pitch_controller_combo.addItems(['imu1', 'imu2', 'imu3', 'imu4', 'imu5'])
        self.pitch_controller_combo.setCurrentText('imu2')
        layout.addWidget(self.pitch_controller_combo, 1, 1)
        
        # Pitch bend range
        layout.addWidget(QLabel("Pitch Bend Range:"), 2, 0)
        self.pitch_range_slider = QSlider(Qt.Horizontal)
        self.pitch_range_slider.setRange(1000, 8192)
        self.pitch_range_slider.setValue(8192)
        self.pitch_range_label = QLabel("8192")
        layout.addWidget(self.pitch_range_slider, 2, 1)
        layout.addWidget(self.pitch_range_label, 2, 2)
        self.pitch_range_slider.valueChanged.connect(lambda v: self.pitch_range_label.setText(str(v)))
        
        # Pitch mode
        layout.addWidget(QLabel("Pitch Mode:"), 3, 0)
        self.pitch_mode_combo = QComboBox()
        self.pitch_mode_combo.addItems(['modulo', 'linear'])
        self.pitch_mode_combo.setCurrentText('modulo')
        layout.addWidget(self.pitch_mode_combo, 3, 1)
        
        # Pitch smoothing
        layout.addWidget(QLabel("Pitch Smoothing:"), 4, 0)
        self.pitch_smoothing_slider = QSlider(Qt.Horizontal)
        self.pitch_smoothing_slider.setRange(1, 20)
        self.pitch_smoothing_slider.setValue(5)
        self.pitch_smoothing_label = QLabel("5")
        layout.addWidget(self.pitch_smoothing_slider, 4, 1)
        layout.addWidget(self.pitch_smoothing_label, 4, 2)
        self.pitch_smoothing_slider.valueChanged.connect(lambda v: self.pitch_smoothing_label.setText(str(v)))
    
    def _create_advanced_control_panel(self, parent_layout):
        """Create advanced control panel"""
        advanced_group = QGroupBox("Advanced Controls")
        parent_layout.addWidget(advanced_group)
        
        layout = QGridLayout(advanced_group)
        
        # Window averaging control
        self.window_control_enabled = QCheckBox("Moving Window Control")
        self.window_control_enabled.setChecked(False)
        layout.addWidget(self.window_control_enabled, 0, 0, 1, 2)
        
        layout.addWidget(QLabel("Window Controller:"), 1, 0)
        self.window_controller_combo = QComboBox()
        self.window_controller_combo.addItems(['imu1', 'imu2', 'imu3', 'imu4', 'imu5'])
        self.window_controller_combo.setCurrentText('imu1')
        layout.addWidget(self.window_controller_combo, 1, 1)
        
        # Configuration management
        config_layout = QHBoxLayout()
        
        save_button = QPushButton("Save Config")
        save_button.clicked.connect(self.save_configuration)
        config_layout.addWidget(save_button)
        
        load_button = QPushButton("Load Config")
        load_button.clicked.connect(self.load_configuration)
        config_layout.addWidget(load_button)
        
        layout.addLayout(config_layout, 2, 0, 1, 2)
        
        # Debug printing option
        self.debug_checkbox = QCheckBox("Enable Debug Printing")
        self.debug_checkbox.setChecked(self.debug_printing)
        self.debug_checkbox.stateChanged.connect(self.toggle_debug_printing)
        layout.addWidget(self.debug_checkbox, 3, 0, 1, 2)
        
        # Preset buttons
        preset_layout = QHBoxLayout()
        
        basic_preset_button = QPushButton("Basic Preset")
        basic_preset_button.clicked.connect(lambda: self.load_preset('basic'))
        preset_layout.addWidget(basic_preset_button)
        
        advanced_preset_button = QPushButton("Advanced Preset")
        advanced_preset_button.clicked.connect(lambda: self.load_preset('advanced'))
        preset_layout.addWidget(advanced_preset_button)
        
        layout.addLayout(preset_layout, 4, 0, 1, 2)
    
    def _create_action_buttons(self, parent_layout):
        """Create main action buttons"""
        button_layout = QHBoxLayout()
        
        self.start_button = QPushButton("Start System")
        self.start_button.clicked.connect(self.start_system)
        self.start_button.setEnabled(self.hardware_available)
        self.start_button.setStyleSheet("QPushButton { background-color: lightgreen; font-weight: bold; padding: 10px; }")
        button_layout.addWidget(self.start_button)
        
        self.stop_button = QPushButton("Stop System")
        self.stop_button.clicked.connect(self.stop_system)
        self.stop_button.setEnabled(False)
        self.stop_button.setStyleSheet("QPushButton { background-color: lightcoral; font-weight: bold; padding: 10px; }")
        button_layout.addWidget(self.stop_button)
        
        parent_layout.addLayout(button_layout)
    
    def _create_realtime_displays(self, parent_layout):
        """Create real-time value displays"""
        display_group = QGroupBox("Real-Time Sensor Values")
        parent_layout.addWidget(display_group)
        
        layout = QVBoxLayout(display_group)
        
        # Volume display
        volume_frame = QFrame()
        volume_frame.setFrameStyle(QFrame.Box)
        volume_layout = QVBoxLayout(volume_frame)
        
        volume_layout.addWidget(QLabel("VOLUME CONTROL"))
        self.volume_lcd = QLCDNumber(3)
        self.volume_lcd.setStyleSheet("QLCDNumber { background-color: darkgreen; color: lime; }")
        volume_layout.addWidget(self.volume_lcd)
        
        self.volume_progress = QProgressBar()
        self.volume_progress.setRange(0, 127)
        self.volume_progress.setStyleSheet("QProgressBar::chunk { background-color: lime; }")
        volume_layout.addWidget(self.volume_progress)
        
        self.volume_controller_label = QLabel("Controller: accel_mag")
        self.volume_controller_label.setAlignment(Qt.AlignCenter)
        volume_layout.addWidget(self.volume_controller_label)
        
        layout.addWidget(volume_frame)
        
        # Pitch bend display
        pitch_frame = QFrame()
        pitch_frame.setFrameStyle(QFrame.Box)
        pitch_layout = QVBoxLayout(pitch_frame)
        
        pitch_layout.addWidget(QLabel("PITCH BEND CONTROL"))
        self.pitch_lcd = QLCDNumber(5)
        self.pitch_lcd.setStyleSheet("QLCDNumber { background-color: darkblue; color: cyan; }")
        pitch_layout.addWidget(self.pitch_lcd)
        
        self.pitch_progress = QProgressBar()
        self.pitch_progress.setRange(-8192, 8192)
        self.pitch_progress.setStyleSheet("QProgressBar::chunk { background-color: cyan; }")
        pitch_layout.addWidget(self.pitch_progress)
        
        self.pitch_controller_label = QLabel("Controller: imu2")
        self.pitch_controller_label.setAlignment(Qt.AlignCenter)
        pitch_layout.addWidget(self.pitch_controller_label)
        
        layout.addWidget(pitch_frame)
        
        # Accelerometer magnitude display
        accel_frame = QFrame()
        accel_frame.setFrameStyle(QFrame.Box)
        accel_layout = QVBoxLayout(accel_frame)
        
        accel_layout.addWidget(QLabel("ACCELEROMETER MAGNITUDE"))
        self.accel_lcd = QLCDNumber(5)
        self.accel_lcd.setStyleSheet("QLCDNumber { background-color: darkred; color: yellow; }")
        accel_layout.addWidget(self.accel_lcd)
        
        self.accel_progress = QProgressBar()
        self.accel_progress.setRange(0, 20000)
        self.accel_progress.setStyleSheet("QProgressBar::chunk { background-color: yellow; }")
        accel_layout.addWidget(self.accel_progress)
        
        layout.addWidget(accel_frame)
        
        # Individual IMU sensor values
        imu_frame = QFrame()
        imu_frame.setFrameStyle(QFrame.Box)
        imu_layout = QGridLayout(imu_frame)
        
        imu_layout.addWidget(QLabel("IMU SENSOR VALUES"), 0, 0, 1, 5)
        
        self.imu_lcds = {}
        self.imu_progress_bars = {}
        
        for i, sensor in enumerate(['imu1', 'imu2', 'imu3', 'imu4', 'imu5']):
            imu_layout.addWidget(QLabel(sensor.upper()), 1, i)
            
            lcd = QLCDNumber(5)
            lcd.setStyleSheet("QLCDNumber { background-color: black; color: white; }")
            self.imu_lcds[sensor] = lcd
            imu_layout.addWidget(lcd, 2, i)
            
            progress = QProgressBar()
            progress.setRange(0, 65535)
            progress.setOrientation(Qt.Vertical)
            self.imu_progress_bars[sensor] = progress
            imu_layout.addWidget(progress, 3, i)
        
        layout.addWidget(imu_frame)
    
    def _create_finger_display(self, parent_layout):
        """Create finger display"""
        finger_group = QGroupBox("Finger Sensors")
        parent_layout.addWidget(finger_group)
        
        finger_layout = QVBoxLayout(finger_group)
        
        # Ten finger display
        self.finger_display = TenFingerDisplay(
            note_maps=self.note_maps, 
            sensor_type=self.sensor_type
        )
        self.finger_display.update_note_maps(self.note_maps, self.note_mapper)
        self.finger_display.set_threshold(self.threshold)
        finger_layout.addWidget(self.finger_display)
    
    def _update_displays(self):
        """Update real-time displays"""
        # Update volume display
        self.volume_lcd.display(self.current_volume)
        self.volume_progress.setValue(self.current_volume)
        self.volume_controller_label.setText(f"Controller: {self.volume_controller_combo.currentText()}")
        
        # Update pitch display
        self.pitch_lcd.display(self.current_pitch_bend)
        self.pitch_progress.setValue(self.current_pitch_bend)
        self.pitch_controller_label.setText(f"Controller: {self.pitch_controller_combo.currentText()}")
        
        # Update accelerometer magnitude
        self.accel_lcd.display(int(self.current_accel_magnitude))
        self.accel_progress.setValue(int(self.current_accel_magnitude))
        
        # Update individual IMU values
        for sensor, value in self.current_imu_values.items():
            if sensor in self.imu_lcds:
                self.imu_lcds[sensor].display(value)
                self.imu_progress_bars[sensor].setValue(value)
    
    def start_system(self):
        """Start the volume/pitch control system"""
        if not self.hardware_available:
            QMessageBox.warning(self, "Hardware Error", "No hardware available!")
            return
        
        try:
            # Create trigger strategy configuration
            trigger_config = {
                'thresholds': {self.trigger_sensor_combo.currentText(): self.threshold_spinbox.value()},
                'hysteresis': {self.trigger_sensor_combo.currentText(): self.hysteresis_spinbox.value()},
                'trigger_sensors': {'l': self.trigger_sensor_combo.currentText(), 'r': self.trigger_sensor_combo.currentText()}
            }
            
            # Create modulation strategies based on current configuration
            modulation_strategies = self._create_modulation_strategies()
            
            # Initialize sensor processing thread
            self.sensor_thread = SensorProcessingThread(
                self.reader, trigger_config, self.trigger_sensor_combo.currentText(), modulation_strategies, self.debug_printing
            )
            
            # Connect signals
            self.sensor_thread.sensor_update.connect(self.on_sensor_update)
            self.sensor_thread.modulation_update.connect(self.on_modulation_update)
            self.sensor_thread.raw_sensor_update.connect(self.on_raw_sensor_update)
            self.sensor_thread.error_signal.connect(self.on_error)
            self.sensor_thread.start_processing()
            
            self.running = True
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            
            self.status_bar.showMessage("🔥 Volume/Pitch Control Active - Play and move your hands!")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start system: {e}")
            print(f"❌ Error starting system: {e}")
    
    def stop_system(self):
        """Stop the volume/pitch control system"""
        if self.sensor_thread:
            self.sensor_thread.stop_processing()
            self.sensor_thread = None
        
        self.running = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_bar.showMessage("⏹️ System stopped")
    
    def _create_modulation_strategies(self):
        """Create modulation strategies based on current configuration"""
        strategies = []
        
        if self.volume_enabled.isChecked():
            if self.volume_controller_combo.currentText() == 'accel_mag':
                # Use accelerometer magnitude (original implementation)
                volume_config = {
                    'base_volume': self.base_volume_slider.value(),
                    'max_volume': 127,
                    'accel_sensors': ['imu3', 'imu4', 'imu5'],
                    'smoothing_window': self.volume_smoothing_slider.value(),
                    'scaling_factor': self.volume_sensitivity_slider.value()
                }
                strategies.append(AccelVolumeModulation(volume_config))
            else:
                # Use individual IMU sensor (custom implementation)
                volume_config = {
                    'base_volume': self.base_volume_slider.value(),
                    'max_volume': 127,
                    'accel_sensors': [self.volume_controller_combo.currentText()],  # Single sensor
                    'smoothing_window': self.volume_smoothing_slider.value(),
                    'scaling_factor': self.volume_sensitivity_slider.value()
                }
                strategies.append(AccelVolumeModulation(volume_config))
        
        if self.pitch_enabled.isChecked():
            pitch_config = {
                'pitch_bend_sensor': self.pitch_controller_combo.currentText(),
                'pitch_bend_range': self.pitch_range_slider.value(),
                'smoothing_factor': self.pitch_smoothing_slider.value() / 10.0,
                'modulation_mode': self.pitch_mode_combo.currentText()
            }
            strategies.append(IMUPitchBendModulation(pitch_config))
        
        if self.window_control_enabled.isChecked():
            window_config = {
                'window_controller': self.window_controller_combo.currentText(),
                'min_window_size': 1,
                'max_window_size': 50,
                'control_parameter': 'averaging_window'
            }
            strategies.append(MovingWindowModulation(window_config))
        
        return strategies
    
    def on_sensor_update(self, hand, finger_idx, sensor_value, triggered, switch_event):
        """Handle sensor data updates"""
        # Update finger display
        self.finger_display.update_sensor_value(hand, finger_idx, sensor_value)
        self.finger_display.set_triggered(hand, finger_idx, triggered)
        
        # Play MIDI note on trigger events
        if self.midi_available and self.midi_writer and switch_event != 0:
            if hand in self.note_maps and finger_idx < len(self.note_maps[hand]):
                note = self.note_maps[hand][finger_idx]
                
                if self.debug_printing:
                    if switch_event == 1:
                        print(f"🎵 MIDI ON: {hand.upper()} finger {finger_idx}, note {note}")
                    elif switch_event == -1:
                        print(f"🎵 MIDI OFF: {hand.upper()} finger {finger_idx}, note {note}")
                if switch_event == 1:
                    self.midi_writer.trig_note(note, vel=80)
                elif switch_event == -1:
                    self.midi_writer.trig_note(note, vel=0)
    
    def on_modulation_update(self, hand, modulation_dict):
        """Handle modulation updates"""
        if not self.midi_available or not self.midi_writer:
            return
        
        # Update current values for display
        if 'volume' in modulation_dict:
            self.current_volume = modulation_dict['volume']
            self.midi_writer.aftertouch(int(self.current_volume))
        
        if 'pitch_bend' in modulation_dict:
            self.current_pitch_bend = modulation_dict['pitch_bend']
            self.midi_writer.pitch_bend(int(self.current_pitch_bend))
    
    def on_raw_sensor_update(self, hand, raw_sensor_dict):
        """Handle raw sensor data for quantification displays"""
        # Handle the actual IMU data structure from raw sensor data
        # In raw sensor data, IMU is stored as an array under the 'imu' key: [yaw, pitch, roll, gx, gy, gz]
        if 'imu' in raw_sensor_dict and isinstance(raw_sensor_dict['imu'], (list, tuple)):
            imu_array = raw_sensor_dict['imu']
            
            # Map array indices to IMU sensor names based on original app structure
            # Based on data_analysis.py: [yaw, pitch, roll, gx, gy, gz] = indices [0, 1, 2, 3, 4, 5]
            # For original app compatibility: imu1=yaw, imu2=pitch, imu3=gx, imu4=gy, imu5=gz
            if len(imu_array) >= 6:
                self.current_imu_values['imu1'] = int(imu_array[0])  # yaw
                self.current_imu_values['imu2'] = int(imu_array[1])  # pitch
                self.current_imu_values['imu3'] = int(imu_array[3])  # gx (accelerometer X)
                self.current_imu_values['imu4'] = int(imu_array[4])  # gy (accelerometer Y)
                self.current_imu_values['imu5'] = int(imu_array[5])  # gz (accelerometer Z)
            
        # Update accelerometer magnitude calculation using current IMU values
        # Use the same formula as app_10fig_accel.py
        if all(sensor in self.current_imu_values for sensor in ['imu3', 'imu4', 'imu5']):
            TWO_BYTE = 65535
            ZERO_ACCEL = TWO_BYTE / 4.0 - 680.0
            
            accel_x = self.current_imu_values['imu3'] - TWO_BYTE / 2.0
            accel_y = self.current_imu_values['imu4'] - TWO_BYTE / 2.0
            accel_z = self.current_imu_values['imu5'] - TWO_BYTE / 2.0
            
            import numpy as np
            norm = np.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
            magnitude = max(norm - ZERO_ACCEL, 0)
            self.current_accel_magnitude = magnitude
        
        # Update counter for debug output timing
        self.raw_sensor_update_count += 1
        
        # Debug output (only for right hand to avoid spam)
        if hand == 'r' and self.raw_sensor_update_count % 20 == 0 and self.debug_printing:
            print(f"📊 Raw sensor data - IMU: {list(self.current_imu_values.values())}, Accel Mag: {self.current_accel_magnitude:.1f}")
    
    def on_error(self, error_message):
        """Handle errors from sensor thread"""
        if self.debug_printing:
            print(f"❌ Sensor error: {error_message}")
        self.status_bar.showMessage(f"Error: {error_message}")
    
    def save_configuration(self):
        """Save current configuration to file"""
        config = {
            'trigger_sensor': self.trigger_sensor_combo.currentText(),
            'threshold': self.threshold_spinbox.value(),
            'hysteresis': self.hysteresis_spinbox.value(),
            'volume_enabled': self.volume_enabled.isChecked(),
            'volume_controller': self.volume_controller_combo.currentText(),
            'base_volume': self.base_volume_slider.value(),
            'volume_sensitivity': self.volume_sensitivity_slider.value(),
            'volume_smoothing': self.volume_smoothing_slider.value(),
            'pitch_enabled': self.pitch_enabled.isChecked(),
            'pitch_controller': self.pitch_controller_combo.currentText(),
            'pitch_range': self.pitch_range_slider.value(),
            'pitch_mode': self.pitch_mode_combo.currentText(),
            'pitch_smoothing': self.pitch_smoothing_slider.value(),
            'window_control_enabled': self.window_control_enabled.isChecked(),
            'window_controller': self.window_controller_combo.currentText(),
            'debug_printing': self.debug_printing
        }
        
        filename, _ = QFileDialog.getSaveFileName(
            self, 'Save Volume/Pitch Configuration', 'volume_pitch_config.json', 'JSON files (*.json)'
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
            self, 'Load Volume/Pitch Configuration', '', 'JSON files (*.json)'
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
                'volume_controller': 'accel_mag',
                'base_volume': 20,
                'volume_sensitivity': 15000,
                'volume_smoothing': 10,
                'pitch_enabled': True,
                'pitch_controller': 'imu2',
                'pitch_range': 4096,
                'pitch_mode': 'modulo',
                'pitch_smoothing': 5,
                'window_control_enabled': False,
                'debug_printing': False
            }
        elif preset_name == 'advanced':
            config = {
                'volume_enabled': True,
                'volume_controller': 'accel_mag',
                'base_volume': 10,
                'volume_sensitivity': 20000,
                'volume_smoothing': 5,
                'pitch_enabled': True,
                'pitch_controller': 'imu1',
                'pitch_range': 8192,
                'pitch_mode': 'linear',
                'pitch_smoothing': 3,
                'window_control_enabled': True,
                'window_controller': 'imu3',
                'debug_printing': False
            }
        else:
            QMessageBox.warning(self, "Error", f"Unknown preset: {preset_name}")
            return
        
        self._apply_configuration(config)
        QMessageBox.information(self, "Preset Loaded", f"Applied {preset_name} preset configuration")
    
    def _apply_configuration(self, config):
        """Apply configuration to GUI controls"""
        if 'trigger_sensor' in config:
            self.trigger_sensor_combo.setCurrentText(config['trigger_sensor'])
        if 'threshold' in config:
            self.threshold_spinbox.setValue(config['threshold'])
        if 'hysteresis' in config:
            self.hysteresis_spinbox.setValue(config['hysteresis'])
        
        if 'volume_enabled' in config:
            self.volume_enabled.setChecked(config['volume_enabled'])
        if 'volume_controller' in config:
            self.volume_controller_combo.setCurrentText(config['volume_controller'])
        if 'base_volume' in config:
            self.base_volume_slider.setValue(config['base_volume'])
        if 'volume_sensitivity' in config:
            self.volume_sensitivity_slider.setValue(config['volume_sensitivity'])
        if 'volume_smoothing' in config:
            self.volume_smoothing_slider.setValue(config['volume_smoothing'])
        
        if 'pitch_enabled' in config:
            self.pitch_enabled.setChecked(config['pitch_enabled'])
        if 'pitch_controller' in config:
            self.pitch_controller_combo.setCurrentText(config['pitch_controller'])
        if 'pitch_range' in config:
            self.pitch_range_slider.setValue(config['pitch_range'])
        if 'pitch_mode' in config:
            self.pitch_mode_combo.setCurrentText(config['pitch_mode'])
        if 'pitch_smoothing' in config:
            self.pitch_smoothing_slider.setValue(config['pitch_smoothing'])
        
        if 'window_control_enabled' in config:
            self.window_control_enabled.setChecked(config['window_control_enabled'])
        if 'window_controller' in config:
            self.window_controller_combo.setCurrentText(config['window_controller'])
        
        if 'debug_printing' in config:
            self.debug_printing = config['debug_printing']
            self.debug_checkbox.setChecked(self.debug_printing)
    
    def toggle_debug_printing(self, state):
        """Toggle debug printing"""
        self.debug_printing = state == Qt.Checked
    
    def closeEvent(self, event):
        """Handle application close"""
        if self.running:
            self.stop_system()
        event.accept()


def main():
    """Main application entry point"""
    print("🚀 Starting GLUVN Volume/Pitch Control GUI")
    print("=" * 80)
    print("🎛️  Features:")
    print("   • Flexible sensor assignment (accelerometer magnitude or individual IMU)")
    print("   • Real-time volume and pitch bend control")
    print("   • Complete parameter customization")
    print("   • Real-time quantification displays")
    print("   • Save/load configurations")
    print("   • Preset system for quick setup")
    print("=" * 80)
    
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = VolumePitchControlGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 