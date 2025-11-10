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
import math

# PyQt5 imports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGroupBox, QSpinBox, QComboBox, 
                            QPushButton, QLabel, QStatusBar, QMessageBox, QCheckBox, QInputDialog, QSlider, QSizePolicy, QProgressBar)
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
    def __init__(self, max_points=500, plot_mode='imu_vs_imu', parent_gui=None):
        super().__init__()
        self.max_points = max_points
        self.plot_mode = plot_mode  # 'imu_vs_imu' or 'accel_vs_imu'
        self.parent_gui = parent_gui  # Reference to main GUI for accessing controls
        
        # Data buffers
        self.data_buffer = {
            'l': np.zeros((max_points, 3), dtype=np.float32),  # yaw, pitch, roll
            'r': np.zeros((max_points, 3), dtype=np.float32),
            'r_accel_mag': np.zeros((max_points,), dtype=np.float32),
            'volume': np.zeros((max_points,), dtype=np.float32)
        }
        self.buffer_index = {'l': 0, 'r': 0, 'r_accel_mag': 0, 'volume': 0}
        self.last_volume = 0
        
        # Layout
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Create plot widgets
        self.plot_widgets = {
            'left': pg.PlotWidget(title='Acceleration Magnitude (Right Hand)'),
            'right': pg.PlotWidget(title='Right Hand IMU')
        }
        
        # Setup plots
        for side in ['left', 'right']:
            plot = self.plot_widgets[side]
            plot.showGrid(x=True, y=True)
            plot.setLabel('left', 'Value')
            plot.setLabel('bottom', 'Sample')
            plot.setYRange(0, 65535)  # IMU range now 0-65535
            plot.addLegend(offset=(10, 10))
            layout.addWidget(plot)
        
        # Create curves with different colors
        self.curves = {
            'l': [self.plot_widgets['left'].plot(pen=pg.mkPen(color=(255,0,0), width=2), name='Yaw'),
                  self.plot_widgets['left'].plot(pen=pg.mkPen(color=(0,255,0), width=2), name='Pitch'),
                  self.plot_widgets['left'].plot(pen=pg.mkPen(color=(0,0,255), width=2), name='Roll')],
            'r': [self.plot_widgets['right'].plot(pen=pg.mkPen(color=(255,0,0), width=2), name='Yaw'),
                  self.plot_widgets['right'].plot(pen=pg.mkPen(color=(0,255,0), width=2), name='Pitch'),
                  self.plot_widgets['right'].plot(pen=pg.mkPen(color=(0,0,255), width=2), name='Roll')],
            'r_accel_mag': [self.plot_widgets['left'].plot(pen=pg.mkPen(color=(255,255,255), width=3), name='Accel Mag')],
            'volume': [self.plot_widgets['right'].plot(pen=pg.mkPen(color=(255,0,255), width=2), name='Volume (0-127)')]
        }
    
    def set_last_volume(self, midi_vol):
        # Store the latest volume for overlay plotting
        if self.buffer_index['volume'] >= self.max_points:
            self.data_buffer['volume'] = np.roll(self.data_buffer['volume'], -1)
            self.buffer_index['volume'] = self.max_points - 1
        self.data_buffer['volume'][self.buffer_index['volume']] = midi_vol
        self.buffer_index['volume'] += 1
    
    def set_plot_mode(self, plot_mode):
        """Update the plot mode"""
        self.plot_mode = plot_mode
    
    def update_plot(self, hand, imu_array, zero_accel=None, baseline_subtract=True):
        """Update the plot with new IMU data"""
        if not isinstance(imu_array, (list, tuple)) or len(imu_array) < 3:
            if self.parent_gui and hasattr(self.parent_gui, 'print_checkbox') and self.parent_gui.print_checkbox.isChecked():
                print(f"[PLOT] Invalid IMU array for {hand}: {imu_array}")
            return
        
        if self.parent_gui and hasattr(self.parent_gui, 'print_checkbox') and self.parent_gui.print_checkbox.isChecked():
            print(f"[PLOT] Updating plot for {hand}, mode: {self.plot_mode}, data length: {len(imu_array)}")
            
        # Update IMU data
        ypr_data = np.array(imu_array[:3], dtype=np.float32)
        if self.buffer_index[hand] > 0:
            ypr_data = 0.9 * self.data_buffer[hand][self.buffer_index[hand]-1] + 0.1 * ypr_data
        if self.buffer_index[hand] >= self.max_points:
            self.data_buffer[hand] = np.roll(self.data_buffer[hand], -1, axis=0)
            self.buffer_index[hand] = self.max_points - 1
        self.data_buffer[hand][self.buffer_index[hand]] = ypr_data
        self.buffer_index[hand] += 1
        
        if self.parent_gui and hasattr(self.parent_gui, 'print_checkbox') and self.parent_gui.print_checkbox.isChecked():
            print(f"[PLOT] Buffer index for {hand}: {self.buffer_index[hand]}, YPR: {ypr_data}")
        
        # If right hand, also update accel magnitude (centered, baseline-subtracted)
        if hand == 'r' and len(imu_array) >= 6 and zero_accel is not None:
            ax, ay, az = imu_array[3], imu_array[4], imu_array[5]
            ax_c = ax - 32767.5
            ay_c = ay - 32767.5
            az_c = az - 32767.5
            norm = (ax_c**2 + ay_c**2 + az_c**2) ** 0.5
            if baseline_subtract:
                accel_mag = max(norm - zero_accel, 0)
            else:
                accel_mag = norm
            # Print if enabled and parent GUI is available
            if self.parent_gui and hasattr(self.parent_gui, 'print_checkbox') and self.parent_gui.print_checkbox.isChecked():
                print(f"[PLOT] Centered norm: {norm}, After baseline: {accel_mag}")
            if self.buffer_index['r_accel_mag'] >= self.max_points:
                self.data_buffer['r_accel_mag'] = np.roll(self.data_buffer['r_accel_mag'], -1)
                self.buffer_index['r_accel_mag'] = self.max_points - 1
            self.data_buffer['r_accel_mag'][self.buffer_index['r_accel_mag']] = accel_mag
            self.buffer_index['r_accel_mag'] += 1
        
        # Plotting
        if self.plot_mode == 'imu_vs_imu':
            if self.parent_gui and hasattr(self.parent_gui, 'print_checkbox') and self.parent_gui.print_checkbox.isChecked():
                print(f"[PLOT] IMU vs IMU mode - plotting {hand} hand")
            # Left: left hand IMU
            if self.buffer_index['l'] > 0:
                valid_data = self.data_buffer['l'][:self.buffer_index['l']]
                x = np.arange(len(valid_data))
                for i in range(3):
                    self.curves['l'][i].setData(x, valid_data[:, i])
            # Right: right hand IMU
            if self.buffer_index['r'] > 0:
                valid_data = self.data_buffer['r'][:self.buffer_index['r']]
                x = np.arange(len(valid_data))
                for i in range(3):
                    self.curves['r'][i].setData(x, valid_data[:, i])
        elif self.plot_mode == 'imu_vs_volume':
            if self.parent_gui and hasattr(self.parent_gui, 'print_checkbox') and self.parent_gui.print_checkbox.isChecked():
                print(f"[PLOT] IMU vs Volume mode - plotting acceleration and volume")
            # Left: right hand acceleration magnitude
            if self.buffer_index['r_accel_mag'] > 0:
                valid_accel = self.data_buffer['r_accel_mag'][:self.buffer_index['r_accel_mag']]
                x = np.arange(len(valid_accel))
                self.curves['r_accel_mag'][0].setData(x, valid_accel)
                # Auto-scale y-axis for accel on left plot
                if len(valid_accel) > 0:
                    min_y = np.min(valid_accel)
                    max_y = np.max(valid_accel)
                    if min_y != max_y:
                        self.plot_widgets['left'].setYRange(min_y, max_y)
            # Right: IMU data + volume overlay
            if self.buffer_index['r'] > 0:
                valid_data = self.data_buffer['r'][:self.buffer_index['r']]
                x = np.arange(len(valid_data))
                for i in range(3):
                    self.curves['r'][i].setData(x, valid_data[:, i])
                # Set IMU range for right plot
                self.plot_widgets['right'].setYRange(0, 65535)
            # Volume overlay on right plot (scaled to IMU range)
            if self.buffer_index['volume'] > 0:
                valid_vol = self.data_buffer['volume'][:self.buffer_index['volume']]
                x = np.arange(len(valid_vol))
                # Scale volume (0-127) to IMU range (0-65535) for visibility
                vol_scaled = valid_vol * (65535 / 127.0)
                self.curves['volume'][0].setData(x, vol_scaled)
                if self.parent_gui and hasattr(self.parent_gui, 'print_checkbox') and self.parent_gui.print_checkbox.isChecked():
                    print(f"[PLOT] Volume data points: {len(valid_vol)}, last value: {valid_vol[-1] if len(valid_vol) > 0 else 'N/A'}, scaled: {vol_scaled[-1] if len(vol_scaled) > 0 else 'N/A'}")

class TenFingerPitchBendGUI(QMainWindow):
    """Main GUI application with real-time sensor value display and pitch bend using IMU2"""
    BASELINE_SUBTRACT = True  # Set to False to disable baseline subtraction
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
        # Setup UI-related state
        self.pitch_bend_imu_channel = 2  # Use Roll by default (IMU2 (Roll) in combo)
        self.volume_control_hand = 'r'
        self.last_volume_update = 0
        self.midi_update_interval = 0.01  # faster PB updates
        self.base_volume = 10
        self.avg_window = 10
        self.max_avg_window = 100
        self.accel_mag_buffers = {'l': [0]*self.max_avg_window, 'r': [0]*self.max_avg_window}
        self.buffer_index = {'l': 0, 'r': 0}
        self.TWO_BYTE = 65535
        self.ZERO_ACCEL = self.TWO_BYTE / 4.0 - 680.0
        self.sensitivity = 1.0
        # Pitch bend visual/smoothing
        self.pb_window = 2  # snappier default
        self.pb_vals = deque(maxlen=100)
        self.pb_sensitivity = 1.0  # scale roll response (less sensitive by default)
        self.pb_flip = True        # flip so palm-down is neutral by default
        self.pb_span_deg = 45.0    # degrees that map to full-scale bend before sensitivity
        self.plot_mode = 'imu_vs_imu'
        # Build UI
        self._setup_ui()
        self.status_bar = QStatusBar()
        self.status_bar.showMessage("Ready - Click 'Start Sensors' to begin")
        self.setStatusBar(self.status_bar)
        # Optional plot timer removed; plotting is driven by sensor updates
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
        # Restore the previous working vertical layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        title = QLabel("GLUVN Ten Finger GUI with Pitch Bend (IMU2)")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Bold))
        main_layout.addWidget(title)
        # Controls panel (horizontal row)
        controls_row = QHBoxLayout()
        # Start Sensors button
        self.start_button = QPushButton("Start Sensors")
        self.start_button.clicked.connect(self.start_sensors)
        controls_row.addWidget(self.start_button)
        # Pitch bend source selector
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
        self.pitch_bend_combo.setCurrentIndex(1)
        self.pitch_bend_combo.currentIndexChanged.connect(self._on_pitch_bend_channel_changed)
        pitch_bend_layout.addWidget(pitch_bend_label)
        pitch_bend_layout.addWidget(self.pitch_bend_combo)
        controls_row.addWidget(pitch_bend_group)
        # Volume control hand selector
        volume_group = QGroupBox('Volume Control')
        volume_layout = QHBoxLayout(volume_group)
        volume_label = QLabel('Volume Control Hand:')
        self.volume_combo = QComboBox()
        self.volume_combo.addItems(['Left', 'Right'])
        self.volume_combo.setCurrentIndex(1)
        self.volume_combo.currentIndexChanged.connect(self._on_volume_hand_changed)
        volume_layout.addWidget(volume_label)
        volume_layout.addWidget(self.volume_combo)
        controls_row.addWidget(volume_group)
        # Sensitivity slider
        sens_group = QGroupBox('Accelerometer Sensitivity')
        sens_layout = QHBoxLayout(sens_group)
        sens_label = QLabel('Sensitivity:')
        self.sens_slider = QSlider(Qt.Horizontal)
        self.sens_slider.setMinimum(1)
        self.sens_slider.setMaximum(200)
        self.sens_slider.setValue(int(self.sensitivity * 100))
        self.sens_slider.valueChanged.connect(self._on_sensitivity_changed)
        self.sens_value_label = QLabel(f"{self.sensitivity:.2f}")
        sens_layout.addWidget(sens_label)
        sens_layout.addWidget(self.sens_slider)
        sens_layout.addWidget(self.sens_value_label)
        controls_row.addWidget(sens_group)
        # Moving average window slider
        avg_group = QGroupBox('Moving Average Window')
        avg_layout = QHBoxLayout(avg_group)
        avg_label = QLabel('Window Size:')
        self.avg_slider = QSlider(Qt.Horizontal)
        self.avg_slider.setMinimum(1)
        self.avg_slider.setMaximum(self.max_avg_window)
        self.avg_slider.setValue(self.avg_window)
        self.avg_slider.valueChanged.connect(self._on_avg_window_changed)
        self.avg_value_label = QLabel(f"{self.avg_window}")
        avg_layout.addWidget(avg_label)
        avg_layout.addWidget(self.avg_slider)
        avg_layout.addWidget(self.avg_value_label)
        controls_row.addWidget(avg_group)
        # Plot mode selector
        plot_group = QGroupBox('Plot Mode')
        plot_layout = QHBoxLayout(plot_group)
        plot_label = QLabel('Plot Mode:')
        self.plot_combo = QComboBox()
        self.plot_combo.addItems(['IMU vs IMU', 'IMU vs Volume'])
        self.plot_combo.setCurrentIndex(0)
        self.plot_combo.currentIndexChanged.connect(self._on_plot_mode_changed)
        plot_layout.addWidget(plot_label)
        plot_layout.addWidget(self.plot_combo)
        controls_row.addWidget(plot_group)
        # Print toggle
        print_group = QGroupBox('Printing')
        print_layout = QHBoxLayout(print_group)
        self.print_checkbox = QCheckBox('Print Values')
        self.print_checkbox.setChecked(False)
        print_layout.addWidget(self.print_checkbox)
        controls_row.addWidget(print_group)
        main_layout.addLayout(controls_row)
        # Add finger/flex sensor visualizer
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
        # Pitch bend indicator (below finger display)
        pb_group = QGroupBox("Pitch Bend")
        pb_layout = QHBoxLayout(pb_group)
        self.pb_label = QLabel("0 st")
        self.pb_bar = QProgressBar()
        self.pb_bar.setRange(-8192, 8191)
        self.pb_bar.setValue(0)
        self.pb_bar.setTextVisible(False)
        # PB controls (sensitivity + flip)
        self.pb_sens_slider = QSlider(Qt.Horizontal)
        self.pb_sens_slider.setMinimum(50)   # 0.5x
        self.pb_sens_slider.setMaximum(300)  # 3.0x
        self.pb_sens_slider.setValue(int(self.pb_sensitivity*100))
        self.pb_sens_slider.setFixedWidth(120)
        self.pb_sens_slider.valueChanged.connect(self._on_pb_sens_changed)
        self.pb_flip_checkbox = QCheckBox("Flip")
        self.pb_flip_checkbox.setChecked(self.pb_flip)
        self.pb_flip_checkbox.stateChanged.connect(self._on_pb_flip_changed)
        pb_layout.addWidget(QLabel("Bend:"))
        pb_layout.addWidget(self.pb_bar, 1)
        pb_layout.addWidget(self.pb_label)
        pb_layout.addWidget(QLabel("Sens:"))
        pb_layout.addWidget(self.pb_sens_slider)
        pb_layout.addWidget(self.pb_flip_checkbox)
        main_layout.addWidget(pb_group)
        # Add IMU plot
        self.imu_plot = IMUPlotWidget(max_points=500, plot_mode=self.plot_mode, parent_gui=self)
        main_layout.addWidget(self.imu_plot)
    def _get_status_info(self):
        if self.hardware_available:
            if self.midi_available:
                return "✅ Hardware + MIDI Connected - Full functionality available", "lightgreen"
            else:
                return "⚠️ Hardware Connected, MIDI Unavailable - Sensor display only", "lightyellow"
        else:
            return "❌ Hardware Not Available - Check connection and restart", "lightcoral"
    def start_sensors(self):
        if self.running:
            return
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
        # self.stop_button.setEnabled(False)
        self.status_bar.showMessage("⏹️ Sensors stopped")
    def on_sensor_update(self, hand, finger_idx, sensor_value, triggered, switch_event):
        self.finger_display.update_sensor_value(hand, finger_idx, sensor_value)
        self.finger_display.set_triggered(hand, finger_idx, triggered)
        if self.midi_available and self.midi_writer and switch_event != 0:
            if hand in self.note_maps and finger_idx < len(self.note_maps[hand]):
                note = self.note_maps[hand][finger_idx]
                if switch_event == 1:
                    # Use lower velocity for smoother attack
                    self.midi_writer.trig_note(note, vel=60)
                    if self.debug_printing:
                        print(f"🎵 MIDI ON: {hand.upper()} finger {finger_idx}, note {note}")
                elif switch_event == -1:
                    # Use velocity 64 instead of 0 for smoother release
                    # self.midi_writer.trig_note(note, vel=64)
                    # Small delay before sending note off
                    time.sleep(0.001)  # 1ms delay
                    self.midi_writer.trig_note(note, vel=0)
                    if self.debug_printing:
                        print(f"🎵 MIDI OFF: {hand.upper()} finger {finger_idx}, note {note}")
    def on_raw_sensor_update(self, hand, raw_sensor_dict):
        # Update IMU plot
        if 'imu' in raw_sensor_dict:
            self.imu_plot.update_plot(hand, raw_sensor_dict['imu'], zero_accel=self.ZERO_ACCEL, baseline_subtract=self.BASELINE_SUBTRACT)
        
        # Pitch bend: use selected IMU channel (expect roll at index 2)
        if hand == 'r' and 'imu' in raw_sensor_dict and self.midi_available and self.midi_writer:
            imu = raw_sensor_dict['imu']
            if len(imu) > self.pitch_bend_imu_channel:
                raw_val = float(imu[self.pitch_bend_imu_channel])  # 0..65535
                # Map IMU roll channel raw (0..65535) -> angle 0..360
                angle_deg = (raw_val / 65535.0) * 360.0
                # Circular shortest signed difference to neutral 0° in [-180,180)
                delta_deg = ((angle_deg + 180.0) % 360.0) - 180.0
                # Optional flip only inverts direction, does NOT change neutral
                if self.pb_flip:
                    delta_deg = -delta_deg
                # Normalize to [-1,1] using span degrees, then smooth
                norm = max(min(delta_deg / max(self.pb_span_deg, 1e-3), 1.0), -1.0)
                self.pb_vals.append(norm)
                win = min(self.pb_window, len(self.pb_vals))
                smoothed = sum(list(self.pb_vals)[-win:]) / max(win, 1)
                # Sensitivity scaling
                scaled = max(min(smoothed * self.pb_sensitivity, 1.0), -1.0)
                # Map to MIDI PB units (assumes synth PB range set to +/- 2 semitones ~ full note)
                pb_units = int(scaled * 8191)
                # Throttle and send
                now = time.time()
                if now - getattr(self, 'last_pitch_update', 0) >= self.midi_update_interval:
                    self.midi_writer.pitch_bend(pb_units)
                    self.last_pitch_update = now
                # Update indicator (also show approx semitone bend with +/-2 range)
                self.pb_bar.setValue(pb_units)
                approx_semitones = scaled * 2.0  # +/- 2 semitones
                self.pb_label.setText(f"{approx_semitones:+.2f} st")
                if self.print_checkbox.isChecked():
                    print(f"[PB] raw:{raw_val:.0f} ang:{angle_deg:6.2f} d:{delta_deg:+6.2f} norm:{norm:+.3f} sm:{smoothed:+.3f} sc:{scaled:+.3f} pb:{pb_units:+d} st:{approx_semitones:+.2f}")
        # Volume control (selected hand, moving average of accel magnitude, window size adjustable)
        if hand == self.volume_control_hand and 'imu' in raw_sensor_dict:
            imu = raw_sensor_dict['imu']
            if len(imu) >= 6:
                ax, ay, az = imu[3], imu[4], imu[5]
                # Center the acceleration values
                ax_c = ax - 32767.5
                ay_c = ay - 32767.5
                az_c = az - 32767.5
                norm = (ax_c**2 + ay_c**2 + az_c**2) ** 0.5
                if self.BASELINE_SUBTRACT:
                    accel_val = norm - self.ZERO_ACCEL  # allow negatives
                else:
                    accel_val = norm
                if self.print_checkbox.isChecked():
                    print(f"[DEBUG] Centered norm: {norm}, After baseline: {accel_val}")
                # Update buffer for moving average
                buf = self.accel_mag_buffers[hand]
                buf.append(accel_val)
                if len(buf) > self.avg_window:
                    buf.pop(0)
                avg_accel = np.mean(buf[-self.avg_window:])
                # --- Smooth nonlinear mapping (tanh) ---
                # Map avg_accel to [-1, 1] for tanh, then to [0, 127]
                normed = math.tanh(self.sensitivity * avg_accel / 10000.0)
                midi_vol = int(np.clip(self.base_volume + ((normed + 1) / 2) * (127 - self.base_volume), 0, 127))
                now = time.time()
                if now - self.last_volume_update >= self.midi_update_interval:
                    self.midi_writer.aftertouch(midi_vol)
                    self.last_volume_update = now
                # Pass midi_vol to plot for overlay
                self.imu_plot.set_last_volume(midi_vol)
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
    def _on_volume_hand_changed(self, idx):
        self.volume_control_hand = 'l' if idx == 0 else 'r'
    def _on_sensitivity_changed(self, value):
        self.sensitivity = value / 100.0
        self.sens_value_label.setText(f"{self.sensitivity:.2f}")
    def _on_avg_window_changed(self, value):
        self.avg_window = value
        self.avg_value_label.setText(f"{self.avg_window}")
        # Resize buffer for both hands
        for hand in ['l', 'r']:
            old_buf = self.accel_mag_buffers[hand]
            if len(old_buf) < self.avg_window:
                self.accel_mag_buffers[hand] = old_buf + [0]*(self.avg_window - len(old_buf))
            elif len(old_buf) > self.avg_window:
                self.accel_mag_buffers[hand] = old_buf[-self.avg_window:]
    def _on_plot_mode_changed(self, index):
        self.plot_mode = 'imu_vs_imu' if index == 0 else 'imu_vs_volume'
        self.imu_plot.set_plot_mode(self.plot_mode)
    def _on_pb_sens_changed(self, val):
        self.pb_sensitivity = max(0.1, val/100.0)
    def _on_pb_flip_changed(self, state):
        self.pb_flip = (state == Qt.Checked)
    def _update_plot(self):
        # Remove this method as it's causing errors and not needed
        # The plot updates are handled by on_raw_sensor_update
        pass
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