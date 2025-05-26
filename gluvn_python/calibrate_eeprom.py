#!/usr/bin/env python3
"""
GLUVN Real-Time Calibration Tool with Live Plotting
Fixed: IMU data handling, individual pressure calibration, hand selection
"""

import sys
import time
import serial
import numpy as np
import os
import shutil
from __init__ import portR, portL, baud, mainDir
from port_read import Reader

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QPushButton, QLabel, QComboBox, 
                            QGroupBox, QTextEdit, QSplitter)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

import pyqtgraph as pg

class CalibrationToolWithRealTimePlots(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GLUVN Sensor Calibration Tool with Live Plotting")
        self.setGeometry(100, 100, 1400, 800)
        
        # Hand configuration - start with right hand
        self.selected_hand = 'r'
        self.port = portR
        
        # Calibration state
        self.calibration_step = 'idle'
        self.recorded_data = []
        self.start_time = None
        self.recording_duration = 3.0  # seconds
        self.current_finger = 0  # For individual pressure calibration
        
        # Calibration values storage
        self.calibration_data = {
            'flex_min': None,
            'flex_max': None,
            'press_min': None,
            'press_max': [None] * 5  # Individual finger pressure max values
        }
        
        # Current sensor data
        self.current_data = {'flex': [0]*5, 'press': [0]*5}
        
        # Data buffer for plotting
        self.max_points = 500
        self.data_buffer = {
            'flex': np.zeros((self.max_points, 5), dtype=np.float32),
            'press': np.zeros((self.max_points, 5), dtype=np.float32)
        }
        self.buffer_index = 0
        
        # Reader instance
        self.reader = None
        self.current_mode = 'CALIBRATION'  # Start in calibration mode for raw data
        
        self.setup_ui()
        
        # Set up timer for reading data
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_sensor_data)
        self.timer.start(50)  # Update every 50ms
        
        # Start in calibration mode for raw data
        self.set_arduino_mode('CALIBRATION')
        
        print(f"✅ Calibration tool started for {self.selected_hand.upper()} hand")
        
    def setup_ui(self):
        """Setup the user interface with plots"""
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # Create horizontal splitter for controls and plots
        main_splitter = QSplitter(Qt.Horizontal)
        main_layout = QHBoxLayout(main_widget)
        main_layout.addWidget(main_splitter)
        
        # Left side: Controls
        self.setup_controls(main_splitter)
        
        # Right side: Plots
        self.setup_plots(main_splitter)
        
        # Set splitter sizes (controls: 400px, plots: rest)
        main_splitter.setSizes([400, 1000])
        
    def setup_controls(self, parent):
        """Setup the control panel"""
        controls_widget = QWidget()
        parent.addWidget(controls_widget)
        controls_layout = QVBoxLayout(controls_widget)
        
        # Title
        title = QLabel("GLUVN Sensor Calibration")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        controls_layout.addWidget(title)
        
        # Hand selection
        hand_group = QGroupBox("Hand Selection")
        hand_layout = QVBoxLayout(hand_group)
        
        hand_selection_layout = QHBoxLayout()
        self.hand_combo = QComboBox()
        self.hand_combo.addItems(['Right Hand', 'Left Hand'])
        self.hand_combo.setCurrentIndex(0)  # Default to right hand
        self.hand_combo.currentTextChanged.connect(self.change_hand)
        
        hand_selection_layout.addWidget(QLabel("Hand:"))
        hand_selection_layout.addWidget(self.hand_combo)
        hand_layout.addLayout(hand_selection_layout)
        
        self.hand_label = QLabel(f"Calibrating: {self.selected_hand.upper()} Hand")
        self.hand_label.setFont(QFont("Arial", 12, QFont.Bold))
        hand_layout.addWidget(self.hand_label)
        controls_layout.addWidget(hand_group)
        
        # Mode control
        mode_group = QGroupBox("Arduino Mode")
        mode_layout = QVBoxLayout(mode_group)
        
        self.mode_toggle_btn = QPushButton("Switch to NORMAL Mode\n(View Calibrated Data)")
        self.mode_toggle_btn.clicked.connect(self.toggle_mode)
        self.mode_toggle_btn.setStyleSheet("QPushButton { background-color: #ff6b6b; color: white; font-weight: bold; }")
        
        self.mode_label = QLabel("Mode: CALIBRATION (Raw Data)")
        self.mode_label.setFont(QFont("Arial", 10, QFont.Bold))
        
        mode_layout.addWidget(self.mode_toggle_btn)
        mode_layout.addWidget(self.mode_label)
        controls_layout.addWidget(mode_group)
        
        # Current sensor values
        values_group = QGroupBox("Current Sensor Values")
        values_layout = QVBoxLayout(values_group)
        
        self.flex_label = QLabel("Flex:  [0, 0, 0, 0, 0]")
        self.press_label = QLabel("Press: [0, 0, 0, 0, 0]")
        
        values_layout.addWidget(self.flex_label)
        values_layout.addWidget(self.press_label)
        controls_layout.addWidget(values_group)
        
        # Calibration steps
        steps_group = QGroupBox("Calibration Steps (Use RAW mode)")
        steps_layout = QVBoxLayout(steps_group)
        
        self.step1_btn = QPushButton("Step 1: Record Flex MIN\n(fingers straight)")
        self.step1_btn.clicked.connect(lambda: self.start_recording('flex_min'))
        
        self.step2_btn = QPushButton("Step 2: Record Flex MAX\n(fingers bent)")
        self.step2_btn.clicked.connect(lambda: self.start_recording('flex_max'))
        
        self.step3_btn = QPushButton("Step 3: Record Pressure MIN\n(no pressure)")
        self.step3_btn.clicked.connect(lambda: self.start_recording('press_min'))
        
        # Individual pressure buttons
        self.pressure_buttons = []
        finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        for i in range(5):
            btn = QPushButton(f"Step {4+i}: Press {finger_names[i]}\n(finger {i+1} max pressure)")
            btn.clicked.connect(lambda checked, finger=i: self.start_recording('press_max', finger))
            self.pressure_buttons.append(btn)
            steps_layout.addWidget(btn)
        
        steps_layout.addWidget(self.step1_btn)
        steps_layout.addWidget(self.step2_btn)
        steps_layout.addWidget(self.step3_btn)
        
        for btn in self.pressure_buttons:
            steps_layout.addWidget(btn)
            
        controls_layout.addWidget(steps_group)
        
        # Status
        self.status_label = QLabel("📊 Ready to calibrate!\nUse CALIBRATION mode for recording.")
        self.status_label.setFont(QFont("Arial", 10, QFont.Bold))
        controls_layout.addWidget(self.status_label)
        
        # Save button
        self.save_btn = QPushButton("Save Calibration to EEPROM")
        self.save_btn.clicked.connect(self.save_calibration)
        self.save_btn.setEnabled(False)
        controls_layout.addWidget(self.save_btn)
        
        # Reset to defaults button
        self.reset_btn = QPushButton("Reset to calibration.h Defaults")
        self.reset_btn.clicked.connect(self.reset_to_defaults)
        self.reset_btn.setStyleSheet("QPushButton { background-color: #ffa500; color: white; font-weight: bold; }")
        controls_layout.addWidget(self.reset_btn)
        
        controls_layout.addStretch()
        
    def setup_plots(self, parent):
        """Setup the plotting area"""
        plots_widget = QWidget()
        parent.addWidget(plots_widget)
        plots_layout = QVBoxLayout(plots_widget)
        
        # Flex sensors plot
        self.flex_plot = pg.PlotWidget(title='Flex Sensors (Real-time)')
        self.flex_plot.showGrid(x=True, y=True)
        self.flex_plot.setLabel('left', 'Sensor Value')
        self.flex_plot.setLabel('bottom', 'Sample')
        self.flex_plot.setYRange(0, 1023)  # Start with raw data range
        self.flex_plot.addLegend(offset=(10, 10))
        
        # Create flex sensor curves
        flex_colors = ['red', 'green', 'blue', 'yellow', 'cyan']
        self.flex_curves = []
        for i in range(5):
            curve = self.flex_plot.plot(pen=pg.mkPen(color=flex_colors[i], width=2), 
                                       name=f'Flex {i+1}')
            self.flex_curves.append(curve)
        
        plots_layout.addWidget(self.flex_plot)
        
        # Pressure sensors plot
        self.press_plot = pg.PlotWidget(title='Pressure Sensors (Real-time)')
        self.press_plot.showGrid(x=True, y=True)
        self.press_plot.setLabel('left', 'Sensor Value')
        self.press_plot.setLabel('bottom', 'Sample')
        self.press_plot.setYRange(0, 1023)  # Start with raw data range
        self.press_plot.addLegend(offset=(10, 10))
        
        # Create pressure sensor curves
        press_colors = ['red', 'green', 'blue', 'yellow', 'cyan']
        self.press_curves = []
        for i in range(5):
            curve = self.press_plot.plot(pen=pg.mkPen(color=press_colors[i], width=2), 
                                        name=f'Press {i+1}')
            self.press_curves.append(curve)
        
        plots_layout.addWidget(self.press_plot)
        
    def change_hand(self, hand_text):
        """Change the hand being calibrated"""
        if hand_text == 'Right Hand':
            self.selected_hand = 'r'
            self.port = portR
        else:
            self.selected_hand = 'l'
            self.port = portL
            
        self.hand_label.setText(f"Calibrating: {self.selected_hand.upper()} Hand")
        
        # Reset calibration data for new hand
        self.calibration_data = {
            'flex_min': None,
            'flex_max': None,
            'press_min': None,
            'press_max': [None] * 5
        }
        self.save_btn.setEnabled(False)
        
        # Restart reader with new hand
        self.set_arduino_mode(self.current_mode)
        
        print(f"✅ Switched to {self.selected_hand.upper()} hand")
        
    def toggle_mode(self):
        """Toggle between CALIBRATION and NORMAL modes"""
        if self.current_mode == 'CALIBRATION':
            self.set_arduino_mode('NORMAL')
        else:
            self.set_arduino_mode('CALIBRATION')
            
    def set_arduino_mode(self, mode):
        """Set Arduino to specific mode and restart reader with correct format"""
        print(f"\n🔧 Setting Arduino to {mode}...")
        
        # Stop current reader if running
        if self.reader:
            self.reader.stop_readers()
            time.sleep(0.5)
        
        # Send command to Arduino
        ser = serial.Serial(self.port, baud, timeout=1)
        time.sleep(2)
        
        ser.write(f'HAND_{self.selected_hand.upper()}\n'.encode())
        time.sleep(0.3)
        
        if mode == 'CALIBRATION':
            ser.write(b'CAL_START\n')
        else:
            ser.write(b'CAL_STOP\n')
        time.sleep(0.5)
        
        # Check status
        ser.write(b'STATUS\n')
        time.sleep(0.5)
        while ser.in_waiting > 0:
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            if 'HAND:' in response:
                print(f"📋 Arduino Status: {response}")
        
        ser.reset_input_buffer()
        ser.close()
        
        # Start reader with appropriate format
        sensor_config = {self.selected_hand: {'flex': True, 'press': True, 'imu': True}}  # Include IMU
        
        if mode == 'CALIBRATION':
            # Raw data format (16-bit values) - IMU(6) + Flex(10) + Press(10) + hand(1) + newline(1) = 28 bytes
            # But Arduino sends: hand(1) + IMU(6*2=12) + Flex(5*2=10) + Press(5*2=10) + newline(1) = 34 bytes
            self.reader = Reader(sensor_config=sensor_config, save=False,
                               message_format='>sHHHHHHHHHHHHHHHH', length_checksum=33)
            self.flex_plot.setYRange(0, 1023)
            self.press_plot.setYRange(0, 1023)
            self.flex_plot.setTitle('Flex Sensors (RAW Data 0-1023)')
            self.press_plot.setTitle('Pressure Sensors (RAW Data 0-1023)')
            
            # Update UI
            self.mode_toggle_btn.setText("Switch to NORMAL Mode\n(View Calibrated Data)")
            self.mode_toggle_btn.setStyleSheet("QPushButton { background-color: #ff6b6b; color: white; font-weight: bold; }")
            self.mode_label.setText("Mode: CALIBRATION (Raw Data)")
            
            # Enable calibration buttons
            self.enable_calibration_buttons(True)
                
        else:
            # Calibrated data format (8-bit values) - IMU(6*2=12) + Flex(5) + Press(5) + hand(1) + newline(1) = 24 bytes
            self.reader = Reader(sensor_config=sensor_config, save=False,
                               message_format='>sHHHHHHBBBBBBBBBB', length_checksum=23)
            self.flex_plot.setYRange(0, 255)
            self.press_plot.setYRange(0, 255)
            self.flex_plot.setTitle('Flex Sensors (CALIBRATED Data 0-255)')
            self.press_plot.setTitle('Pressure Sensors (CALIBRATED Data 0-255)')
            
            # Update UI
            self.mode_toggle_btn.setText("Switch to CALIBRATION Mode\n(Record Raw Data)")
            self.mode_toggle_btn.setStyleSheet("QPushButton { background-color: #4ecdc4; color: white; font-weight: bold; }")
            self.mode_label.setText("Mode: NORMAL (Calibrated Data)")
            
            # Disable calibration buttons in normal mode
            self.enable_calibration_buttons(False)
        
        self.reader.start_readers()
        self.current_mode = mode
        time.sleep(1)
        
    def enable_calibration_buttons(self, enabled):
        """Enable or disable calibration buttons"""
        self.step1_btn.setEnabled(enabled)
        self.step2_btn.setEnabled(enabled)
        self.step3_btn.setEnabled(enabled)
        for btn in self.pressure_buttons:
            btn.setEnabled(enabled)
        
    def update_sensor_data(self):
        """Read sensor data and update plots"""
        if not self.reader:
            return
            
        try:
            data_updated = False
            
            while True:
                try:
                    data = self.reader.threads[self.selected_hand]['parser'].getQ().get(block=False)
                    
                    # Update current sensor values
                    if data and 'flex' in data:
                        self.current_data['flex'] = list(data['flex'])
                        data_updated = True
                    if data and 'press' in data:
                        self.current_data['press'] = list(data['press'])
                        data_updated = True
                        
                    # Store for calibration if recording (only in calibration mode)
                    if (self.calibration_step != 'idle' and 
                        self.start_time is not None and 
                        self.current_mode == 'CALIBRATION'):
                        self.recorded_data.append({
                            'flex': self.current_data['flex'].copy(),
                            'press': self.current_data['press'].copy()
                        })
                        
                except Exception:
                    break
            
            # Update display and plots if we got new data
            if data_updated:
                self.update_display()
                self.update_plots()
            
            # Check if recording is complete
            if self.calibration_step != 'idle' and self.start_time is not None:
                elapsed = time.time() - self.start_time
                remaining = self.recording_duration - elapsed
                
                if remaining > 0:
                    if self.calibration_step == 'press_max':
                        finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
                        self.status_label.setText(f"🔴 Recording {finger_names[self.current_finger]} pressure...\n{remaining:.1f}s remaining")
                    else:
                        self.status_label.setText(f"🔴 Recording {self.calibration_step}...\n{remaining:.1f}s remaining")
                else:
                    self.finish_recording()
                    
        except Exception as e:
            print(f"Error reading sensor data: {e}")
            
    def update_display(self):
        """Update the text display of current sensor values"""
        flex_str = ', '.join([f"{val:3d}" for val in self.current_data['flex']])
        press_str = ', '.join([f"{val:3d}" for val in self.current_data['press']])
        
        self.flex_label.setText(f"Flex:  [{flex_str}]")
        self.press_label.setText(f"Press: [{press_str}]")
        
    def update_plots(self):
        """Update the real-time plots"""
        # Add current data to buffer
        self.data_buffer['flex'][self.buffer_index] = self.current_data['flex']
        self.data_buffer['press'][self.buffer_index] = self.current_data['press']
        self.buffer_index = (self.buffer_index + 1) % self.max_points
        
        # Create x-axis (sample numbers)
        x_data = np.arange(self.max_points)
        
        # Update flex curves
        for i in range(5):
            # Roll the data so the newest point is at the right
            rolled_data = np.roll(self.data_buffer['flex'][:, i], -self.buffer_index)
            self.flex_curves[i].setData(x_data, rolled_data)
        
        # Update pressure curves
        for i in range(5):
            # Roll the data so the newest point is at the right
            rolled_data = np.roll(self.data_buffer['press'][:, i], -self.buffer_index)
            self.press_curves[i].setData(x_data, rolled_data)
            
    def start_recording(self, step, finger=None):
        """Start recording data for a calibration step"""
        # Check if we're in calibration mode
        if self.current_mode != 'CALIBRATION':
            self.status_label.setText("⚠️ Switch to CALIBRATION mode first!")
            return
        
        self.calibration_step = step
        self.recorded_data = []
        self.start_time = time.time()
        
        if step == 'press_max':
            self.current_finger = finger
        
        step_names = {
            'flex_min': 'Flex MIN (fingers straight)',
            'flex_max': 'Flex MAX (fingers bent)', 
            'press_min': 'Pressure MIN (no pressure)',
        }
        
        if step == 'press_max' and finger is not None:
            step_names['press_max'] = f'Pressure MAX (finger {finger+1})'
        
        self.status_label.setText(f"🔴 Recording {step_names[step]}...\n{self.recording_duration:.1f}s remaining")
        
        # Disable all buttons during recording
        self.enable_calibration_buttons(False)
        self.mode_toggle_btn.setEnabled(False)
        self.hand_combo.setEnabled(False)
            
    def finish_recording(self):
        """Finish recording and calculate average values"""
        if not self.recorded_data:
            self.status_label.setText("❌ No data recorded!")
            self.enable_buttons()
            return
            
        # Calculate averages
        flex_avg = np.mean([d['flex'] for d in self.recorded_data], axis=0)
        press_avg = np.mean([d['press'] for d in self.recorded_data], axis=0)
        
        # Store calibration values
        if self.calibration_step == 'flex_min':
            self.calibration_data['flex_min'] = flex_avg.tolist()
            values_str = ', '.join([f"{int(x)}" for x in flex_avg])
        elif self.calibration_step == 'flex_max':
            self.calibration_data['flex_max'] = flex_avg.tolist()
            values_str = ', '.join([f"{int(x)}" for x in flex_avg])
        elif self.calibration_step == 'press_min':
            self.calibration_data['press_min'] = press_avg.tolist()
            values_str = ', '.join([f"{int(x)}" for x in press_avg])
        elif self.calibration_step == 'press_max':
            # Store individual finger pressure (use minimum value as pressure decreases when pressed)
            self.calibration_data['press_max'][self.current_finger] = int(np.min([d['press'][self.current_finger] for d in self.recorded_data]))
            values_str = f"Finger {self.current_finger+1}: {self.calibration_data['press_max'][self.current_finger]}"
            
        self.status_label.setText(f"✅ {self.calibration_step} recorded!\n{len(self.recorded_data)} samples\nValues: {values_str}")
        
        # Reset recording state
        self.calibration_step = 'idle'
        self.start_time = None
        self.recorded_data = []
        
        self.enable_buttons()
        self.check_calibration_complete()
        
    def enable_buttons(self):
        """Re-enable all calibration buttons"""
        self.enable_calibration_buttons(True)
        self.mode_toggle_btn.setEnabled(True)
        self.hand_combo.setEnabled(True)
            
    def check_calibration_complete(self):
        """Check if all calibration steps are complete"""
        complete = (self.calibration_data['flex_min'] is not None and
                   self.calibration_data['flex_max'] is not None and
                   self.calibration_data['press_min'] is not None and
                   all(x is not None for x in self.calibration_data['press_max']))
        
        self.save_btn.setEnabled(complete)
        
        if complete:
            self.status_label.setText("🎉 All calibration steps complete!\nClick 'Save Calibration' to finish.\nThen switch to NORMAL mode to test!")
            
    def save_calibration(self):
        """Save calibration data to Arduino EEPROM and file"""
        if not (self.calibration_data['flex_min'] is not None and
                self.calibration_data['flex_max'] is not None and
                self.calibration_data['press_min'] is not None and
                all(x is not None for x in self.calibration_data['press_max'])):
            self.status_label.setText("❌ Calibration incomplete!")
            return
        
        try:
            # Send calibration data to Arduino EEPROM
            self.status_label.setText("📤 Sending calibration to Arduino EEPROM...")
            QApplication.processEvents()
            
            # Format calibration data for Arduino
            hand_prefix = self.selected_hand.upper()
            
            # Send MIN_FLEX
            min_flex_str = ','.join(map(str, [int(x) for x in self.calibration_data['flex_min']]))
            command = f"SET_CAL:{hand_prefix}:MIN_FLEX:{min_flex_str}\n"
            ser = serial.Serial(self.port, baud, timeout=1)
            ser.write(command.encode())
            time.sleep(0.1)
            
            # Send MAX_FLEX  
            max_flex_str = ','.join(map(str, [int(x) for x in self.calibration_data['flex_max']]))
            command = f"SET_CAL:{hand_prefix}:MAX_FLEX:{max_flex_str}\n"
            ser.write(command.encode())
            time.sleep(0.1)
            
            # Send MIN_PRESS
            min_press_str = ','.join(map(str, [int(x) for x in self.calibration_data['press_min']]))
            command = f"SET_CAL:{hand_prefix}:MIN_PRESS:{min_press_str}\n"
            ser.write(command.encode())
            time.sleep(0.1)
            
            # Send MAX_PRESS
            max_press_str = ','.join(map(str, [int(x) for x in self.calibration_data['press_max']]))
            command = f"SET_CAL:{hand_prefix}:MAX_PRESS:{max_press_str}\n"
            ser.write(command.encode())
            time.sleep(0.1)
            
            # Save to EEPROM
            ser.write(b"EEPROM_SAVE\n")
            time.sleep(0.2)
            
            # Also save to file for backup
            self.save_calibration_file()
            
            self.status_label.setText(f"✅ Calibration saved to Arduino EEPROM and file! Switch to NORMAL mode to test.")
            
        except Exception as e:
            self.status_label.setText(f"❌ Error saving calibration: {str(e)}")
    
    def save_calibration_file(self):
        """Save calibration data to backup file"""
        filename = "calibration.h"
        filepath = os.path.join(mainDir, "data", "calibration", filename)
        
        # Ensure directory exists
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        # Use a prefix based on the hand
        prefix = f"{self.selected_hand.upper()}_"
        
        # Generate the new calibration values
        new_content = []
        
        new_content.append(f"const int {prefix}MIN_FLEX[] = {{")
        for i in range(4):
            new_content.append(f"{int(self.calibration_data['flex_min'][i])}, ")
        new_content.append(f"{int(self.calibration_data['flex_min'][4])}}};\n")
        
        new_content.append(f"const int {prefix}MAX_FLEX[] = {{")
        for i in range(4):
            new_content.append(f"{int(self.calibration_data['flex_max'][i])}, ")
        new_content.append(f"{int(self.calibration_data['flex_max'][4])}}};\n")
        
        new_content.append(f"const int {prefix}MIN_PRESS[] = {{")
        for i in range(4):
            new_content.append(f"{int(self.calibration_data['press_min'][i])}, ")
        new_content.append(f"{int(self.calibration_data['press_min'][4])}}};\n")
        
        new_content.append(f"const int {prefix}MAX_PRESS[] = {{")
        for i in range(4):
            new_content.append(f"{int(self.calibration_data['press_max'][i])}, ")
        new_content.append(f"{int(self.calibration_data['press_max'][4])}}};\n")
        
        new_content = ''.join(new_content)
        
        # Check if the file exists
        file_exists = os.path.exists(filepath)
        
        if file_exists:
            with open(filepath, "r") as f:
                content = f.read()
            
            # Check if the specific hand's values already exist
            if prefix + "MIN_FLEX" in content:
                # Update the values using regex
                import re
                content = re.sub(f"const int {prefix}MIN_FLEX\\[.*?\\] = \\{{.*?\\}};", 
                                new_content.split('\n')[0], content)
                content = re.sub(f"const int {prefix}MAX_FLEX\\[.*?\\] = \\{{.*?\\}};", 
                                new_content.split('\n')[1], content)
                content = re.sub(f"const int {prefix}MIN_PRESS\\[.*?\\] = \\{{.*?\\}};", 
                                new_content.split('\n')[2], content)
                content = re.sub(f"const int {prefix}MAX_PRESS\\[.*?\\] = \\{{.*?\\}};", 
                                new_content.split('\n')[3], content)
            else:
                # Append the new values
                content = content.replace("#endif", new_content + "\n#endif")
            
            # Write the updated content back to the file
            with open(filepath, "w") as f:
                f.write(content)
        else:
            # If the file doesn't exist, create it with the new values
            with open(filepath, "w") as f:
                f.write("#ifndef CALIBRATION_H\n")
                f.write("#define CALIBRATION_H\n")
                f.write("\n")
                f.write(new_content)
                f.write("\n")
                f.write("#endif\n")
        
        # Copy the file to the Arduino directory for backup
        arduino_path = os.path.join(mainDir, "..", "arduino", "sendsens", "calibration.h")
        shutil.copy(filepath, arduino_path)
        
    def reset_to_defaults(self):
        """Reset calibration data to defaults from calibration.h"""
        try:
            self.status_label.setText("📤 Resetting Arduino to calibration.h defaults...")
            QApplication.processEvents()
            
            # Send reset command to Arduino
            ser = serial.Serial(self.port, baud, timeout=1)
            time.sleep(0.5)
            
            # Set hand first
            ser.write(f'HAND_{self.selected_hand.upper()}\n'.encode())
            time.sleep(0.3)
            
            # Reset EEPROM to defaults
            ser.write(b"EEPROM_RESET\n")
            time.sleep(0.5)
            
            # Read response
            response = ser.read_all().decode('utf-8', errors='ignore')
            print(f"Reset response: {response}")
            
            ser.close()
            
            # Reset local calibration data
            self.calibration_data = {
                'flex_min': None,
                'flex_max': None,
                'press_min': None,
                'press_max': [None] * 5
            }
            
            self.save_btn.setEnabled(False)
            
            self.status_label.setText("✅ Reset to calibration.h defaults complete!\nArduino now uses values from calibration.h")
            
        except Exception as e:
            self.status_label.setText(f"❌ Error resetting to defaults: {str(e)}")
        
    def closeEvent(self, event):
        """Clean up when closing"""
        if self.reader:
            self.reader.stop_readers()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CalibrationToolWithRealTimePlots()
    window.show()
    sys.exit(app.exec_()) 