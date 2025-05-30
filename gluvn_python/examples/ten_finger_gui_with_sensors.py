#!/usr/bin/env python3
"""
Ten Finger Hardware GUI with Real-Time Sensor Values

Enhanced version of the ten finger GUI that displays actual sensor readings
in real-time above each trigger indicator. This helps debug trigger thresholds
and understand sensor behavior.

Features:
- Real-time sensor value display (flex/pressure values)
- Visual finger lights showing trigger states
- Configurable thresholds with visual threshold lines
- Live sensor value sliders/progress bars
- MIDI output for triggered notes

Author: Joseph Bakarji
Usage: python ten_finger_gui_with_sensors.py
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
                            QMessageBox, QSizePolicy, QProgressBar, QSlider)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QPalette, QColor, QPaintEvent, QPainter, QBrush

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import GLUVN components
from core.strategies.trigger_strategies import HysteresisTrigger
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


class FingerSensorWidget(QWidget):
    """
    Enhanced finger widget showing both sensor value and trigger state
    """
    
    def __init__(self, finger_name, note_name, sensor_type="flex"):
        super().__init__()
        self.finger_name = finger_name
        self.note_name = note_name
        self.sensor_type = sensor_type
        self.is_triggered = False
        self.sensor_value = 0
        self.threshold = 120
        
        self.setFixedSize(120, 200)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        
        # Create layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)
        
        # Finger name label
        self.finger_label = QLabel(finger_name)
        self.finger_label.setAlignment(Qt.AlignCenter)
        self.finger_label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(self.finger_label)
        
        # Sensor value display
        self.value_label = QLabel("0")
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.value_label.setStyleSheet("color: blue; background-color: lightgray; padding: 2px;")
        layout.addWidget(self.value_label)
        
        # Sensor value progress bar (vertical)
        self.sensor_bar = QProgressBar()
        self.sensor_bar.setOrientation(Qt.Vertical)
        self.sensor_bar.setRange(0, 1023)  # Typical Arduino analog range
        self.sensor_bar.setValue(0)
        self.sensor_bar.setFixedHeight(80)
        self.sensor_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid grey;
                border-radius: 3px;
                text-align: center;
                background-color: #f0f0f0;
            }
            QProgressBar::chunk {
                background-color: #3498db;
                border-radius: 2px;
            }
        """)
        layout.addWidget(self.sensor_bar)
        
        # Threshold indicator
        self.threshold_label = QLabel(f"Thresh: {self.threshold}")
        self.threshold_label.setAlignment(Qt.AlignCenter)
        self.threshold_label.setFont(QFont("Arial", 8))
        self.threshold_label.setStyleSheet("color: red;")
        layout.addWidget(self.threshold_label)
        
        # Trigger light (circle)
        self.light_widget = QWidget()
        self.light_widget.setFixedSize(40, 40)
        layout.addWidget(self.light_widget, 0, Qt.AlignCenter)
        
        # Note label
        self.note_label = QLabel(note_name)
        self.note_label.setAlignment(Qt.AlignCenter)
        self.note_label.setFont(QFont("Arial", 9))
        layout.addWidget(self.note_label)
    
    def paintEvent(self, event):
        """Custom paint event to draw the trigger light"""
        super().paintEvent(event)
        
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Calculate light position relative to light_widget
        light_rect = self.light_widget.geometry()
        center_x = light_rect.x() + light_rect.width() // 2
        center_y = light_rect.y() + light_rect.height() // 2
        radius = 15
        
        # Draw light circle
        if self.is_triggered:
            color = QColor(0, 255, 0)  # Green when triggered
        else:
            color = QColor(139, 0, 0)  # Dark red when not triggered
        
        painter.setBrush(QBrush(color))
        painter.setPen(QColor(0, 0, 0))
        painter.drawEllipse(center_x - radius, center_y - radius, radius * 2, radius * 2)
        
        # Draw threshold line on progress bar
        if hasattr(self, 'sensor_bar'):
            bar_rect = self.sensor_bar.geometry()
            threshold_y = bar_rect.y() + bar_rect.height() - int((self.threshold / 1023.0) * bar_rect.height())
            painter.setPen(QColor(255, 0, 0, 180))  # Semi-transparent red
            painter.drawLine(bar_rect.x(), threshold_y, bar_rect.x() + bar_rect.width(), threshold_y)
    
    def update_sensor_value(self, value):
        """Update the sensor value display"""
        self.sensor_value = int(value)
        self.value_label.setText(str(self.sensor_value))
        self.sensor_bar.setValue(self.sensor_value)
        
        # Update progress bar color based on threshold
        if self.sensor_value >= self.threshold:
            color = "#e74c3c"  # Red when above threshold
        else:
            color = "#3498db"  # Blue when below threshold
        
        self.sensor_bar.setStyleSheet(f"""
            QProgressBar {{
                border: 2px solid grey;
                border-radius: 3px;
                text-align: center;
                background-color: #f0f0f0;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 2px;
            }}
        """)
        
        self.update()  # Trigger repaint for threshold line
    
    def set_triggered(self, triggered):
        """Set the trigger state"""
        if triggered != self.is_triggered:
            self.is_triggered = triggered
            self.update()  # Trigger repaint
    
    def set_threshold(self, threshold):
        """Update the threshold value"""
        self.threshold = threshold
        self.threshold_label.setText(f"Thresh: {threshold}")
        self.update()  # Trigger repaint for threshold line
    
    def update_note(self, note_name):
        """Update the note name"""
        self.note_name = note_name
        self.note_label.setText(note_name)


class SensorProcessingThread(QThread):
    """Thread for processing real sensor data from hardware"""
    
    sensor_update = pyqtSignal(str, int, int, bool, int)  # hand, finger_idx, sensor_value, triggered, switch_event
    error_signal = pyqtSignal(str)
    
    def __init__(self, reader, threshold, hysteresis, sensor_type='flex'):
        super().__init__()
        self.reader = reader
        self.threshold = threshold
        self.hysteresis = hysteresis
        self.sensor_type = sensor_type
        self.running = False
        
        # Track trigger states for hysteresis (like original SensorProcess)
        self.turn_state = {'l': np.zeros(5, dtype=bool), 'r': np.zeros(5, dtype=bool)}
        self.trig_on = {'l': np.zeros(5, dtype=bool), 'r': np.zeros(5, dtype=bool)}
        self.trig_off = {'l': np.zeros(5, dtype=bool), 'r': np.zeros(5, dtype=bool)}
    
    def trigger_logic(self, sensor_values, hand):
        """
        Implement exact same hysteresis-based triggering logic as original SensorProcess
        Returns n_switch array: 1 = turn on, -1 = turn off, 0 = no change
        """
        trigon_prev = self.trig_on[hand]
        trigoff_prev = self.trig_off[hand]
        sensarr = np.asarray(sensor_values[:5])  # Ensure we only process 5 fingers
        
        # Pad with zeros if we have fewer than 5 sensor values
        if len(sensarr) < 5:
            padded = np.zeros(5)
            padded[:len(sensarr)] = sensarr
            sensarr = padded
        
        sdiff = sensarr - self.threshold        # subtract threshold from readings
        trigon = sdiff - self.hysteresis > 0    # Above threshold + hysteresis
        trigoff = sdiff + self.hysteresis < 0   # Below threshold - hysteresis
        
        # Edge detection: turn on only if wasn't on before and not currently triggered
        turnon = np.logical_and(np.logical_and(trigon, np.logical_not(trigon_prev)), 
                               np.logical_not(self.turn_state[hand]))
        
        # Edge detection: turn off only if wasn't off before and currently triggered  
        turnoff = np.logical_and(np.logical_and(trigoff, np.logical_not(trigoff_prev)), 
                                self.turn_state[hand])
        
        # Calculate switch events: 1 = turn on, -1 = turn off, 0 = no change
        n_switch = turnon.astype(int) - turnoff.astype(int)
        
        # Update states
        self.turn_state[hand] = n_switch + self.turn_state[hand]
        self.trig_on[hand] = trigon
        self.trig_off[hand] = trigoff

        return n_switch
    
    def run(self):
        """Main sensor processing loop"""
        try:
            if not self.reader:
                self.error_signal.emit("No hardware reader available")
                return
                
            self.reader.start_readers()
            print("Sensor processing started")
            
            # Debug: Print available hands and threads
            print(f"Available reader threads: {list(self.reader.threads.keys())}")
            for hand in self.reader.threads:
                print(f"Hand {hand} threads: {list(self.reader.threads[hand].keys())}")
            
            while self.running:
                try:
                    # Process each hand
                    for hand in ['l', 'r']:
                        if hand not in self.reader.threads or 'parser' not in self.reader.threads[hand]:
                            continue
                        
                        parser_queue = self.reader.threads[hand]['parser'].getQ()
                        
                        # Get latest data by draining the queue (like simple_sensor_test.py)
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
                            sensor_values = latest_data[self.sensor_type]
                            
                            # Debug: Print what data we received (less frequently)
                            if data_count > 0:  # Only print when we actually got new data
                                print(f"📊 {hand.upper()}: {self.sensor_type} = {sensor_values} (processed {data_count} samples)")
                            
                            # Apply trigger logic to get switch events for all fingers at once
                            n_switch = self.trigger_logic(sensor_values, hand)
                            
                            # Process each finger
                            for finger_idx in range(min(5, len(sensor_values))):
                                value = sensor_values[finger_idx]
                                switch_event = n_switch[finger_idx]
                                currently_triggered = self.turn_state[hand][finger_idx]
                                
                                # Debug switch events
                                if switch_event == 1:
                                    print(f"🔥 TRIGGER ON: {hand.upper()} finger {finger_idx}, value={value}, threshold={self.threshold}")
                                elif switch_event == -1:
                                    print(f"⚪ TRIGGER OFF: {hand.upper()} finger {finger_idx}, value={value}, threshold={self.threshold}")
                                
                                # Emit sensor update signal with switch event
                                self.sensor_update.emit(hand, finger_idx, int(value), currently_triggered, switch_event)
                    
                    # Small delay to prevent excessive CPU usage
                    self.msleep(50)  # 50ms = 20 FPS
                    
                except Exception as e:
                    self.error_signal.emit(f"Error processing sensors: {e}")
                    print(f"Exception in sensor processing: {e}")
                    self.msleep(100)
                    
        except Exception as e:
            self.error_signal.emit(f"Failed to start sensor reader: {e}")
            print(f"Exception starting sensor reader: {e}")
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
    
    def update_threshold(self, threshold):
        """Update threshold value"""
        self.threshold = threshold
    
    def update_hysteresis(self, hysteresis):
        """Update hysteresis value"""
        self.hysteresis = hysteresis


class TenFingerSensorGUI(QMainWindow):
    """Main GUI application with real-time sensor value display"""
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("GLUVN Ten Finger GUI - Real-Time Sensor Values")
        self.setGeometry(100, 100, 1200, 800)
        
        # Application state
        self.running = False
        self.sensor_thread = None
        self.reader = None
        
        # Finger widgets storage
        self.finger_widgets = {'l': [], 'r': []}
        
        # Configuration
        self.threshold = 120
        self.hysteresis = 10
        self.sensor_type = 'flex'
        
        # Initialize MIDI writer FIRST
        try:
            self.midi_writer = MidiWriter()
            self.note_mapper = NoteMapper(root_note='C', scale='major')
            self.note_maps = self.note_mapper.basic_map_2hands()
            self.midi_available = True
        except Exception as e:
            print(f"Warning: MIDI not available: {e}")
            self.midi_writer = None
            self.midi_available = False
        
        # Initialize components
        self.setup_hardware()
        self.setup_ui()
    
    def setup_hardware(self):
        """Initialize hardware reader"""
        if HARDWARE_AVAILABLE:
            try:
                sensor_config = {
                    'l': {'flex': True, 'press': True, 'imu': False},
                    'r': {'flex': True, 'press': True, 'imu': False}
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
        """Create the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # Title
        title = QLabel("GLUVN Ten Finger GUI - Real-Time Sensor Values")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Bold))
        main_layout.addWidget(title)
        
        # Status info
        if self.hardware_available:
            status_text = "✅ Hardware Connected - Real sensor data will be displayed"
            status_color = "lightgreen"
        else:
            status_text = "❌ Hardware Not Available - Check connection and restart"
            status_color = "lightcoral"
        
        status_label = QLabel(status_text)
        status_label.setAlignment(Qt.AlignCenter)
        status_label.setStyleSheet(f"background-color: {status_color}; padding: 10px; border-radius: 5px;")
        main_layout.addWidget(status_label)
        
        # Control panel
        self.create_control_panel(main_layout)
        
        # Hands display
        self.create_hands_display(main_layout)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.status_bar.showMessage("Ready - Click 'Start Sensors' to begin")
        self.setStatusBar(self.status_bar)
    
    def create_control_panel(self, main_layout):
        """Create control panel"""
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
        
        # Configuration
        config_layout = QGridLayout()
        
        # Threshold
        config_layout.addWidget(QLabel("Threshold:"), 0, 0)
        self.threshold_spinbox = QSpinBox()
        self.threshold_spinbox.setRange(50, 500)
        self.threshold_spinbox.setValue(120)
        self.threshold_spinbox.valueChanged.connect(self.update_threshold)
        config_layout.addWidget(self.threshold_spinbox, 0, 1)
        
        # Hysteresis
        config_layout.addWidget(QLabel("Hysteresis:"), 0, 2)
        self.hysteresis_spinbox = QSpinBox()
        self.hysteresis_spinbox.setRange(1, 50)
        self.hysteresis_spinbox.setValue(self.hysteresis)
        self.hysteresis_spinbox.valueChanged.connect(self.update_hysteresis)
        config_layout.addWidget(self.hysteresis_spinbox, 0, 3)
        
        # Sensor type
        config_layout.addWidget(QLabel("Sensor:"), 1, 0)
        self.sensor_combo = QComboBox()
        self.sensor_combo.addItems(['flex', 'press'])
        self.sensor_combo.currentTextChanged.connect(self.update_sensor_type)
        config_layout.addWidget(self.sensor_combo, 1, 1)
        
        control_layout.addLayout(config_layout)
    
    def create_hands_display(self, main_layout):
        """Create the hands display with finger sensors"""
        hands_group = QGroupBox("Finger Sensors - Real-Time Values")
        main_layout.addWidget(hands_group)
        
        hands_layout = QHBoxLayout(hands_group)
        
        # Left hand
        left_group = QGroupBox("Left Hand")
        hands_layout.addWidget(left_group)
        left_layout = QHBoxLayout(left_group)
        
        finger_names = ["Pinky", "Ring", "Middle", "Index", "Thumb"]
        
        for i, finger_name in enumerate(finger_names):
            note_name = "---"
            if self.midi_available and 'l' in self.note_maps:
                if i < len(self.note_maps['l']):
                    note_name = self.note_mapper.midi2note.get(self.note_maps['l'][i], "---")
            
            widget = FingerSensorWidget(finger_name, note_name, self.sensor_type)
            widget.set_threshold(self.threshold)
            self.finger_widgets['l'].append(widget)
            left_layout.addWidget(widget)
        
        # Right hand
        right_group = QGroupBox("Right Hand")
        hands_layout.addWidget(right_group)
        right_layout = QHBoxLayout(right_group)
        
        for i, finger_name in enumerate(["Thumb", "Index", "Middle", "Ring", "Pinky"]):
            note_name = "---"
            if self.midi_available and 'r' in self.note_maps:
                if i < len(self.note_maps['r']):
                    note_name = self.note_mapper.midi2note.get(self.note_maps['r'][i], "---")
            
            widget = FingerSensorWidget(finger_name, note_name, self.sensor_type)
            widget.set_threshold(self.threshold)
            self.finger_widgets['r'].append(widget)
            right_layout.addWidget(widget)
    
    def start_sensors(self):
        """Start sensor processing"""
        if not self.hardware_available:
            QMessageBox.warning(self, "Hardware Error", "No hardware available!")
            return
        
        try:
            self.sensor_thread = SensorProcessingThread(
                self.reader, self.threshold, self.hysteresis, self.sensor_type
            )
            self.sensor_thread.sensor_update.connect(self.on_sensor_update)
            self.sensor_thread.error_signal.connect(self.on_error)
            self.sensor_thread.start_processing()
            
            self.running = True
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.status_bar.showMessage("Sensors running - Flex your fingers!")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start sensors: {e}")
    
    def stop_sensors(self):
        """Stop sensor processing"""
        if self.sensor_thread:
            self.sensor_thread.stop_processing()
            self.sensor_thread = None
        
        self.running = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_bar.showMessage("Sensors stopped")
    
    def on_sensor_update(self, hand, finger_idx, sensor_value, triggered, switch_event):
        """Handle sensor data updates"""
        if finger_idx < len(self.finger_widgets[hand]):
            widget = self.finger_widgets[hand][finger_idx]
            widget.update_sensor_value(sensor_value)
            widget.set_triggered(triggered)
            
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
        print(f"Sensor error: {error_message}")
        self.status_bar.showMessage(f"Error: {error_message}")
    
    def update_threshold(self, value):
        """Update threshold value"""
        self.threshold = value
        for hand in ['l', 'r']:
            for widget in self.finger_widgets[hand]:
                widget.set_threshold(value)
        
        if self.sensor_thread:
            self.sensor_thread.update_threshold(value)
    
    def update_hysteresis(self, value):
        """Update hysteresis value"""
        self.hysteresis = value
        if self.sensor_thread:
            self.sensor_thread.update_hysteresis(value)
    
    def update_sensor_type(self, sensor_type):
        """Update sensor type"""
        self.sensor_type = sensor_type
        # Note: Would need to restart sensors to change sensor type
        self.status_bar.showMessage(f"Sensor type changed to {sensor_type} - Restart sensors to apply")
    
    def closeEvent(self, event):
        """Handle application close"""
        if self.running:
            self.stop_sensors()
        event.accept()


def main():
    """Main application entry point"""
    print("Starting GLUVN Ten Finger GUI with Real-Time Sensor Values")
    print("=" * 60)
    print("This application shows real-time sensor readings above each finger trigger.")
    print("You can see the actual flex/pressure values and how they relate to thresholds.")
    print("=" * 60)
    
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Create and show main window
    window = TenFingerSensorGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 