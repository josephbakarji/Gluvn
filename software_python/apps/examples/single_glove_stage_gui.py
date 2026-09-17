#!/usr/bin/env python3
import sys, os, time, math
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QGroupBox, QGridLayout, QLabel,
                             QComboBox, QPushButton, QStatusBar, QCheckBox,
                             QProgressBar, QSlider)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from mapper import NoteMapper
from midi_writer import MidiWriter
from visualization.sensor_threads import SensorProcessingThread
from visualization import TenFingerDisplay
try:
    from port_read import Reader
    HARDWARE_AVAILABLE = True
except Exception:
    Reader = None
    HARDWARE_AVAILABLE = False

FLEX_COUNT = 5
PRESS_COUNT = 5

class CustomReader:
    """Custom Reader class that accepts a specific port"""
    def __init__(self, port, sensor_config={'r': {'flex': True, 'press': True, 'imu': True}}):
        from __init__ import baud, EXPDIR
        from port_read import ReadSerial, ParseSerial
        
        self.port = port
        self.baud = baud
        self.hands = list(sensor_config.keys())
        self.sensor_config = sensor_config
        self.directory = EXPDIR
        self.message_format = '>sBHHHHHHBBBBBBBBBB'
        self.length_checksum = 24
        self.parse_qsize = 30
        self.threads = self.make_reader_threads()
        self.printers = self.make_printer_threads()
    
    def make_reader_threads(self):
        import time
        from port_read import ReadSerial, ParseSerial
        
        time0 = time.time()
        threads = {hand: {} for hand in self.hands}
        for hand in self.hands:
            threads[hand]['port'] = self.port  # Use the specified port
            print(f"CustomReader using port: {self.port}")  # Debug output
            threads[hand]['serial'] = ReadSerial(threads[hand]['port'], self.baud)
            threads[hand]['parser'] = ParseSerial(
                threads[hand]['serial'].getQ(), 
                time0, 
                format=self.message_format, 
                length_checksum=self.length_checksum,
                **self.sensor_config[hand], 
                qsize=self.parse_qsize
            )
        return threads
    
    def make_printer_threads(self):
        from port_read import printSens
        printers = {}
        for hand in self.hands:
            printers[hand] = printSens(self.threads[hand]['parser'].getQ())
        return printers
    
    def start_readers(self):
        for hand in self.hands:
            self.threads[hand]['serial'].start()
            self.threads[hand]['parser'].start()
    
    def stop_readers(self):
        for hand in self.hands:
            self.threads[hand]['serial'].join(timeout=1)
            self.threads[hand]['parser'].join(timeout=1)
    
    def start_printers(self):
        for hand in self.hands:
            self.printers[hand].start()
    
    def stop_printers(self):
        for hand in self.hands:
            self.printers[hand].join(timeout=1)
    
    def get_latest_data(self, hand):
        """Get latest sensor data for a hand"""
        if hand in self.threads:
            return self.threads[hand]['parser'].get_latest_data()
        return None

class IMUPlot(QWidget):
    def __init__(self):
        super().__init__()
        layout = QHBoxLayout(self)
        self.plot = pg.PlotWidget(title='Right Hand IMU (Yaw/Pitch/Roll)')
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel('left', 'Value')
        self.plot.setLabel('bottom', 'Sample')
        layout.addWidget(self.plot)
        self.max_points = 500
        self.x = list(range(self.max_points))
        self.yaw = deque([0]*self.max_points, maxlen=self.max_points)
        self.pitch = deque([0]*self.max_points, maxlen=self.max_points)
        self.roll = deque([0]*self.max_points, maxlen=self.max_points)
        self.cur_yaw = self.plot.plot(pen=pg.mkPen(color=(255,0,0), width=2), name='Yaw')
        self.cur_pitch = self.plot.plot(pen=pg.mkPen(color=(0,255,0), width=2), name='Pitch')
        self.cur_roll = self.plot.plot(pen=pg.mkPen(color=(0,0,255), width=2), name='Roll')
    def update(self, imu_array):
        if not isinstance(imu_array, (list, tuple)) or len(imu_array) < 3:
            return
        self.yaw.append(imu_array[0]); self.pitch.append(imu_array[1]); self.roll.append(imu_array[2])
        self.cur_yaw.setData(self.x[-len(self.yaw):], list(self.yaw))
        self.cur_pitch.setData(self.x[-len(self.pitch):], list(self.pitch))
        self.cur_roll.setData(self.x[-len(self.roll):], list(self.roll))

class SingleGloveStageGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('GLUVN Single-Glove Stage GUI')
        self.setGeometry(100, 100, 1400, 900)
        self.running = False
        self.reader = None
        self.sensor_thread = None
        self.midi_writer = None
        self.note_mapper = NoteMapper(root_note='C', scale='major')
        # Trigger config defaults
        self.sensor_type = 'flex'
        self.threshold = 120
        self.hysteresis = 10
        # IMU CC send throttle
        self.imu_cc_last = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.imu_cc_interval = 0.02
        self.print_enabled = False
        self._initialize_midi()
        self._initialize_hardware()
        # Note maps for indicator widget (right hand only)
        self.note_maps = {'r': [60,62,64,65,67]}
        self._setup_ui()

    def _initialize_midi(self):
        try:
            self.midi_writer = MidiWriter()
            print("MIDI writer initialized successfully")
        except Exception as e:
            print(f"Failed to initialize MIDI writer: {e}")
            self.midi_writer = None

    def _initialize_hardware(self):
        if HARDWARE_AVAILABLE:
            try:
                sensor_config = {'r': {'flex': True, 'press': True, 'imu': True}}
                self.reader = Reader(sensor_config=sensor_config)
                self.hardware_available = True
            except Exception:
                self.reader = None
                self.hardware_available = False
        else:
            self.hardware_available = False

    def _create_reader_with_port(self, port):
        """Create a Reader instance with a specific port"""
        if not HARDWARE_AVAILABLE:
            return None
        try:
            # Create a custom reader that uses the specified port
            sensor_config = {'r': {'flex': True, 'press': True, 'imu': True}}
            reader = CustomReader(port, sensor_config=sensor_config)
            return reader
        except Exception as e:
            print(f"Error creating reader with port {port}: {e}")
            return None

    def _setup_ui(self):
        central = QWidget(); self.setCentralWidget(central)
        main = QVBoxLayout(central)
        title = QLabel('GLUVN Single-Glove Stage Controller')
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont('Arial', 18, QFont.Bold))
        main.addWidget(title)
        # Controls row
        controls = QHBoxLayout()
        self.start_btn = QPushButton('Start')
        self.start_btn.clicked.connect(self.start)
        controls.addWidget(self.start_btn)
        self.stop_btn = QPushButton('Stop')
        self.stop_btn.clicked.connect(self.stop)
        self.stop_btn.setEnabled(False)
        controls.addWidget(self.stop_btn)
        self.print_cb = QCheckBox('Print')
        self.print_cb.stateChanged.connect(lambda s: setattr(self, 'print_enabled', s == Qt.Checked))
        controls.addWidget(self.print_cb)
        # Port selection
        controls.addWidget(QLabel('Port:'))
        self.port_combo = QComboBox()
        self.port_combo.currentTextChanged.connect(self._on_port_changed)
        self._populate_ports()
        controls.addWidget(self.port_combo)
        # Refresh ports button
        refresh_btn = QPushButton('Refresh')
        refresh_btn.clicked.connect(self._populate_ports)
        controls.addWidget(refresh_btn)
        # Test MIDI button
        test_midi_btn = QPushButton('Test MIDI')
        test_midi_btn.clicked.connect(self._test_midi)
        controls.addWidget(test_midi_btn)
        main.addLayout(controls)
        # Mapping panel for sensors
        map_group = QGroupBox('Right Hand Sensor Mapping')
        grid = QGridLayout(map_group)
        grid.addWidget(QLabel('Sensor'), 0, 0)
        grid.addWidget(QLabel('Type'), 0, 1)
        grid.addWidget(QLabel('Value'), 0, 2)
        self.flex_type = []; self.flex_val = []
        for i in range(FLEX_COUNT):
            grid.addWidget(QLabel(f'Flex {i+1}'), i+1, 0)
            t = QComboBox(); t.addItems(['CC', 'Note'])
            v = QComboBox(); v.addItems([str(n) for n in range(0,128)])
            # defaults: CC 1..5
            t.setCurrentIndex(0)
            v.setCurrentText(str(i+1))
            self.flex_type.append(t); self.flex_val.append(v)
            grid.addWidget(t, i+1, 1); grid.addWidget(v, i+1, 2)
        base = 1 + FLEX_COUNT
        self.press_type = []; self.press_val = []
        for i in range(PRESS_COUNT):
            grid.addWidget(QLabel(f'Press {i+1}'), base+i, 0)
            t = QComboBox(); t.addItems(['CC', 'Note'])
            v = QComboBox(); v.addItems([str(n) for n in range(0,128)])
            # defaults: Notes 60..64
            t.setCurrentIndex(1)
            v.setCurrentText(str(60+i))
            self.press_type.append(t); self.press_val.append(v)
            grid.addWidget(t, base+i, 1); grid.addWidget(v, base+i, 2)
        # Real-time sensor indicators (right hand only) side-by-side with mapping
        top_row = QHBoxLayout()
        # Flex indicators (right hand only)
        self.flex_display = self._create_single_hand_display('flex', 'Flex Sensors (Right Hand)')
        top_row.addWidget(self.flex_display, 1)
        # Pressure indicators (right hand only)
        self.press_display = self._create_single_hand_display('press', 'Pressure Sensors (Right Hand)')
        top_row.addWidget(self.press_display, 1)
        # Mapping group on the right
        top_row.addWidget(map_group, 1)
        main.addLayout(top_row)
        # IMU CC mapping (Yaw/Pitch/Roll)
        imu_group = QGroupBox('IMU CC Mapping (Yaw/Pitch/Roll)')
        imu_layout = QVBoxLayout(imu_group)
        
        # Top row: CC selection dropdowns
        imu_row = QHBoxLayout()
        self.imu_cc_boxes = {}
        # Default CC values: Yaw=50, Pitch=51, Roll=52
        default_ccs = {'yaw': 50, 'pitch': 51, 'roll': 52}
        for name in ['Yaw', 'Pitch', 'Roll']:
            box = QComboBox(); box.addItem('None')
            for n in range(128): box.addItem(str(n))
            # Set default CC value
            default_cc = default_ccs[name.lower()]
            box.setCurrentText(str(default_cc))
            imu_row.addWidget(QLabel(name)); imu_row.addWidget(box)
            self.imu_cc_boxes[name.lower()] = box
        imu_layout.addLayout(imu_row)
        
        # Bottom row: Value indicators
        imu_indicators_row = QHBoxLayout()
        self.imu_indicators = {}
        for name in ['Yaw', 'Pitch', 'Roll']:
            # Create indicator group
            indicator_group = QGroupBox(f'{name} Value')
            indicator_layout = QVBoxLayout(indicator_group)
            
            # Value label
            value_label = QLabel('0')
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setFont(QFont('Arial', 14, QFont.Bold))
            value_label.setStyleSheet("color: blue; background-color: lightgray; padding: 5px; border-radius: 3px;")
            indicator_layout.addWidget(value_label)
            
            # Progress bar
            progress_bar = QProgressBar()
            progress_bar.setRange(0, 127)
            progress_bar.setValue(0)
            progress_bar.setStyleSheet("""
                QProgressBar {
                    border: 1px solid grey;
                    border-radius: 3px;
                    text-align: center;
                }
                QProgressBar::chunk {
                    background-color: #3498db;
                    border-radius: 2px;
                }
            """)
            indicator_layout.addWidget(progress_bar)
            
            # Store references
            self.imu_indicators[name.lower()] = {
                'label': value_label,
                'bar': progress_bar
            }
            
            imu_indicators_row.addWidget(indicator_group)
        
        imu_layout.addLayout(imu_indicators_row)
        main.addWidget(imu_group)
        # IMU plot under all
        self.imu_plot = IMUPlot();
        main.addWidget(self.imu_plot)
        # Status
        self.status = QStatusBar(); self.setStatusBar(self.status)
        self.status.showMessage('Ready')

    def _create_single_hand_display(self, sensor_type, title):
        """Create a single-hand display widget showing only right hand"""
        from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel
        from visualization import FingerSensorWidget
        
        group = QGroupBox(title)
        layout = QVBoxLayout(group)
        
        # Create right hand only
        right_group = QGroupBox("Right Hand")
        right_layout = QHBoxLayout(right_group)
        
        finger_names_right = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        
        # Initialize finger_widgets if not exists
        if not hasattr(self, 'finger_widgets'):
            self.finger_widgets = {'r': []}
        
        # Create widgets for this sensor type
        widgets = []
        for i, finger_name in enumerate(finger_names_right):
            note_name = "---"
            if 'r' in self.note_maps and i < len(self.note_maps['r']):
                note_name = str(self.note_maps['r'][i])
            
            widget = FingerSensorWidget(finger_name, note_name, sensor_type)
            # Fix pressure sensor range - they use 0-255 like flex sensors, not 0-1023
            if sensor_type == 'press':
                widget.sensor_bar.setRange(0, 255)
            widgets.append(widget)
            right_layout.addWidget(widget)
        
        # Store widgets by sensor type
        if not hasattr(self, 'sensor_widgets'):
            self.sensor_widgets = {}
        self.sensor_widgets[sensor_type] = widgets
        
        layout.addWidget(right_group)
        return group

    def _populate_ports(self):
        """Populate the port combo box with available USB serial ports"""
        import glob
        import os
        
        # Common USB serial port patterns
        port_patterns = [
            '/dev/cu.usbmodem*',  # macOS
            '/dev/ttyUSB*',       # Linux
            '/dev/ttyACM*',       # Linux
            'COM*'                # Windows
        ]
        
        available_ports = []
        for pattern in port_patterns:
            found_ports = glob.glob(pattern)
            available_ports.extend(found_ports)
            print(f"Pattern {pattern} found: {found_ports}")  # Debug output
        
        # Sort ports for consistent ordering
        available_ports.sort()
        
        # Always add common GLUVN ports as options (they might not be detected by glob)
        common_ports = ['/dev/cu.usbmodem101', '/dev/cu.usbmodem1101', '/dev/cu.usbmodem201', '/dev/cu.usbmodem2101']
        for port in common_ports:
            if port not in available_ports:
                available_ports.append(port)
        
        # Update combo box (temporarily disconnect signal to avoid interference)
        current_selection = self.port_combo.currentText()
        try:
            self.port_combo.currentTextChanged.disconnect()
        except TypeError:
            pass  # Signal not connected yet
        self.port_combo.clear()
        self.port_combo.addItems(available_ports)
        
        # Restore selection if it still exists, otherwise select first available
        if current_selection in available_ports:
            self.port_combo.setCurrentText(current_selection)
        elif available_ports:
            self.port_combo.setCurrentText(available_ports[0])
        
        # Reconnect signal
        self.port_combo.currentTextChanged.connect(self._on_port_changed)
        
        print(f"Final available ports: {available_ports}")  # Debug output
        print(f"Current selection: {self.port_combo.currentText()}")  # Debug output

    def _on_port_changed(self, port_text):
        """Handle port selection change"""
        print(f"Port changed to: {port_text}")  # Debug output

    def _test_midi(self):
        """Test MIDI output"""
        if self.midi_writer:
            try:
                # Send a test note
                self.midi_writer.trig_note(60, 64)  # Middle C
                print("Test MIDI note sent: C4 (60)")
                # Send a test CC
                self.midi_writer.control_change(1, 64)  # CC1 = 64
                print("Test MIDI CC sent: CC1 = 64")
            except Exception as e:
                print(f"MIDI test failed: {e}")
        else:
            print("No MIDI writer available")

    def start(self):
        if self.running: return
        
        # Get selected port and create reader
        selected_port = self.port_combo.currentText()
        print(f"Selected port: {selected_port}")  # Debug output
        self.reader = self._create_reader_with_port(selected_port)
        
        if not self.reader:
            self.status.showMessage(f'Error: Could not connect to port {selected_port}')
            return
            
        try:
            # Create separate sensor threads for both flex and pressure
            self.sensor_threads = {}
            
            # Flex sensor thread
            flex_config = {'thresholds': {'flex': self.threshold},
                          'hysteresis': {'flex': self.hysteresis},
                          'trigger_sensors': {'r': 'flex'}}
            self.sensor_threads['flex'] = SensorProcessingThread(self.reader, flex_config, 'flex', None, self.print_enabled)
            self.sensor_threads['flex'].sensor_update.connect(self.on_sensor_update)
            self.sensor_threads['flex'].raw_sensor_update.connect(self.on_raw)
            self.sensor_threads['flex'].error_signal.connect(self.on_error)
            
            # Pressure sensor thread
            press_config = {'thresholds': {'press': 25},
                           'hysteresis': {'press': 5},
                           'trigger_sensors': {'r': 'press'}}
            self.sensor_threads['press'] = SensorProcessingThread(self.reader, press_config, 'press', None, self.print_enabled)
            self.sensor_threads['press'].sensor_update.connect(self.on_sensor_update)
            self.sensor_threads['press'].raw_sensor_update.connect(self.on_raw)
            self.sensor_threads['press'].error_signal.connect(self.on_error)
            
            # Start both threads
            for thread in self.sensor_threads.values():
                thread.start_processing()
            
            self.running = True
            self.start_btn.setEnabled(False); self.stop_btn.setEnabled(True)
            self.status.showMessage(f'Running on {selected_port}')
            self.setWindowTitle(f'GLUVN Single-Glove Stage GUI - {selected_port}')
        except Exception as e:
            self.status.showMessage(f'Error: {e}')

    def stop(self):
        if hasattr(self, 'sensor_threads'):
            for thread in self.sensor_threads.values():
                thread.stop_processing()
            self.sensor_threads = None
        if self.reader:
            self.reader.stop_readers(); self.reader = None
        self.running = False
        self.start_btn.setEnabled(True); self.stop_btn.setEnabled(False)
        self.status.showMessage('Stopped')
        self.setWindowTitle('GLUVN Single-Glove Stage GUI')

    def on_error(self, msg):
        self.status.showMessage(msg)

    def on_sensor_update(self, hand, finger_idx, sensor_value, triggered, switch_event):
        # Send mapped messages for flex/press on right hand only
        if hand != 'r':
            return
        
        # Determine which sensor type this update is for based on the sender
        sender = self.sender()
        sensor_type = None
        if hasattr(self, 'sensor_threads') and self.sensor_threads is not None:
            for stype, thread in self.sensor_threads.items():
                if thread == sender:
                    sensor_type = stype
                    break
        
        if not sensor_type:
            return
        
        # Update visual indicator for the appropriate sensor type
        try:
            if hasattr(self, 'sensor_widgets') and sensor_type in self.sensor_widgets:
                widgets = self.sensor_widgets[sensor_type]
                if finger_idx < len(widgets):
                    widgets[finger_idx].update_sensor_value(sensor_value)
                    widgets[finger_idx].set_triggered(triggered)
        except Exception:
            pass
        
        if not self.midi_writer:
            return
        
        # Send MIDI messages based on sensor type and finger index
        if sensor_type == 'flex' and finger_idx < FLEX_COUNT:
            t = self.flex_type[finger_idx].currentText()
            val = int(self.flex_val[finger_idx].currentText())
            if t == 'CC':
                self.midi_writer.control_change(int(max(min(sensor_value,127),0)), val)
                if self.print_enabled:
                    print(f"Flex {finger_idx}: CC {val} = {sensor_value}")
            else:  # Note
                if switch_event == 1:
                    self.midi_writer.trig_note(val, 64)
                    if self.print_enabled:
                        print(f"Flex {finger_idx}: Note ON {val}")
                elif switch_event == -1:
                    self.midi_writer.trig_note(val, 0)
                    if self.print_enabled:
                        print(f"Flex {finger_idx}: Note OFF {val}")
        
        elif sensor_type == 'press' and finger_idx < PRESS_COUNT:
            t = self.press_type[finger_idx].currentText()
            val = int(self.press_val[finger_idx].currentText())
            if t == 'CC':
                self.midi_writer.control_change(int(max(min(sensor_value,127),0)), val)
                if self.print_enabled:
                    print(f"Press {finger_idx}: CC {val} = {sensor_value}")
            else:  # Note
                if switch_event == 1:
                    self.midi_writer.trig_note(val, 64)
                    if self.print_enabled:
                        print(f"Press {finger_idx}: Note ON {val}")
                elif switch_event == -1:
                    self.midi_writer.trig_note(val, 0)
                    if self.print_enabled:
                        print(f"Press {finger_idx}: Note OFF {val}")

    def on_raw(self, hand, raw):
        if hand != 'r': return
        if 'imu' in raw and isinstance(raw['imu'], (list, tuple)) and len(raw['imu']) >= 3:
            self.imu_plot.update(raw['imu'])
            # IMU CC mappings
            now = time.time()
            imu_axes = {'yaw': 0, 'pitch': 1, 'roll': 2}
            for name, idx in imu_axes.items():
                box = self.imu_cc_boxes.get(name)
                if not box: continue
                text = box.currentText()
                if text == 'None':
                    # Update indicator to show 0 when not mapped
                    if name in self.imu_indicators:
                        self.imu_indicators[name]['label'].setText('0')
                        self.imu_indicators[name]['bar'].setValue(0)
                    continue
                cc_num = int(text)
                raw_axis = int(raw['imu'][idx])
                cc_val = int(max(min((raw_axis/65535.0)*127.0, 127.0), 0.0))
                
                # Update visual indicators
                if name in self.imu_indicators:
                    self.imu_indicators[name]['label'].setText(str(cc_val))
                    self.imu_indicators[name]['bar'].setValue(cc_val)
                
                if self.midi_writer and (now - self.imu_cc_last[name]) >= self.imu_cc_interval:
                    self.midi_writer.control_change(cc_val, cc_num)
                    self.imu_cc_last[name] = now
                    if self.print_enabled:
                        print(f"IMU {name}: CC {cc_num} = {cc_val}")
        # Update displays with raw arrays if available
        try:
            if hasattr(self, 'sensor_widgets'):
                if 'flex' in raw and isinstance(raw['flex'], (list, tuple)) and 'flex' in self.sensor_widgets:
                    widgets = self.sensor_widgets['flex']
                    for i in range(min(FLEX_COUNT, len(raw['flex']))):
                        if i < len(widgets):
                            widgets[i].update_sensor_value(int(raw['flex'][i]))
                if 'press' in raw and isinstance(raw['press'], (list, tuple)) and 'press' in self.sensor_widgets:
                    widgets = self.sensor_widgets['press']
                    for i in range(min(PRESS_COUNT, len(raw['press']))):
                        if i < len(widgets):
                            widgets[i].update_sensor_value(int(raw['press'][i]))
        except Exception:
            pass


def main():
    app = QApplication(sys.argv)
    win = SingleGloveStageGUI(); win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
