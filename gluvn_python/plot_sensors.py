import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QGroupBox, QSplitter, QGridLayout
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg
from port_read import Reader
import time
import queue

class SensorPlotter(QMainWindow):
    # === USER-CONTROLLED PARAMETERS ===
    LINE_WIDTH = 4  # Line width for all plots
    POINTS_SKIP = 1  # Number of points to skip (downsampling factor)
    MAX_POINTS = 1000  # Buffer size
    # Colors for each channel (can be changed by user)
    FLEX_COLORS = [(255,0,0), (0,255,0), (0,0,255), (255,255,0), (0,255,255)]
    PRESS_COLORS = [(255,0,0), (0,255,0), (0,0,255), (255,255,0), (0,255,255)]
    ORIENTATION_COLORS = [(255,0,0), (0,255,0), (0,0,255)]  # Red, Green, Blue
    ACCEL_COLORS = [(255,128,0), (128,0,255), (0,128,255)]  # Orange, Purple, Cyan
    # ================================

    def __init__(self):
        super().__init__()
        self.setWindowTitle('Sensor Data Plotter')
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize sensor reading with both flex, pressure, and IMU sensors
        self.sensor_config = {'l': {'flex': True, 'press': True, 'imu': True},
                            'r': {'flex': True, 'press': True, 'imu': True}}
        self.reader = Reader(sensor_config=self.sensor_config, save=False)
        
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # Create control panel with checkboxes
        self.create_control_panel(main_layout)
        
        # Create side-by-side layout for sensors
        self.create_side_by_side_layout(main_layout)
        
        # Define buffer size
        self.max_points = self.MAX_POINTS
        
        # Initialize data storage with numpy arrays for better performance
        self.data_buffer = {
            'l': {'flex': np.zeros((self.max_points, 5), dtype=np.float32),
                  'press': np.zeros((self.max_points, 5), dtype=np.float32),
                  'imu': np.zeros((self.max_points, 6), dtype=np.float32)},
            'r': {'flex': np.zeros((self.max_points, 5), dtype=np.float32),
                  'press': np.zeros((self.max_points, 5), dtype=np.float32),
                  'imu': np.zeros((self.max_points, 6), dtype=np.float32)}
        }
        self.buffer_index = {
            'l': {'flex': 0, 'press': 0, 'imu': 0},
            'r': {'flex': 0, 'press': 0, 'imu': 0}
        }
        
        # Create curves with user-defined colors and line width
        self.curves = {
            'l': {
                'flex': [self.flex_plot_l.plot(pen=pg.mkPen(color=self.FLEX_COLORS[i], width=self.LINE_WIDTH)) for i in range(5)],
                'press': [self.press_plot_l.plot(pen=pg.mkPen(color=self.PRESS_COLORS[i], width=self.LINE_WIDTH)) for i in range(5)],
                'orientation': [self.orientation_plot_l.plot(pen=pg.mkPen(color=self.ORIENTATION_COLORS[i], width=self.LINE_WIDTH)) for i in range(3)],
                'accel': [self.accel_plot_l.plot(pen=pg.mkPen(color=self.ACCEL_COLORS[i], width=self.LINE_WIDTH)) for i in range(3)]
            },
            'r': {
                'flex': [self.flex_plot_r.plot(pen=pg.mkPen(color=self.FLEX_COLORS[i], width=self.LINE_WIDTH)) for i in range(5)],
                'press': [self.press_plot_r.plot(pen=pg.mkPen(color=self.PRESS_COLORS[i], width=self.LINE_WIDTH)) for i in range(5)],
                'orientation': [self.orientation_plot_r.plot(pen=pg.mkPen(color=self.ORIENTATION_COLORS[i], width=self.LINE_WIDTH)) for i in range(3)],
                'accel': [self.accel_plot_r.plot(pen=pg.mkPen(color=self.ACCEL_COLORS[i], width=self.LINE_WIDTH)) for i in range(3)]
            }
        }
        
        # Set up timer for updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(30)  # Update every 30ms for faster refresh
        
        # Start sensor reading
        self.reader.start_readers()
        
    def create_control_panel(self, main_layout):
        """Create control panel with checkboxes for each sensor type"""
        control_group = QGroupBox("Sensor Controls")
        control_group.setFixedHeight(140)  # Increased height for additional IMU controls
        control_layout = QHBoxLayout(control_group)
        
        # Left hand controls
        left_group = QGroupBox("Left Hand")
        left_layout = QVBoxLayout(left_group)
        
        self.checkbox_l_flex = QCheckBox("Flex Sensors")
        self.checkbox_l_flex.setChecked(True)
        self.checkbox_l_flex.stateChanged.connect(self.update_layout)
        left_layout.addWidget(self.checkbox_l_flex)
        
        self.checkbox_l_press = QCheckBox("Pressure Sensors")
        self.checkbox_l_press.setChecked(True)
        self.checkbox_l_press.stateChanged.connect(self.update_layout)
        left_layout.addWidget(self.checkbox_l_press)
        
        self.checkbox_l_orientation = QCheckBox("Orientation (YPR)")
        self.checkbox_l_orientation.setChecked(True)
        self.checkbox_l_orientation.stateChanged.connect(self.update_layout)
        left_layout.addWidget(self.checkbox_l_orientation)
        
        self.checkbox_l_accel = QCheckBox("Acceleration (XYZ)")
        self.checkbox_l_accel.setChecked(True)
        self.checkbox_l_accel.stateChanged.connect(self.update_layout)
        left_layout.addWidget(self.checkbox_l_accel)
        
        # Right hand controls
        right_group = QGroupBox("Right Hand")
        right_layout = QVBoxLayout(right_group)
        
        self.checkbox_r_flex = QCheckBox("Flex Sensors")
        self.checkbox_r_flex.setChecked(True)
        self.checkbox_r_flex.stateChanged.connect(self.update_layout)
        right_layout.addWidget(self.checkbox_r_flex)
        
        self.checkbox_r_press = QCheckBox("Pressure Sensors")
        self.checkbox_r_press.setChecked(True)
        self.checkbox_r_press.stateChanged.connect(self.update_layout)
        right_layout.addWidget(self.checkbox_r_press)
        
        self.checkbox_r_orientation = QCheckBox("Orientation (YPR)")
        self.checkbox_r_orientation.setChecked(True)
        self.checkbox_r_orientation.stateChanged.connect(self.update_layout)
        right_layout.addWidget(self.checkbox_r_orientation)
        
        self.checkbox_r_accel = QCheckBox("Acceleration (XYZ)")
        self.checkbox_r_accel.setChecked(True)
        self.checkbox_r_accel.stateChanged.connect(self.update_layout)
        right_layout.addWidget(self.checkbox_r_accel)
        
        control_layout.addWidget(left_group)
        control_layout.addWidget(right_group)
        main_layout.addWidget(control_group)
        
    def create_side_by_side_layout(self, main_layout):
        """Create side-by-side layout with left and right hand sensors"""
        self.main_splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(self.main_splitter)

        # Left side
        self.left_container = QWidget()
        self.left_layout = QGridLayout(self.left_container)
        self.left_layout.setContentsMargins(5, 5, 5, 5)
        self.main_splitter.addWidget(self.left_container)

        # Right side
        self.right_container = QWidget()
        self.right_layout = QGridLayout(self.right_container)
        self.right_layout.setContentsMargins(5, 5, 5, 5)
        self.main_splitter.addWidget(self.right_container)

        # Create plot widgets
        self.create_plot_widgets()

        # Add plots to both sides
        self.left_layout.addWidget(self.flex_plot_l, 0, 0)
        self.left_layout.addWidget(self.press_plot_l, 1, 0)
        self.left_layout.addWidget(self.orientation_plot_l, 2, 0)
        self.left_layout.addWidget(self.accel_plot_l, 3, 0)
        self.left_layout.setRowStretch(0, 1)
        self.left_layout.setRowStretch(1, 1)
        self.left_layout.setRowStretch(2, 1)
        self.left_layout.setRowStretch(3, 1)

        self.right_layout.addWidget(self.flex_plot_r, 0, 0)
        self.right_layout.addWidget(self.press_plot_r, 1, 0)
        self.right_layout.addWidget(self.orientation_plot_r, 2, 0)
        self.right_layout.addWidget(self.accel_plot_r, 3, 0)
        self.right_layout.setRowStretch(0, 1)
        self.right_layout.setRowStretch(1, 1)
        self.right_layout.setRowStretch(2, 1)
        self.right_layout.setRowStretch(3, 1)

        # Set equal sizes for left and right
        self.main_splitter.setSizes([600, 600])

        # Set initial visibility
        self.update_layout()
        
    def create_plot_widgets(self):
        """Create all plot widgets for flex, pressure, and IMU sensors"""
        # Left hand plots
        self.flex_plot_l = pg.PlotWidget(title='Left Hand Flex')
        self.flex_plot_l.showGrid(x=True, y=True)
        self.flex_plot_l.setLabel('left', 'Value')
        self.flex_plot_l.setLabel('bottom', 'Sample')
        self.flex_plot_l.setYRange(0, 255)
        self.flex_plot_l.addLegend(offset=(10, 10))
        
        self.press_plot_l = pg.PlotWidget(title='Left Hand Pressure')
        self.press_plot_l.showGrid(x=True, y=True)
        self.press_plot_l.setLabel('left', 'Value')
        self.press_plot_l.setLabel('bottom', 'Sample')
        self.press_plot_l.setYRange(0, 255)
        self.press_plot_l.addLegend(offset=(10, 10))
        
        self.orientation_plot_l = pg.PlotWidget(title='Left Hand Orientation (Yaw, Pitch, Roll)')
        self.orientation_plot_l.showGrid(x=True, y=True)
        self.orientation_plot_l.setLabel('left', 'Value')
        self.orientation_plot_l.setLabel('bottom', 'Sample')
        self.orientation_plot_l.setYRange(0, 65535)
        self.orientation_plot_l.addLegend(offset=(10, 10))
        
        self.accel_plot_l = pg.PlotWidget(title='Left Hand Acceleration (Ax, Ay, Az)')
        self.accel_plot_l.showGrid(x=True, y=True)
        self.accel_plot_l.setLabel('left', 'Value')
        self.accel_plot_l.setLabel('bottom', 'Sample')
        self.accel_plot_l.setYRange(0, 65535)
        self.accel_plot_l.addLegend(offset=(10, 10))
        
        # Right hand plots
        self.flex_plot_r = pg.PlotWidget(title='Right Hand Flex')
        self.flex_plot_r.showGrid(x=True, y=True)
        self.flex_plot_r.setLabel('left', 'Value')
        self.flex_plot_r.setLabel('bottom', 'Sample')
        self.flex_plot_r.setYRange(0, 255)
        self.flex_plot_r.addLegend(offset=(10, 10))
        
        self.press_plot_r = pg.PlotWidget(title='Right Hand Pressure')
        self.press_plot_r.showGrid(x=True, y=True)
        self.press_plot_r.setLabel('left', 'Value')
        self.press_plot_r.setLabel('bottom', 'Sample')
        self.press_plot_r.setYRange(0, 255)
        self.press_plot_r.addLegend(offset=(10, 10))
        
        self.orientation_plot_r = pg.PlotWidget(title='Right Hand Orientation (Yaw, Pitch, Roll)')
        self.orientation_plot_r.showGrid(x=True, y=True)
        self.orientation_plot_r.setLabel('left', 'Value')
        self.orientation_plot_r.setLabel('bottom', 'Sample')
        self.orientation_plot_r.setYRange(0, 65535)
        self.orientation_plot_r.addLegend(offset=(10, 10))
        
        self.accel_plot_r = pg.PlotWidget(title='Right Hand Acceleration (Ax, Ay, Az)')
        self.accel_plot_r.showGrid(x=True, y=True)
        self.accel_plot_r.setLabel('left', 'Value')
        self.accel_plot_r.setLabel('bottom', 'Sample')
        self.accel_plot_r.setYRange(0, 65535)
        self.accel_plot_r.addLegend(offset=(10, 10))
        
        # Store plot widgets for easy access
        self.plot_widgets = {
            'l': {
                'flex': self.flex_plot_l,
                'press': self.press_plot_l,
                'orientation': self.orientation_plot_l,
                'accel': self.accel_plot_l
            },
            'r': {
                'flex': self.flex_plot_r,
                'press': self.press_plot_r,
                'orientation': self.orientation_plot_r,
                'accel': self.accel_plot_r
            }
        }
        
        # Channel names for legends
        flex_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        press_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        orientation_names = ['Yaw', 'Pitch', 'Roll']
        accel_names = ['Ax', 'Ay', 'Az']

        # Create curves with user-defined colors, line width, and legend names
        self.curves = {
            'l': {
                'flex': [self.flex_plot_l.plot(pen=pg.mkPen(color=self.FLEX_COLORS[i], width=self.LINE_WIDTH), name=flex_names[i]) for i in range(5)],
                'press': [self.press_plot_l.plot(pen=pg.mkPen(color=self.PRESS_COLORS[i], width=self.LINE_WIDTH), name=press_names[i]) for i in range(5)],
                'orientation': [self.orientation_plot_l.plot(pen=pg.mkPen(color=self.ORIENTATION_COLORS[i], width=self.LINE_WIDTH), name=orientation_names[i]) for i in range(3)],
                'accel': [self.accel_plot_l.plot(pen=pg.mkPen(color=self.ACCEL_COLORS[i], width=self.LINE_WIDTH), name=accel_names[i]) for i in range(3)]
            },
            'r': {
                'flex': [self.flex_plot_r.plot(pen=pg.mkPen(color=self.FLEX_COLORS[i], width=self.LINE_WIDTH), name=flex_names[i]) for i in range(5)],
                'press': [self.press_plot_r.plot(pen=pg.mkPen(color=self.PRESS_COLORS[i], width=self.LINE_WIDTH), name=press_names[i]) for i in range(5)],
                'orientation': [self.orientation_plot_r.plot(pen=pg.mkPen(color=self.ORIENTATION_COLORS[i], width=self.LINE_WIDTH), name=orientation_names[i]) for i in range(3)],
                'accel': [self.accel_plot_r.plot(pen=pg.mkPen(color=self.ACCEL_COLORS[i], width=self.LINE_WIDTH), name=accel_names[i]) for i in range(3)]
            }
        }
        
    def update_layout(self):
        """Show/hide plots based on checkbox states"""
        # Left hand
        left_visible = [
            self.checkbox_l_flex.isChecked(),
            self.checkbox_l_press.isChecked(),
            self.checkbox_l_orientation.isChecked(),
            self.checkbox_l_accel.isChecked()
        ]
        self.flex_plot_l.setVisible(left_visible[0])
        self.press_plot_l.setVisible(left_visible[1])
        self.orientation_plot_l.setVisible(left_visible[2])
        self.accel_plot_l.setVisible(left_visible[3])
        
        # Right hand
        right_visible = [
            self.checkbox_r_flex.isChecked(),
            self.checkbox_r_press.isChecked(),
            self.checkbox_r_orientation.isChecked(),
            self.checkbox_r_accel.isChecked()
        ]
        self.flex_plot_r.setVisible(right_visible[0])
        self.press_plot_r.setVisible(right_visible[1])
        self.orientation_plot_r.setVisible(right_visible[2])
        self.accel_plot_r.setVisible(right_visible[3])

        # For each side, set row stretch so that if only one plot is visible, it fills the space
        for layout, visible in [
            (self.left_layout, left_visible),
            (self.right_layout, right_visible)
        ]:
            num_visible = sum(visible)
            for i, is_vis in enumerate(visible):
                if num_visible == 1:
                    layout.setRowStretch(i, 1 if is_vis else 0)
                else:
                    layout.setRowStretch(i, 1 if is_vis else 0)
        
    def update_plots(self):
        try:
            # Read data from both hands
            for hand in ['l', 'r']:
                # Try to get all available data
                while True:
                    try:
                        data = self.reader.threads[hand]['parser'].getQ().get(block=False)
                        
                        # Process flex data
                        if data and 'flex' in data:
                            flex_data = np.array(data['flex'], dtype=np.float32)
                            if self.buffer_index[hand]['flex'] > 0:
                                flex_data = 0.9 * self.data_buffer[hand]['flex'][self.buffer_index[hand]['flex']-1] + 0.1 * flex_data
                            if self.buffer_index[hand]['flex'] >= self.max_points:
                                self.data_buffer[hand]['flex'] = np.roll(self.data_buffer[hand]['flex'], -1, axis=0)
                                self.buffer_index[hand]['flex'] = self.max_points - 1
                            self.data_buffer[hand]['flex'][self.buffer_index[hand]['flex']] = flex_data
                            self.buffer_index[hand]['flex'] += 1
                        
                        # Process pressure data
                        if data and 'press' in data:
                            press_data = np.array(data['press'], dtype=np.float32)
                            if self.buffer_index[hand]['press'] > 0:
                                press_data = 0.9 * self.data_buffer[hand]['press'][self.buffer_index[hand]['press']-1] + 0.1 * press_data
                            if self.buffer_index[hand]['press'] >= self.max_points:
                                self.data_buffer[hand]['press'] = np.roll(self.data_buffer[hand]['press'], -1, axis=0)
                                self.buffer_index[hand]['press'] = self.max_points - 1
                            self.data_buffer[hand]['press'][self.buffer_index[hand]['press']] = press_data
                            self.buffer_index[hand]['press'] += 1
                        
                        # Process IMU data
                        if data and 'imu' in data:
                            imu_data = np.array(data['imu'], dtype=np.float32)
                            if self.buffer_index[hand]['imu'] > 0:
                                imu_data = 0.9 * self.data_buffer[hand]['imu'][self.buffer_index[hand]['imu']-1] + 0.1 * imu_data
                            if self.buffer_index[hand]['imu'] >= self.max_points:
                                self.data_buffer[hand]['imu'] = np.roll(self.data_buffer[hand]['imu'], -1, axis=0)
                                self.buffer_index[hand]['imu'] = self.max_points - 1
                            self.data_buffer[hand]['imu'][self.buffer_index[hand]['imu']] = imu_data
                            self.buffer_index[hand]['imu'] += 1
                            
                    except queue.Empty:
                        break
                
                # Update plots for all sensor types
                if self.buffer_index[hand]['flex'] > 0:
                    valid_data = self.data_buffer[hand]['flex'][:self.buffer_index[hand]['flex']]
                    y = valid_data[::self.POINTS_SKIP]
                    x = np.arange(len(y))
                    for i in range(5):
                        self.curves[hand]['flex'][i].setData(x, y[:, i])
                
                if self.buffer_index[hand]['press'] > 0:
                    valid_data = self.data_buffer[hand]['press'][:self.buffer_index[hand]['press']]
                    y = valid_data[::self.POINTS_SKIP]
                    x = np.arange(len(y))
                    for i in range(5):
                        self.curves[hand]['press'][i].setData(x, y[:, i])
                
                if self.buffer_index[hand]['imu'] > 0:
                    valid_data = self.data_buffer[hand]['imu'][:self.buffer_index[hand]['imu']]
                    y = valid_data[::self.POINTS_SKIP]
                    x = np.arange(len(y))
                    # Update orientation plots (first 3 channels)
                    for i in range(3):
                        self.curves[hand]['orientation'][i].setData(x, y[:, i])
                    # Update acceleration plots (last 3 channels)
                    for i in range(3):
                        self.curves[hand]['accel'][i].setData(x, y[:, i+3])
                
        except Exception as e:
            print(f"Error updating plots: {e}")
    
    def closeEvent(self, event):
        self.reader.stop_readers()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SensorPlotter()
    window.show()
    sys.exit(app.exec_())