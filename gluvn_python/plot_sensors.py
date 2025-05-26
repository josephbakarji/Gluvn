import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QGroupBox, QSplitter, QGridLayout
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg
from port_read import Reader
import time
import queue

class SensorPlotter(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Sensor Data Plotter')
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize sensor reading with both flex and pressure sensors
        self.sensor_config = {'l': {'flex': True, 'press': True, 'imu': False},
                            'r': {'flex': True, 'press': True, 'imu': False}}
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
        self.max_points = 1000
        
        # Initialize data storage with numpy arrays for better performance
        self.data_buffer = {
            'l': {'flex': np.zeros((self.max_points, 5), dtype=np.float32),
                  'press': np.zeros((self.max_points, 5), dtype=np.float32)},
            'r': {'flex': np.zeros((self.max_points, 5), dtype=np.float32),
                  'press': np.zeros((self.max_points, 5), dtype=np.float32)}
        }
        self.buffer_index = {
            'l': {'flex': 0, 'press': 0},
            'r': {'flex': 0, 'press': 0}
        }
        
        # Create curves with different colors
        colors = [(255,0,0), (0,255,0), (0,0,255), (255,255,0), (0,255,255)]
        self.curves = {
            'l': {
                'flex': [self.flex_plot_l.plot(pen=pg.mkPen(color=colors[i], width=2)) for i in range(5)],
                'press': [self.press_plot_l.plot(pen=pg.mkPen(color=colors[i], width=2)) for i in range(5)]
            },
            'r': {
                'flex': [self.flex_plot_r.plot(pen=pg.mkPen(color=colors[i], width=2)) for i in range(5)],
                'press': [self.press_plot_r.plot(pen=pg.mkPen(color=colors[i], width=2)) for i in range(5)]
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
        control_group.setFixedHeight(100)  # Fixed height to prevent expansion
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

        # Add both plots (flex and pressure) to both sides, always
        self.left_layout.addWidget(self.flex_plot_l, 0, 0)
        self.left_layout.addWidget(self.press_plot_l, 1, 0)
        self.left_layout.setRowStretch(0, 1)
        self.left_layout.setRowStretch(1, 1)

        self.right_layout.addWidget(self.flex_plot_r, 0, 0)
        self.right_layout.addWidget(self.press_plot_r, 1, 0)
        self.right_layout.setRowStretch(0, 1)
        self.right_layout.setRowStretch(1, 1)

        # Set equal sizes for left and right
        self.main_splitter.setSizes([600, 600])

        # Set initial visibility
        self.update_layout()
        
    def create_plot_widgets(self):
        """Create all plot widgets for flex and pressure sensors"""
        # Left hand plots
        self.flex_plot_l = pg.PlotWidget(title='Left Hand Flex')
        self.flex_plot_l.showGrid(x=True, y=True)
        self.flex_plot_l.setLabel('left', 'Value')
        self.flex_plot_l.setLabel('bottom', 'Sample')
        self.flex_plot_l.setYRange(0, 255)
        
        self.press_plot_l = pg.PlotWidget(title='Left Hand Pressure')
        self.press_plot_l.showGrid(x=True, y=True)
        self.press_plot_l.setLabel('left', 'Value')
        self.press_plot_l.setLabel('bottom', 'Sample')
        self.press_plot_l.setYRange(0, 255)
        
        # Right hand plots
        self.flex_plot_r = pg.PlotWidget(title='Right Hand Flex')
        self.flex_plot_r.showGrid(x=True, y=True)
        self.flex_plot_r.setLabel('left', 'Value')
        self.flex_plot_r.setLabel('bottom', 'Sample')
        self.flex_plot_r.setYRange(0, 255)
        
        self.press_plot_r = pg.PlotWidget(title='Right Hand Pressure')
        self.press_plot_r.showGrid(x=True, y=True)
        self.press_plot_r.setLabel('left', 'Value')
        self.press_plot_r.setLabel('bottom', 'Sample')
        self.press_plot_r.setYRange(0, 255)
        
        # Store plot widgets for easy access
        self.plot_widgets = {
            'l': {'flex': self.flex_plot_l, 'press': self.press_plot_l},
            'r': {'flex': self.flex_plot_r, 'press': self.press_plot_r}
        }
        
    def update_layout(self):
        """Show/hide plots based on checkbox states, always keeping both rows in the layout. Expand to fill window if only one plot is visible per side."""
        # Left hand
        left_visible = [self.checkbox_l_flex.isChecked(), self.checkbox_l_press.isChecked()]
        self.flex_plot_l.setVisible(left_visible[0])
        self.press_plot_l.setVisible(left_visible[1])
        # Right hand
        right_visible = [self.checkbox_r_flex.isChecked(), self.checkbox_r_press.isChecked()]
        self.flex_plot_r.setVisible(right_visible[0])
        self.press_plot_r.setVisible(right_visible[1])

        # For each side, set row stretch so that if only one plot is visible, it fills the space
        # If both are visible, split space equally
        # (This logic will generalize to more sensor types in the future)
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
                            
                            # Apply stronger moving average smoothing
                            if self.buffer_index[hand]['flex'] > 0:
                                # Blend with previous value (90% previous, 10% new) for more stability
                                flex_data = 0.9 * self.data_buffer[hand]['flex'][self.buffer_index[hand]['flex']-1] + 0.1 * flex_data
                            
                            # If buffer is full, use numpy's roll to shift data
                            if self.buffer_index[hand]['flex'] >= self.max_points:
                                # Roll the entire buffer left by one position
                                self.data_buffer[hand]['flex'] = np.roll(self.data_buffer[hand]['flex'], -1, axis=0)
                                self.buffer_index[hand]['flex'] = self.max_points - 1
                            
                            # Store in buffer
                            self.data_buffer[hand]['flex'][self.buffer_index[hand]['flex']] = flex_data
                            self.buffer_index[hand]['flex'] += 1
                        
                        # Process pressure data
                        if data and 'press' in data:
                            press_data = np.array(data['press'], dtype=np.float32)
                            
                            # Apply stronger moving average smoothing
                            if self.buffer_index[hand]['press'] > 0:
                                # Blend with previous value (90% previous, 10% new) for more stability
                                press_data = 0.9 * self.data_buffer[hand]['press'][self.buffer_index[hand]['press']-1] + 0.1 * press_data
                            
                            # If buffer is full, use numpy's roll to shift data
                            if self.buffer_index[hand]['press'] >= self.max_points:
                                # Roll the entire buffer left by one position
                                self.data_buffer[hand]['press'] = np.roll(self.data_buffer[hand]['press'], -1, axis=0)
                                self.buffer_index[hand]['press'] = self.max_points - 1
                            
                            # Store in buffer
                            self.data_buffer[hand]['press'][self.buffer_index[hand]['press']] = press_data
                            self.buffer_index[hand]['press'] += 1
                            
                    except queue.Empty:
                        break
                
                # Update plots for both sensor types
                for sensor_type in ['flex', 'press']:
                    if self.buffer_index[hand][sensor_type] > 0:
                        # Get every 5th point to reduce data density
                        valid_data = self.data_buffer[hand][sensor_type][:self.buffer_index[hand][sensor_type]]
                        y = valid_data[::5]
                        x = np.arange(len(y))
                        
                        # Update all curves at once
                        for i in range(5):
                            self.curves[hand][sensor_type][i].setData(x, y[:, i])
                
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