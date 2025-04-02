import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
from port_read import Reader
import time
import queue

class SensorPlotter(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Sensor Data Plotter')
        self.setGeometry(100, 100, 800, 600)
        
        # Initialize sensor reading
        self.sensor_config = {'l': {'flex': True, 'press': False, 'imu': False},
                            'r': {'flex': True, 'press': False, 'imu': False}}
        self.reader = Reader(sensor_config=self.sensor_config, save=False)
        
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # Create plot widgets
        self.flex_plot_l = pg.PlotWidget(title='Left Hand Flex')
        self.flex_plot_l.showGrid(x=True, y=True)
        self.flex_plot_l.setLabel('left', 'Value')
        self.flex_plot_l.setLabel('bottom', 'Sample')
        self.flex_plot_l.setYRange(0, 255)
        
        self.flex_plot_r = pg.PlotWidget(title='Right Hand Flex')
        self.flex_plot_r.showGrid(x=True, y=True)
        self.flex_plot_r.setLabel('left', 'Value')
        self.flex_plot_r.setLabel('bottom', 'Sample')
        self.flex_plot_r.setYRange(0, 255)
        
        layout.addWidget(self.flex_plot_l)
        layout.addWidget(self.flex_plot_r)
        
        # Define buffer size
        self.max_points = 1000
        
        # Initialize data storage with numpy arrays for better performance
        self.data_buffer = {
            'l': np.zeros((self.max_points, 5), dtype=np.float32),
            'r': np.zeros((self.max_points, 5), dtype=np.float32)
        }
        self.buffer_index = {'l': 0, 'r': 0}
        
        # Create curves with different colors
        colors = [(255,0,0), (0,255,0), (0,0,255), (255,255,0), (0,255,255)]
        self.curves = {
            'l': [self.flex_plot_l.plot(pen=pg.mkPen(color=colors[i], width=2)) for i in range(5)],
            'r': [self.flex_plot_r.plot(pen=pg.mkPen(color=colors[i], width=2)) for i in range(5)]
        }
        
        # Set up timer for updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(30)  # Update every 30ms for faster refresh
        
        # Start sensor reading
        self.reader.start_readers()
        
    def update_plots(self):
        try:
            # Read data from both hands
            for hand in ['l', 'r']:
                # Try to get all available data
                while True:
                    try:
                        data = self.reader.threads[hand]['parser'].getQ().get(block=False)
                        if data and 'flex' in data:
                            # Convert to float32 for better precision
                            flex_data = np.array(data['flex'], dtype=np.float32)
                            
                            # Apply stronger moving average smoothing
                            if self.buffer_index[hand] > 0:
                                # Blend with previous value (90% previous, 10% new) for more stability
                                flex_data = 0.9 * self.data_buffer[hand][self.buffer_index[hand]-1] + 0.1 * flex_data
                            
                            # If buffer is full, use numpy's roll to shift data
                            if self.buffer_index[hand] >= self.max_points:
                                # Roll the entire buffer left by one position
                                self.data_buffer[hand] = np.roll(self.data_buffer[hand], -1, axis=0)
                                self.buffer_index[hand] = self.max_points - 1
                            
                            # Store in buffer
                            self.data_buffer[hand][self.buffer_index[hand]] = flex_data
                            self.buffer_index[hand] += 1
                            
                    except queue.Empty:
                        break
                
                # Get the valid portion of the buffer
                if self.buffer_index[hand] > 0:
                    # Get every 5th point to reduce data density
                    valid_data = self.data_buffer[hand][:self.buffer_index[hand]]
                    y = valid_data[::5]
                    x = np.arange(len(y))
                    
                    # Update all curves at once
                    for i in range(5):
                        self.curves[hand][i].setData(x, y[:, i])
                
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