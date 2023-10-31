

from port_read import Reader
from threading import Thread
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
import queue
from collections import deque


ZERO_GYRIN = 32767.0
G = ZERO_GYRIN / 2.0 - 680.0

class PrintSensAnalyze(Thread):
    def __init__(self, sensorq, data_queue, info=''):
        Thread.__init__(self)
        self.sensorq = sensorq
        self.data_queue = data_queue
        self.info = info
        self.daemon = True
        self.averaging_queue = deque(maxlen=30)

    def run(self):
        accel_prev = 0
        alpha = .9 
        while True:
            sensvalue = self.sensorq.get(block=True)

            accel_mag = max(np.linalg.norm(np.array(sensvalue['imu'][-3:]) - ZERO_GYRIN) - G, 0)
            self.averaging_queue.append(accel_mag)
            

            ## Compute average
            # self.data_queue.put(np.mean(self.averaging_queue))
            # accel_prev = accel


def update_plot():
    if not data_queue.empty():
        new_data = data_queue.get()
        data.append(new_data)
        if len(data) > 1000:  # Limit the size of displayed data
            data.pop(0)
        curve.setData(data)


# Queue to communicate between sensor reading and plotting threads
data_queue = queue.Queue()
data = []

# Start sensor reading thread
save = False
sensor_config = {'r': {'flex': False, 'press': False, 'imu': True}}
reader = Reader(sensor_config=sensor_config, save=save, save_file='test00')
reader.start_readers()
printer = PrintSensAnalyze(reader.threads['r']['parser'].getQ(), data_queue)
printer.start()

# Set up the PyQt application and window
app = QApplication([])
win = pg.GraphicsLayoutWidget()
win.setWindowTitle('Real-time pyqtgraph Plotting')
plot = win.addPlot(title="Real-time Plot")
curve = plot.plot()
timer = QTimer()
timer.timeout.connect(update_plot)
timer.start(1)
win.show()

# Start the Qt event loop
app.exec_()

# After closing the PyQt window, stop the sensor reading
print('Shutting down...')
reader.stop_readers()