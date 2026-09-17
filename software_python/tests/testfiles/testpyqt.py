import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
import numpy as np

# Create the application instance
app = QApplication([])

# Create a window
win = pg.GraphicsLayoutWidget()
win.setWindowTitle('Real-time pyqtgraph Plotting')

# Add a plot to the window
plot = win.addPlot(title="Real-time Plot")
curve = plot.plot()

data = np.random.normal(size=(10, 1000))
ptr = 0

def update():
    global curve, data, ptr
    curve.setData(data[ptr % 10])
    ptr += 1

timer = QTimer()
timer.timeout.connect(update)
timer.start(50)

# Show the window
win.show()

# Start the Qt event loop
app.exec_()
