# -*- coding: utf-8 -*-


import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import time
import sys
from threading import Thread




# def plotsens(sensdata):
#     win = pg.GraphicsWindow()
#     win.setWindowTitle('pyqtgraph example: PanningPlot')
#     plt = win.addPlot()
#     curve = plt.plot()
#     t0 = time.time()
#     
#     def update(sensdata):
#         global pdata, tarr, curve
#         pdata.append(sensdata)
#         tarr.append(t0 - time.time())
#         if len(pdata) > 100:
#             tarr.pop(0)
#             pdata.pop(0)
#         curve.setData(x = tarr, y= pdata)
#     
#     timer = QtCore.QTimer()
#     timer.timeout.connect(update)
#     timer.start(50)
#     
#     QtGui.QApplication.instance().exec_()



class PlotSens(QtCore.QThread):
    def __init__(self, sensdata):
        QtCore.QThread.__init__(self)
        self.sensdata = sensdata
        self.pdata = []
        self.app = QtGui.QApplication([])
        
        win = pg.GraphicsWindow()
        win.setWindowTitle('Sensor Reading')
        plt = win.addPlot()
         
        self.curve = plt.plot()
        self.t0 = time.time()
        self.tarr = []
 
         
    def update(self):
        self.pdata.append(self.sensdata)
        self.tarr.append(self.t0 - time.time())
        if len(self.pdata) > 100:
            self.tarr.pop(0)
            self.pdata.pop(0)
        self.curve.setData(self.tarr, self.pdata)
        self.app.processEvents()
     
 
     
    def run(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)
    ## Start Qt event loop unless running in interactive mode or using pyside.
        #if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        #QtGui.QApplication.instance().exec_()
