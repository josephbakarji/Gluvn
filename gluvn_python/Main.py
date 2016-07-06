import sensor_read
import senstonote
import queue
from collections import deque
import numpy as np
#from pyqtgraph.Qt import QtCore, QtGui
#import pyqtgraph as pg
import time


# Variables
wlport = '/dev/tty.usbserial-A8004ZZe'
port = '/dev/tty.usbmodem1411'
baud = 57600

# Toggle apps
PressureNotes = 0
FlexNotes = 0
FlexPitchChords = 1

# simulate or actual
sim = 0
  

# initialize queues
sensq = queue.Queue()
dataq = queue.Queue()
pressq = deque(maxlen = 50)
flexq = deque(maxlen = 50)
imuq = deque(maxlen = 50)


  
# Start reading threads
if(sim == 0):
    ParseThread = sensor_read.parse_serial(sensq, dataq, imuq, flexq, pressq)
    SensorThread = sensor_read.read_port(port, baud, sensq)
    ParseThread.start()
    SensorThread.start()
else:
    simThread = sensor_read.sim_glove('data/sim/rnd_glove_ypr_press.csv', imuq, flexq, pressq)
    simThread.start()

# Wait for readings.
while (len(flexq) == 0 or len(imuq) == 0 or len(pressq) == 0):
    pass
###############################



print('Apps Threads Starting ... ')
if(PressureNotes):
    PressTrigP = senstonote.WeighTrig(pressq, 20 * np.ones(len(pressq[0])), ['C3', 'D3', 'E3', 'G3', 'A3'])
    PressTrigP.start()

if(FlexNotes):      
    PressTrigF = senstonote.WeighTrig(flexq, 150 * np.ones(len(flexq[0])), ['G3', 'A3', 'F4', 'D4', 'C4'])
    PressTrigF.start()
   
   
if(FlexPitchChords):
    flexthresh = 110 * np.ones(len(flexq[0]))        
    FlexChord = senstonote.playchords(flexq, flexthresh, imuq, 10, 127)
    FlexChord.start()


while True:
    pass
    #print(imuq[-1][1])
