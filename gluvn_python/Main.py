import sensor_read
import senstonote
import queue
from collections import deque
import numpy as np
import time


# Toggle apps
PressureNotes = 1
FlexNotes = 0
FlexPitchChords = 0
# Notes
n1 = ['C3', 'D3', 'E3', 'G3', 'A3']
n2 = ['G3', 'A3', 'F4', 'D4', 'C4']
# Thresholds
pressTrigThresh = 20
flexTrigThresh = 150
flexChordthresh = 110 


# simulate or actual
sim = 1
simfile = 'rnd_glove_ypr_press.csv'
  


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
    simThread = sensor_read.sim_glove('data/sim/' + simfile, imuq, flexq, pressq)
    simThread.start()

# Wait for readings.
while (len(flexq) == 0 or len(imuq) == 0 or len(pressq) == 0):
    pass
###############################



print('Apps Threads Starting ... ')
if(PressureNotes):
    PressTrigP = senstonote.WeighTrig(pressq, pressTrigThresh * np.ones(len(pressq[0])), n1)
    PressTrigP.start()

if(FlexNotes):      
    PressTrigF = senstonote.WeighTrig(flexq, flexTrigThresh * np.ones(len(flexq[0])), n2)
    PressTrigF.start()
   
if(FlexPitchChords):
    FlexChord = senstonote.playchords(flexq, flexChordthresh * np.ones(len(flexq[0])) , imuq, 10, 127)
    FlexChord.start()


while True:
    pass
    #print(imuq[-1][1])
