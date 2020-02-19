from sensor_read import read_port, parse_serial
import sensor_read
import senstonote
import queue
from collections import deque
import numpy as np
import time
from __init__ import portR, portL, baud
import sys
from struct import *
import matplotlib.pyplot as plt 
import matplotlib.animation as animation

### INITIALIZE

global use_flex, use_press, use_imu, use_fulldata

use_flex = False
use_press = False
use_imu = False
use_fulldata = False

use_vecL = [use_press, use_flex, use_imu, use_fulldata]
use_vecR = [use_press, use_flex, use_imu, use_fulldata]
# Select which app to be used 

PressureNotes = 0
PressureNotes2 = 1
FlexNotes = 0
FlexPitchChords = 0

if(PressureNotes):
    use_vecR[0] = True
elif(PressureNotes2):
    use_vecR[0] = True
    use_vecL[1] = True
elif(FlexNotes):
    use_vecR[1] = True
elif(FlexPitchChords):
    use_vecR[1] = True
    use_vecR[2] = True

# Choose the notes played by each finger 
notesL = ['C3', 'D3', 'E3', 'F3', 'G3']
notesR = ['A3', 'B4', 'C4', 'D4', 'E4']

# Set thresholds for triggers
pressTrigThresh = 20
flexTrigThresh = 120
flexChordthresh = 110 


# sim=1 for emulator (from file) and sim=0 for actual glove
sim = 0
simfile = 'test0'
  
sensqR = queue.Queue(maxsize = 48) # Not used in Main.py! put it locally in sensor_read
dataqR = queue.Queue(maxsize = 48)
pressqR = queue.Queue(maxsize = 10)
flexqR = queue.Queue(maxsize = 10)
imuqR = queue.Queue(maxsize = 14)

sensqL = queue.Queue(maxsize = 48)
dataqL = queue.Queue(maxsize = 48)
pressqL = queue.Queue(maxsize = 10)
flexqL = queue.Queue(maxsize = 10)
imuqL = queue.Queue(maxsize = 14)

collectq = queue.Queue(maxsize = 100)

### RUN THREADS


# Start reading threads
if(sim == 0):
    # Serial Read Threads
    SensorThreadR = read_port(portR, baud, sensqR)
    SensorThreadL = read_port(portL, baud, sensqL)

    # Parsing Threads
    ParseThreadL = parse_serial(dataqL, imuqL, flexqL, pressqL, sensqL, use_vecL)
    ParseThreadR = parse_serial(dataqR, imuqR, flexqR, pressqR, sensqR, use_vecR)
    
    # RUN Threads
    SensorThreadR.start()
    ParseThreadR.start()
    SensorThreadL.start()
    ParseThreadL.start()

else:
    simThreadR = sensor_read.sim_glove(simfile, imuqR, flexqR, pressqR, 'R')
    simThreadL = sensor_read.sim_glove(simfile, imuqL, flexqL, pressqL, 'L')
    simThreadR.start()
    simThreadL.start()


### Run Apps

print('Apps Threads Starting ... ')
if(PressureNotes):
    print('Pressure Notes App')
    #use_vec[0] = True
    PressTrigPR = senstonote.WeighTrig(pressqR, pressTrigThresh, notesR) # ???
    PressTrigPL = senstonote.WeighTrig(pressqL, pressTrigThresh, notesL)

    PressTrigPR.start()
    PressTrigPL.start()

elif(PressureNotes2):
    print('Pressure Notes App 2')
    #use_vec[0] = True

    basenote = 'C2'
    key = 'Major'
    PressTrigPR = senstonote.WeighTrig2h(pressqR, collectq, 'R', pressTrigThresh, 5) # ???
    #PressTrigPL = senstonote.WeighTrig2h(pressqL, collectq, 'L', 40, 10)  
    PressTrigPL = senstonote.WeighTrig2h(flexqL, collectq, 'L', 100, 20)

    CollectSend = senstonote.CombHandsSendMidi(collectq, basenote, key)

    PressTrigPR.start()
    PressTrigPL.start()
    CollectSend.start()

elif(FlexNotes): 
    print('Flex Notes App')

    #use_vec[1] = True
    PressTrigF = senstonote.WeighTrig(flexqR, flexTrigThresh , notesR)
    PressTrigF.start()
   
elif(FlexPitchChords):
    print('Flex-IMU Chords App')    
    #use_vec[1] = True
    #use_vec[2] = True
    FlexChord = senstonote.playchords(flexqR, flexChordthresh, imuqR, 10, 127)
    FlexChord.start()

else:
    print('Choose an application and repeat!')
    sys.exit()



while True:
    key = input('press q to exit \n')
    if(key == 'q'):
        print('Rest your hands')
        sys.exit()
