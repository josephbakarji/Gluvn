import sensor_read
import senstonote
import queue
from collections import deque
import numpy as np
import time

def main(simflag, pressflag, flexflag, chordflag, pnotes, fnotes):
    # Variables


    sensq = queue.Queue()
    dataq = queue.Queue()
    pressq = deque(maxlen = 50)
    flexq = deque(maxlen = 50)
    imuq = deque(maxlen = 50)



    # Start reading threads
    if(not simflag):
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

    if(pressflag):
        PressTrigP = senstonote.WeighTrig(pressq, 20 * np.ones(len(pressq[0])), pnotes)
        PressTrigP.start()

    if(flexflag):      
        PressTrigF = senstonote.WeighTrig(flexq, 150 * np.ones(len(flexq[0])), fnotes)
        PressTrigF.start()

    if(chordflag):
        flexthresh = 110 * np.ones(len(flexq[0]))        
        FlexChord = senstonote.playchords(flexq, flexthresh, imuq, 10, 127)
        FlexChord.start()


        while True:
            pass

