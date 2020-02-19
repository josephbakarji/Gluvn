'''
Created on May 20, 2016

@author: josephbakarji
'''

import numpy as np
from notemidi import signswitch2note, TriggerChordTest
from threading import Thread
  
  
class WeighTrig(Thread):
    def __init__(self, sensorqueue, thresh, notearr):
        Thread.__init__(self)
        self.sensorq = sensorqueue
        self.thresh = thresh
        self.notearr = notearr
        self.daemon = True
        

    def run(self):
        
        # Wait for readings.
        while (len(self.sensorq) == 0):
            pass
        
        trigonprev = (np.asarray(self.sensorq[0]) - self.thresh) > 0
        trigoffprev = (np.asarray(self.sensorq[0]) - self.thresh) > 0

        dshmidt = 5
        while True:
            if(len(self.sensorq) != 0):
                sensarr = np.asarray(self.sensorq[-1]) # pop seems to read backwards; fix if sends are faster than reads .
                #trig = (sensarr - self.thresh) > 0 # compare sensor
                #ss = trig.astype(int) - trigprev.astype(int) # -1 to turn off, +1 to turn on
                
                # Extending to shmidt-like trigger (should be improved)
                trigon = (sensarr - self.thresh) > dshmidt
                trigoff = (sensarr - self.thresh) < -1 * dshmidt
                turnon =  trigon.astype(int) > trigonprev.astype(int) # goes from 0 to 1 (works without astype too)
                turnoff = trigoff.astype(int) > trigoffprev.astype(int) # 0 to 1
                ss = turnon.astype(int) - turnoff.astype(int)
                
                trigonprev = trigon
                trigoffprev = trigoff                
            
                if(not(all(i == 0 for i in ss))):
                    signswitch2note(ss, sensarr, self.notearr)
                    
                    
                    
                    
# Class plays chords by changing pitch and triggering either pressure sensors or flex sensors
class playchords(Thread):
    def __init__(self, sensorqueue, sensthresh, imuq, phist, pthresh):
        Thread.__init__(self)
        self.sensorq = sensorqueue
        self.sensthresh = sensthresh
        self.imq = imuq
        self.phist = phist   # 127 == 0 degrees
        self.pthresh = pthresh
        self.daemon = True
        

    def run(self):

        trigsens = [0, 0, 0, 0, 0]
        trigonprev = True
        trigoffprev = True        
        while True:
            if(len(self.imq) != 0): # REMOVE WHEN WIRE SOLDERED
                pitch = self.imq[-1][1]
                trigon = (pitch - self.pthresh) <  - self.phist # check if state in region with off or on trigger
                trigoff = (pitch - self.pthresh) > self.phist
                turnon =  int(trigon) > int(trigonprev) # goes from 0 to 1 (works without astype too)
                turnoff = int(trigoff) > int(trigoffprev) # 0 to 1
                
                trigonprev = trigon
                trigoffprev = trigoff
                #print(pitch)
                #print(self.sensorq[-1])
                if(turnon):
                    print(self.sensorq[-1])
                    sensarr = np.asarray(self.sensorq[-1])
                    trigsens = (sensarr - self.sensthresh) > 0
                    TriggerChordTest(trigsens, pitch, 'on')
                
                if(turnoff):
                    TriggerChordTest(trigsens, pitch, 'off')
                    
            
