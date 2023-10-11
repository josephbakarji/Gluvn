'''
Created on May 20, 2016

@author: josephbakarji
'''

import numpy as np
from notemidi import TrigNote, TrigNote_midinum, signswitch2note, TriggerChordTest, make_C2midi
from __init__ import settingsDir
from learning import Learn
from threading import Thread
from collections import deque
from mapper import NoteMapper
import queue

# Constants

ZERO_GYRIN = 32767.0
MAX_BEND = 2000
HIGH_JUMP_THRESHOLD = 100

#######################################################################################################################################
#######################################################################################################################################

# NOT TESTED
class Trigger(Thread):
    def __init__(self, hand, sensorq, collectq, thresh=180, dshmidt=5):
        Thread.__init__(self)
        self.hand = hand
        self.sensorq = sensorq
        self.collectq = collectq
        self.thresh = thresh
        self.dshmidt = dshmidt
        self.trigon_prev = False
        self.trigoff_prev = False
        self.turn_state = np.zeros(5, dtype=bool)

    def shmidt_trig(self, sensvalue):
        " Basic trigger function that takes sensor readings and returns trigger on/off and turn on/off "
        sensarr = np.asarray(sensvalue) # Transforming to numpy array everytime might be inefficient
        sdiff = sensarr - self.thresh        # subtract threshold from readings
        trigon = sdiff - self.dshmidt > 0
        trigoff = sdiff + self.dshmidt < 0 
        turnon = ( trigon & np.logical_not( self.trigon_prev ) ) & np.logical_not(self.turn_state)
        turnoff = ( trigoff & np.logical_not( self.trigoff_prev ) ) & self.turn_state 
        nswitch = turnon.astype(int) - turnoff.astype(int) # Turn on if 1, Turn off if -1, No action if 0. - Also works 1*turnon - 1*turnoff
        self.trigon_prev = trigon
        self.trigoff_prev = trigoff
        return nswitch

    def run(self):
        while True:
            sensvalue = self.sensorq.get(block=True)
            nswitch = self.shmidt_trig(sensvalue)

            if nswitch.any(): 
                self.turn_state = nswitch + self.turn_state 
                state = [nswitch, self.hand]
                self.collectq.put(state)
            

#######################################################################################################################################
#######################################################################################################################################
# def trigger_usepast(sensorq, trigon_prev, trigoff_prev, turn_state):

##### NOT TESTED 
class SimpleTrigger(Thread):
    def __init__(self, sensorqR=None, sensorqL=None, thresh=180, dshmidt=5):
        Thread.__init__(self)
        self.collectq = queue.Queue(maxsize=20)

        # Determine whether both are included or not
        self.Trigger_R = Trigger('R', sensorqR, self.collectq, thresh, dshmidt)
        self.Trigger_L = Trigger('L', sensorqL, self.collectq, thresh, dshmidt)

        # Add more mapping options
        self.mapper = NoteMapper(scale='major')
        self.daemon = True

    def run(self):
        self.Trigger_R.start()
        self.Trigger_L.start()
        notearrL = self.mapper.basicMap(first_note='C3')
        notearrR = self.mapper.basicMap(first_note='A3')

        while True: 
            nswitch, hand = self.collectq.get(block=True)
            if hand == 'R':
                signswitch2note(nswitch, notearrR)
            else:
                signswitch2note(nswitch, notearrL)


##############################################################################
##############################################################################

######### NOT DONE
# Collect and combine glove readings
class FlexPressInstrument(SimpleTrigger, Thread):
    def __init__(self, basenote='C', scale='minor'):
        Thread.__init__(self)
        FlexPressInstrument.__init__(self, sensorqR=None, sensorqL=None, thresh=180, dshmidt=5)
        self.basenote = basenote
        self.scale = scale
        self.daemon = True

    def run(self):
        
        fnum = 3
        #sensarr = np.array([0, 0, 0, 0, 0]) # Arbitrary just for test
        NotesOn = ['' for i in range(fnum)]
        [WArr, NArr] = GenerateNoteMap(midnote, scale, mode)
        notearr = WindowMap(np.zeros(5), WArr, NArr)

        while True:
            state = self.collectq.get(block=True)
            ### state = [turn_state, nswitch, self.hand]

            #[TrigNote(notearr[i], vel) for i in range(state[2]) if ]
            ## Normal 5 note usage
            # if(state[2] == 'R'): 
            #     for i in range(fnum):
            #         if(state[1][i] == 1):
            #             TrigNote(notearr[i], 80) # Add function to calculate velocity
            #             NotesOn[i] = notearr[i]
            #         elif(state[1][i] == -1):
            #             TrigNote(NotesOn[i], 0)

            # 3 press usage
            if(state[2] == 'R'): 
                for i, j in enumerate(range(1, 4)):
                    if(state[1][j] == 1):
                        print(i, j)
                        TrigNote(notearr[i], 80) # Add function to calculate velocity
                        NotesOn[i] = notearr[i]
                        print(notearr)
                        #print('note on: ', NotesOn)
                    elif(state[1][j] == -1):
                        TrigNote(NotesOn[i], 0)
                        #print('note off: ', NotesOn)

            else: 
                notearr = WindowMap(state[0], WArr, NArr)
                print(state[0])




## Trigger class for one sensor ##

# # Thread that takes sensor queue as input and sends individual midi triggers as output
# class WeighTrig(Thread):
#     def __init__(self, sensorq, thresh, notearr):
#         Thread.__init__(self)
#         self.sensorq = sensorq
#         self.thresh = thresh
#         self.notearr = notearr
#         self.daemon = True

#     def run(self):

#         trigon_prev = False
#         trigoff_prev = False
#         turn_state = np.zeros(5, dtype=bool)
#         dshmidt = 5     # use as input

#         while True: 
#             sensvalue = self.sensorq.get(block=True)
#             [trigon_prev, trigoff_prev, nswitch] = triggerfun(sensvalue, self.thresh, dshmidt, trigon_prev, trigoff_prev, turn_state)
            
#             if( not(all(i == 0 for i in nswitch)) ): # use if(nswitch.any())
#                 turn_state = nswitch + turn_state
#                 signswitch2note(nswitch, sensvalue, self.notearr)

##############################################################################
##############################################################################
## 2 Hand triggering combined ##



# Collect and combine glove readings
class CombHandsSendMidi(Thread):
    def __init__(self, collectq, basenote, key):
        Thread.__init__(self)
        self.collectq = collectq
        self.basenote = basenote
        self.key = key
        self.daemon = True

    def run(self):
        
        midnote = 'C3'
        mode = 'standard'
        scale = 'major'

        fnum = 3
        #sensarr = np.array([0, 0, 0, 0, 0]) # Arbitrary just for test
        NotesOn = ['' for i in range(fnum)]
        [WArr, NArr] = GenerateNoteMap(midnote, scale, mode)
        notearr = WindowMap(np.zeros(5), WArr, NArr)

        while True:
            state = self.collectq.get(block=True)
            ### state = [turn_state, nswitch, self.hand]

            #[TrigNote(notearr[i], vel) for i in range(state[2]) if ]
            ## Normal 5 note usage
            # if(state[2] == 'R'): 
            #     for i in range(fnum):
            #         if(state[1][i] == 1):
            #             TrigNote(notearr[i], 80) # Add function to calculate velocity
            #             NotesOn[i] = notearr[i]
            #         elif(state[1][i] == -1):
            #             TrigNote(NotesOn[i], 0)

            # 3 press usage
            if(state[2] == 'R'): 
                for i, j in enumerate(range(1, 4)):
                    if(state[1][j] == 1):
                        print(i, j)
                        TrigNote(notearr[i], 80) # Add function to calculate velocity
                        NotesOn[i] = notearr[i]
                        print(notearr)
                        #print('note on: ', NotesOn)
                    elif(state[1][j] == -1):
                        TrigNote(NotesOn[i], 0)
                        #print('note off: ', NotesOn)

            else: 
                notearr = WindowMap(state[0], WArr, NArr)
                print(state[0])





################################################################################
################################################################################
# Play chords with flex (fix)

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
                    

