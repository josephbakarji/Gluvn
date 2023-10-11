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

# Constants

ZERO_GYRIN = 32767.0
MAX_BEND = 2000
HIGH_JUMP_THRESHOLD = 100

#######################################################################################################################################
#######################################################################################################################################

# def trigger_usepast(sensorq, trigon_prev, trigoff_prev, turn_state):
    

def triggerfun(sensvalue, thresh, dshmidt, trigon_prev, trigoff_prev, turn_state):
    sensarr = np.asarray(sensvalue) # Transforming to numpy array everytime might be inefficient
    sdiff = sensarr - thresh        # subtract threshold from readings
    trigon = sdiff - dshmidt > 0
    trigoff = sdiff + dshmidt < 0 
    turnon = ( trigon & np.logical_not( trigon_prev ) ) & np.logical_not(turn_state)
    turnoff = ( trigoff & np.logical_not( trigoff_prev ) ) & turn_state 
    nswitch = turnon.astype(int) - turnoff.astype(int) # Turn on if 1, Turn off if -1, No action if 0. - Also works 1*turnon - 1*turnoff

    trigon_prev = trigon
    trigoff_prev = trigoff

    return trigon_prev, trigoff_prev, nswitch



##############################################################################
##############################################################################
## Trigger class for one sensor ##

# Thread that takes sensor queue as input and sends individual midi triggers as output
class WeighTrig_ai(Thread):
    def __init__(self, pressq, flexq, flexsize, thresh, dshmidt):
        Thread.__init__(self)
        self.pressq = pressq 
        self.flexq = flexq 
        self.thresh = thresh
        self.dshmidt = dshmidt
        self.daemon = True
        self.flexsize = flexsize

        self.C2midi, midi2C = make_C2midi()
        print(self.C2midi)

        L = Learn(includefile=settingsDir + 'excludeSingleNotes.txt', trainsize=0.9)
        Fpair_full, Ndiff_full, flex_array0, flex_array1, flex_full = L.stat.get_features_wtu(idxminus=flexsize, idxplus=0)
        self.tu_predictor, accuracy = L.learn_thumbunder(flex_array0, flex_array1) # Returns thumb_under predictors of length[1]
        data = L.learn_transition_prob_withThumbUnder()
        self.Ndomain = data.Ndomain 
        self.Fdomainidx = data.Fdomainidx
        self.Tmat = data.Tmat

    def run(self):

        trigon_prev = False
        trigoff_prev = False
        finger_prev = 0
        noteC_prev = 20 
        turn_state = np.zeros(5, dtype=bool)
        flexdeq = deque([0 for i in range(self.flexsize)], maxlen=self.flexsize)
        noteC_prev = 20  
        NotesOn = [0, 0, 0, 0, 0]

        while True: 
            pressvalue = self.pressq.get(block=True)
            flexvalue = self.flexq.get(block=True)
            flexdeq.append( flexvalue[0] )
            [trigon_prev, trigoff_prev, nswitch] = triggerfun(pressvalue, self.thresh, self.dshmidt, trigon_prev, trigoff_prev, turn_state)
            
            if((nswitch==-1).any()):
                # turn_state = nswitch + turn_state
                finger = np.nonzero(nswitch==-1)[0][0] # assumes one finger released at a TrigNote_midinum.
                turn_state[finger] = 0
                TrigNote_midinum(self.C2midi[NotesOn[finger]], 0) # make C2midi
                print(NotesOn)
                print(nswitch)
                print('-------')

            if((nswitch==1).any()):
                # turn_state = nswitch + turn_state
                finger = np.nonzero(nswitch==1)[0][0] # assumes only one finger triggered at a time.
                turn_state[finger] = 1
                noteC = self.aifunction(finger_prev, finger, noteC_prev, flexdeq )
                TrigNote_midinum(self.C2midi[noteC], 80)
                finger_prev = finger
                noteC_prev = noteC
                NotesOn[finger] = noteC
                print(noteC_prev)
                print(NotesOn)
                print(nswitch)
                print('-------')

    def aifunction(self, finger_prev, finger, noteC_prev, flexdeq):
        # print(np.asarray(flexdeq).reshape(1, -1))
        tupred = self.tu_predictor[finger].predict(np.asarray(flexdeq).reshape(1, -1)) if finger != 4 else [0.0]
        tu = int(tupred[0])

        feature = ((finger_prev, finger), tu)
        noteC_diff = np.random.choice(self.Ndomain, p=self.Tmat[self.Fdomainidx[feature], :]) # choose according to distribution
        return noteC_prev + noteC_diff


##############################################################################
##############################################################################
## 2 Hand triggering combined ##
