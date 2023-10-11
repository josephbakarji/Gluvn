'''
Created on May 20, 2016

@author: josephbakarji
'''

import numpy as np
from notemidi import TrigNote, TrigNote_midinum, signswitch2note, TriggerChordTest, make_C2midi
from notemidi import  PitchBend, Aftertouch
from __init__ import settingsDir
from learning import Learn
from threading import Thread
from collections import deque


## COPIED FROM notemidi.py
import mido
import csv
from __init__ import IACDriver
import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

rtmidi = mido.Backend('mido.backends.rtmidi')
output = rtmidi.open_output(IACDriver)

# Note to Midi number dictionary
note2numdict = {}
for key, val in csv.reader(open(current_dir+'/data/tables/note2num.csv')):
    note2numdict[key] = int(val)



#######################################################################################################################################
#######################################################################################################################################








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
class WeighTrig(Thread):
    def __init__(self, sensorq, thresh, notearr):
        Thread.__init__(self)
        self.sensorq = sensorq
        self.thresh = thresh
        self.notearr = notearr
        self.daemon = True

    def run(self):

        trigon_prev = False
        trigoff_prev = False
        turn_state = np.zeros(5, dtype=bool)
        dshmidt = 5     # use as input

        while True: 
            sensvalue = self.sensorq.get(block=True)
            [trigon_prev, trigoff_prev, nswitch] = triggerfun(sensvalue, self.thresh, dshmidt, trigon_prev, trigoff_prev, turn_state)
            
            if( not(all(i == 0 for i in nswitch)) ): # use if(nswitch.any())
                turn_state = nswitch + turn_state
                signswitch2note(nswitch, sensvalue, self.notearr)


##########################################################################################################

# Use trigger and bend pitchwheel and other
class TrigBend(Thread):
    def __init__(self, pimuq, thresh, notearr):
        Thread.__init__(self)
        self.pressimuq = pimuq
        self.thresh = thresh
        self.notearr = notearr
        self.daemon = True

    def run(self):

        trigon_prev = False
        trigoff_prev = False
        turn_state = np.zeros((5,), dtype=int)
        dshmidt = 5     # use as input
        zero_gyrin = 32767.0
        max_bend = 2000 # Actual range is -8191..8191
        # pressq_prev = np.zeros((len(turn_state))) 
        noteplayed = None
        volprev = 30
        
        
        while True: 
            sensvalue = self.pressimuq.get(block=True)
            pressq = np.asarray(sensvalue[-5:])
            scaled_gyrq = (np.asarray(sensvalue[3:6], dtype=float) - zero_gyrin ) / zero_gyrin * max_bend
            # print(gyrq[-1])


            # trigger 
            sdiff = pressq - self.thresh        # subtract threshold from readings
            trigon = sdiff - dshmidt > 0
            trigoff = sdiff + dshmidt < 0 
            turnon = ( trigon & np.logical_not( trigon_prev ) ) & np.logical_not(turn_state) # Redundant?
            turnoff = ( trigoff & np.logical_not( trigoff_prev ) ) & turn_state 
            nswitch = turnon.astype(int) - turnoff.astype(int) # Turn on if 1, Turn off if -1, No action if 0. - Also works 1*turnon - 1*turnoff
            trigon_prev = trigon
            trigoff_prev = trigoff

            if nswitch.any(): # use if(nswitch.any())
                # extrapolation = ((pressq + 0.5*(pressq - pressq_prev) ) / 255.0 * 127.0 ).astype(int)
                # Only used for turn_state=1. 
                # Can get rid of np.max([i, 0])
                # velocity = np.array([np.max([i, 0]) if i<127 else 127 for i in extrapolation])
                # print(pressq, pressq_prev, velocity)

                # (nswitch, velocity, self.notearr)
                # pressq_prev = pressq

                idxtrig = np.where(nswitch==1)[0] #Assuming it returns one value
                idxuntrig = np.where(nswitch==-1)[0] #Assuming it returns one value
                on_idx = np.where(turn_state==1)[0]
                # turn_state = nswitch + turn_state

                if len(idxtrig)!=0:
                    noteplayed = idxtrig[0]
                    message = mido.Message('note_on', note=note2numdict[self.notearr[noteplayed]], velocity=30)
                    output.send(message)
                    turn_state[noteplayed] = 1
                    print(turn_state)
                    if len(on_idx)>0:
                        message = mido.Message('note_off', note=note2numdict[self.notearr[on_idx[0]]], velocity=0)
                        output.send(message)
                        turn_state[on_idx[0]] = 0
                        print(turn_state)
                        PitchBend(0)
                if len(idxuntrig)!=0:
                    if turn_state[idxuntrig[0]] == 1:
                        print('UNTRIGGERING')
                        message = mido.Message('note_off', note=note2numdict[self.notearr[idxuntrig[0]]], velocity=0)
                        output.send(message)
                        PitchBend(0)
                        turn_state[idxuntrig[0]] = 0
                        noteplayed = None
                        volprev = 30

            # Bend
            if noteplayed is not None:
                PitchBend(int(scaled_gyrq[-1]))

                pressed_idx = np.where(pressq>20)
                print(pressed_idx)
                if len(pressed_idx[0])>0:
                    press_val = np.mean(pressq[pressed_idx])
                else:
                    press_val = pressq[noteplayed] 

                volnew = volprev + 0.01 * (press_val*127/255 - volprev)
                print(volnew)
                Aftertouch(int(volnew))
                volprev = volnew
                ## TODO: assign average between previous and new note

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


class WeighTrig2h(Thread):
    def __init__(self, sensorq, collectq, hand, thresh, dshmidt):
        Thread.__init__(self)
        self.sensorq = sensorq
        self.collectq = collectq
        self.hand = hand
        self.thresh = thresh
        self.dshmidt = dshmidt
        self.daemon = True

    def run(self):
        trigon_prev = False
        trigoff_prev = False
        turn_state = np.zeros(5, dtype=bool)

        while True: 
            sensvalue = self.sensorq.get(block=True)
            [trigon_prev, trigoff_prev, nswitch] = triggerfun(sensvalue, self.thresh, self.dshmidt, trigon_prev, trigoff_prev, turn_state)

            #print(self.hand)
            if nswitch.any(): 
                print(self.hand)
                turn_state = nswitch + turn_state 
                state = [turn_state, nswitch, self.hand]
                self.collectq.put(state)


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



#####


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
                    

# Use trigger and bend pitchwheel and other
# class TrigBend(Thread):
#     def __init__(self, pimuq, thresh, notearr):
#         Thread.__init__(self)
#         self.pressimuq = pimuq
#         self.thresh = thresh
#         self.notearr = notearr
#         self.daemon = True

#     def run(self):

#         trigon_prev = False
#         trigoff_prev = False
#         turn_state = np.zeros((5,), dtype=int)
#         dshmidt = 5     # use as input
#         zero_gyrin = 32767.0
#         max_bend = 2000 # Actual range is -8191..8191
#         pressq_prev = np.asarray([0])
        
        

#         while True: 
#             sensvalue = self.pressimuq.get(block=True)
#             pressq = np.asarray(sensvalue[-5:])
#             scaled_gyrq = (np.asarray(sensvalue[3:6], dtype=float) - zero_gyrin ) / zero_gyrin * max_bend
#             # print(gyrq[-1])


#             # trigger 
#             sdiff = pressq - self.thresh        # subtract threshold from readings
#             trigon = sdiff - dshmidt > 0
#             trigoff = sdiff + dshmidt < 0 
#             turnon = ( trigon & np.logical_not( trigon_prev ) ) & np.logical_not(turn_state) # Redundant?
#             turnoff = ( trigoff & np.logical_not( trigoff_prev ) ) & turn_state 
#             nswitch = turnon.astype(int) - turnoff.astype(int) # Turn on if 1, Turn off if -1, No action if 0. - Also works 1*turnon - 1*turnoff
#             trigon_prev = trigon
#             trigoff_prev = trigoff

#             if nswitch.any(): # use if(nswitch.any())
#                 # turn_state = nswitch + turn_state
#                 extrapolation = ((pressq + 0.5*(pressq - pressq_prev) ) / 255.0 * 127.0 ).astype(int)
#                 # Only used for turn_state=1. 
#                 # Can get rid of np.max([i, 0])
#                 velocity = np.array([np.max([i, 0]) if i<127 else 127 for i in extrapolation])
#                 # print(pressq, pressq_prev, velocity)

#                 turn_state, switch_single(nswitch, velocity, self.notearr)
#                 pressq_prev = pressq

#                 #pitch bend those tuning off two zeros
#                 pitchbend_trignotes(turnoff.astype(int), 0)

#             # Bend
#             if turn_state.any():
#                 pitchbend_trignotes(int(scaled_gyrq[-1]))
#                 aftertouch_trignotes(aftertouch_val)
