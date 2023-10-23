import numpy as np
from code.gluvn_python.midi_writer import TrigNote, TrigNote_midinum, signswitch2note, TriggerChordTest, make_C2midi
from code.gluvn_python.midi_writer import PitchBend, Aftertouch
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

ZERO_GYRIN = 32767.0
MAX_BEND = 2000
HIGH_JUMP_THRESHOLD = 100

# Note to Midi number dictionary
note2numdict = {}
for key, val in csv.reader(open(current_dir+'/data/tables/note2num.csv')):
    note2numdict[key] = int(val)



def triggerfun(sensvalue, thresh, dshmidt, trigon_prev, trigoff_prev, turn_state):
    sdiff = sensvalue - thresh        # subtract threshold from readings
    trigon = sdiff - dshmidt > 0
    trigoff = sdiff + dshmidt < 0 
    turnon = ( trigon & np.logical_not( trigon_prev ) ) & np.logical_not(turn_state)
    turnoff = ( trigoff & np.logical_not( trigoff_prev ) ) & turn_state 
    nswitch = turnon.astype(int) - turnoff.astype(int) # Turn on if 1, Turn off if -1, No action if 0. - Also works 1*turnon - 1*turnoff

    trigon_prev = trigon
    trigoff_prev = trigoff

    return trigon_prev, trigoff_prev, nswitch

class TrigBend_L(Thread):
    def __init__(self, sensorq, collectq, thresh, dshmidt):
        Thread.__init__(self)
        self.sensorq = sensorq
        self.collectq = collectq
        self.hand = 'L' 
        self.thresh = thresh
        self.dshmidt = dshmidt
        self.daemon = True

    def run(self):
        trigon_prev = False
        trigoff_prev = False
        turn_state = np.zeros(5, dtype=bool)
        sensvalue_prev = np.zeros(5)

        while True: 
            sensvalue = np.asarray(self.sensorq.get(block=True))

            # This is a hack to prevent sensor fluctuations; should be fixed in the glove
            if (np.abs(sensvalue - sensvalue_prev)>100).any():
                continue
            sensvalue_prev = sensvalue
            [trigon_prev, trigoff_prev, nswitch] = triggerfun(sensvalue, self.thresh, self.dshmidt, trigon_prev, trigoff_prev, turn_state)
            if nswitch.any(): 
                print(self.hand)
                turn_state = nswitch + turn_state 
                # if (turn_state != [0, 0, 0, 0]).any():
                state = [self.hand, turn_state, nswitch]
                self.collectq.put(state)



#### RIGHT HAND ####

class TrigBend_R(Thread):
    def __init__(self, pimuq, collectq, thresh, dshmidt):
        Thread.__init__(self)
        self.collectq = collectq
        self.pressimuq = pimuq
        self.hand = 'R'
        self.thresh = thresh
        self.dshmidt = dshmidt
        self.daemon = True

    def run(self):

        trigon_prev = False
        trigoff_prev = False
        turn_state = np.zeros((4,), dtype=int)
        zero_gyrin = 32767.0
        max_bend = 2000 # Actual range is -8191..8191
        
        
        while True: 
            sensvalue = self.pressimuq.get(block=True)
            pressq = np.asarray(sensvalue[-4:])
            scaled_gyrq = (np.asarray(sensvalue[3:6], dtype=float) - zero_gyrin ) / zero_gyrin * max_bend

            [trigon_prev, trigoff_prev, nswitch] = triggerfun(pressq, self.thresh, self.dshmidt, trigon_prev, trigoff_prev, turn_state)

            thread_info = [self.hand]

            switch = False
            thread_info += [pressq, scaled_gyrq]

            if nswitch.any(): 
                switch = True
                idxtrig = np.where(nswitch==1)[0] #Assuming it returns one value
                idxuntrig = np.where(nswitch==-1)[0] #Assuming it returns one value
                on_idx = np.where(turn_state==1)[0]
                thread_info += [idxtrig, idxuntrig, on_idx, turn_state.copy()]

                if len(idxtrig)!=0:
                    turn_state[idxtrig[0]] = 1
                    if len(on_idx)>0:
                        turn_state[on_idx[0]] = 0
                if len(idxuntrig)!=0:
                    if turn_state[idxuntrig[0]] == 1:
                        turn_state[idxuntrig[0]] = 0

            thread_info += [switch]
            self.collectq.put(thread_info)


# Collect and combine glove readings
class TrigBend_Combine(Thread):
    def __init__(self, collectq, basenote, scale, noterange):
        Thread.__init__(self)
        self.collectq = collectq
        self.basenote = basenote
        self.scale = scale
        self.noterange = noterange 
        self.daemon = True

    def run(self):

        numfingersR = 4 # Add to inputs
        numfingersL = 5 # Add to inputs
        WArr, NArr, scalearr = GenerateHandMap(self.basenote, numfingersR, self.scale, mode='dminor', baseoct=2, noterange=self.noterange)
        notearr, notearr_idx = WindowMap(np.zeros(numfingersL), WArr, NArr)

        noteplayed = None
        idxplayed = None
        volprev = 30
        smooth_factor = 0.04

        while True:
            thread_info = self.collectq.get(block=True)
            
            # If right hand reading
            if(thread_info[0] == 'R'): 
                switch = thread_info[-1]

                # If a note is being switched, turn notes on and off accordingly 
                if switch: # Switching notes
                    idxtrig, idxuntrig, on_idx, turn_state = thread_info[3:7]

                    # if note was untriggered, turn it off, and reset noteplayed to None
                    if len(idxuntrig)!=0 and turn_state[idxuntrig[0]] == 1:
                        message = mido.Message('note_off', note=noteplayed, velocity=0)
                        output.send(message)
                        PitchBend(0)
                        if len(idxtrig)==0:
                            noteplayed = None
                            idxplayed = None
                        volprev = 30
                        
                    # if note was triggered, turn it on
                    if len(idxtrig)!=0:

                        # First, turn off the note that was previously played
                        if len(on_idx)>0:
                            message = mido.Message('note_off', note=noteplayed, velocity=0)
                            output.send(message)
                            PitchBend(0)
            
                        print(idxplayed)

                        # if the triggered note is the last (pinkie), set note played to that in the next window
                        if idxtrig[0] == 3:
                            if idxplayed is None:
                                idxplayed = 0
                            noteplayed = NArr[notearr_idx+1][idxplayed]

                        # otherwise, the note played is set from the current window according indxtrig
                        else:
                            idxplayed = idxtrig[0]
                            noteplayed = notearr[idxplayed]
                            print(notearr)

                        # send note
                        print(noteplayed)
                        message = mido.Message('note_on', note=noteplayed, velocity=30)
                        output.send(message)


                # If there is a note that is being played, send pitch bend and aftertouch
                if idxplayed is not None:
                    pressq, scaled_gyrq = thread_info[1:3]
                    PitchBend(int(scaled_gyrq[-1]))

                    # check if any of the fingers are pressed (using 20 threshold)
                    pressed_idx = np.where(pressq>20)

                    # if any finger is pressed, use the average of the pressed fingers to set volume
                    if len(pressed_idx[0]) > 0:
                        press_val = np.mean(pressq[pressed_idx])

                    # otherwise, use current value of the finger that is being played
                    else:
                        press_val = pressq[idxplayed] 

                    # set the velocity to a new value using a smoothing factor
                    volnew = volprev + smooth_factor * (press_val*127/255 - volprev)

                    Aftertouch(int(volnew))

                    volprev = volnew

            else: 
                notearr, notearr_idx = WindowMap(thread_info[1], WArr, NArr)
                print('notearr:', notearr)


def WindowMap(state, WArr, NArr):
    for i, win in enumerate(WArr):
        if (win == state).all():
            return NArr[i], i
    return NArr[3], 3
# 

def GenerateHandMap(basenote, numfingers, scale, mode, baseoct=3, noterange=None):

    notes = generate_scale(basenote, scale)

    if mode == 'flexpress':
        WArr = [ [1, 1, 1, 1, 1], 
        [0, 1, 1, 1, 1], 
        [0, 0, 1, 1, 1], 
        [0, 0, 0, 1, 1], 
        [0, 0, 0, 0, 1], 
        [0, 0, 0, 0, 0],
        [1, 0, 0, 0, 0],
        [1, 1, 0, 0, 0],
        [1, 1, 1, 0, 0],
        [1, 1, 1, 1, 0] ]

        # Find combination containing midnote 
        for idx, WArrcomb in enumerate(WArr):
            if WArrcomb == [0, 0, 0, 0, 0]:
                WArr_mid = idx
                break

        # Find index of midnote 
        midnote = basenote + str(baseoct) 
        for idx, note in enumerate(notes):
            if note == midnote:
                idx_midnote = idx 
            if noterange is not None:
                if note == noterange[0]:
                    idx_firstnote = idx 
                if note == noterange[1]:
                    idx_endnote = idx 

        if noterange is None:
            numcombinations = len(WArr)
            idx_firstnote = idx_midnote - numfingers*WArr_mid
            idx_endnote = idx_firstnote+numcombinations*numfingers
            notes_span = notes[idx_firstnote:idx_endnote]
            print(idx_firstnote, idx_endnote)
            NArr = np.reshape(notes_span, (numcombinations, numfingers))
        else:
            notes_span = notes[idx_firstnote:idx_endnote]
            numcomb = np.floor(len(notes_span)/numfingers)
            WArr = WArr[WArr_mid-np.floor(numcomb/2):WArr_mid+np.floor(numcomb/2)]
            notes_span = notes_span[:len(WArr)]
            NArr = np.reshape(notes_span, (len(WArr), numfingers))

    if mode == 'press2h':
        WArr = [ [0, 1, 1, 1], 
        [0, 0, 1, 1], 
        [0, 0, 0, 1], 
        [0, 0, 0, 0], 
        [1, 0, 0, 0],
        [1, 1, 0, 0],
        [1, 1, 1, 0]]

        # Find combination containing midnote 
        for idx, WArrcomb in enumerate(WArr):
            if WArrcomb == [0,0,0,0]:
                WArr_mid = idx
                break

        # Find index of midnote 
        midnote = basenote + '2'
        for idx, note in enumerate(notes):
            if note == midnote:
                idx_midnote = idx 
                break

        numfingers = len(WArr[0])
        numcombinations = len(WArr)
        idx_firstnote = idx_midnote - numfingers*WArr_mid
        notes_span = notes[idx_firstnote:idx_firstnote+numcombinations*numfingers]

        NArr = np.reshape(notes_span, (numcombinations, numfingers))

    if mode == 'standard':

        WArr = [ [1, 1, 1, 1, 1], 
        [0, 1, 1, 1, 1], 
        [0, 0, 1, 1, 1], 
        [0, 0, 0, 1, 1], 
        [0, 0, 0, 0, 1], 
        [0, 0, 0, 0, 0],
        [1, 0, 0, 0, 0],
        [1, 1, 0, 0, 0],
        [1, 1, 1, 0, 0],
        [1, 1, 1, 1, 0] ]

        NArr = [ ['B0', 'C1', 'D1'],
        ['E1', 'F1', 'G1'],
        ['A1', 'B1', 'C2'],
        ['D2', 'E2', 'F2'],
        ['G2', 'A2', 'B2'],
        ['C3', 'D3', 'E3'],
        ['F3', 'G3', 'A3'],
        ['B3', 'C4', 'D4'],
        ['E4', 'F4', 'G4'],
        ['A4', 'B4', 'C5']]

    if mode == 'dminor':

        WArr = [
        [0, 1, 1, 1, 1], 
        [0, 0, 1, 1, 1], 
        [0, 0, 0, 1, 1], 
        [0, 0, 0, 0, 1], 
        [0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 1, 1, 1, 0] ]

        NArr = [
        ['E1', 'F1', 'G1'],
        ['A1', 'A#1', 'C#2'],
        ['D2', 'E2', 'F2'],
        ['G2', 'A2', 'B2'],
        ['D3', 'E3', 'F3'],
        ['G3', 'A3', 'A#3'],
        ['C#4', 'D4', 'E4'],
        ['F4', 'G4', 'A4']]
        
        NArr_num = NArr.copy()
        for i, Nwin in enumerate(NArr):
            for j, Note in enumerate(Nwin):
                NArr_num[i][j] = note2numdict[NArr[i][j]]
            
        scalearr = sum(NArr_num, [])
        NArr = NArr_num

    return WArr, NArr, scalearr


def generate_scale(basenote, scale):
    # Generate all notes
    wnotes = ['C', 'D', 'E', 'F', 'G', 'A', 'B']
    sharps = np.asarray([1, 1, 0, 1, 1, 1, 0]).astype(bool)
    octaves = 6
    all_notes = []
    for i in range(octaves):
        for idx_note, note in enumerate(wnotes):
            all_notes.append(note + str(i+1))
            if sharps[idx_note]:
                all_notes.append(note+'#'+str(i+1))
                
    # Locate base note
    idx_first = None
    for i, note in enumerate(all_notes):
        if note[0] == basenote:
            idx_first = i
            break

    # All possible scales
    scales = {
    'major' : [2, 2, 1, 2, 2, 2, 1],
    'minor' : [2, 1, 2, 2, 2, 1, 2],
    'pentatonic' : [3, 2, 2, 3, 2]
    }

    current_scale = scales[scale]
    curr_idx = [idx_first]

    i = 0
    while curr_idx[-1] < len(all_notes):
        increment = current_scale[i%len(current_scale)]
        i += 1
        curr_idx.append(curr_idx[-1]+increment)

    scale_notes = [all_notes[i] for i in curr_idx[:-1]]
    return scale_notes





