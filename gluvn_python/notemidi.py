'''
Created on Mar 31, 2016

@author: josephbakarji
'''
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

def make_C2midi():
    Cnotes = []
    for key, val in csv.reader(open(current_dir+'/data/tables/note2num.csv')):
        if(len(key)==2):
            Cnotes.append(val)
    midi2C = dict()
    C2midi = []
    for idx, val in enumerate(sorted(Cnotes)):
        midi2C[int(val)] = idx 
        C2midi.append(int(val))

    return C2midi, midi2C

# Takes an analog sensor vector stream as input and sends midi on or off according to threshold
def signswitch2note(switch, sensarr, notearr):
    for i in range(len(switch)):
        if(switch[i] == 1):
            message = mido.Message('note_on', note=note2numdict[notearr[i]], velocity=80)#sens2vol(sensarr[i], 'linear'))
            output.send(message)
            print( notearr[i] )
        if(switch[i] == -1):
            message = mido.Message('note_off', note=note2numdict[notearr[i]], velocity=0)
            output.send(message)
            print( notearr[i] )

# Takes an analog sensor vector stream as input and sends midi on or off according to threshold
def TrigNote(notename, vel):
    message = mido.Message('note_on', note=note2numdict[notename], velocity=vel )#sens2vol(sensarr[i], 'linear'))
    output.send(message)
    print( notename )

def PitchBend(pitch_change):
    #pitch = note2numdict[notename]
    message = mido.Message('pitchwheel', pitch=pitch_change)
    output.send(message)

def Aftertouch(val, channel=1):
    message = mido.Message('aftertouch', channel=channel, value=val)
    output.send(message)

def Polytouch(notename, val, channel=1):
    message = mido.Message('polytouch', channel=channel, note=note2numdict[notename], value=val)
    output.send(message)

def stop():
    output.reset()

def TrigNote_midinum(notenum, vel):
    message = mido.Message('note_on', note=notenum, velocity=vel )#sens2vol(sensarr[i], 'linear'))
    output.send(message)

# Map analog sensor value to midi volume
def sens2vol(sensval, method):
    if(method == 'linear'):
        return int(sensval * 100/256)
    
    if(method == 'chord'):
        return int(abs(sensval) * 100/40)
    


# Send on off midi messages for chord triggers.
def TriggerChordTest(trigsens, pitchvolume, state):
    chord = trig2chord(trigsens)
    if(chord != None):
        if(state =='off'):
            for i in range(len(chord)):
                message = mido.Message('note_off', note=note2numdict[chord[i]], velocity=80)# sens2vol(pitchvolume, 'chord'))
                output.send(message)
        
        if(state == 'on'):
            print(chord)
            for i in range(len(chord)):
                print(chord[i])
                message = mido.Message('note_on', note=note2numdict[chord[i]], velocity=80)# sens2vol(pitchvolume, 'chord'))
                output.send(message)
            

# TODO: Write arrays in separate file.
# Input 5 binary numbers deciding whether a finger is on or off, and outputs a list a notes to be played (chord).
def trig2chord(binarylist):
    list1 = {'01111': ['C3', 'E3', 'G3'], 
            '00111': ['D3', 'F3', 'A3'], 
            '00011': ['E3', 'G3', 'B3'], 
            '00001': ['F3', 'A3', 'C4'], 
            '00000': ['G3', 'B3', 'D4'], 
            '10000': ['A3', 'C4', 'E4'], 
            '11000': ['B3', 'D4', 'F4']}
    
    listminor = {'01111': ['A2', 'C3', 'E3'], 
                '00111': ['B2', 'D3', 'F3'], 
                '00011': ['C3', 'E3', 'G3'], 
                '00001': ['D3', 'F3', 'A3'], 
                '00000': ['E3', 'G#3', 'B3'], 
                '10000': ['F3', 'A3', 'C4'], 
                '11000': ['G3', 'B3', 'D4']}
    
    trigint = ''.join([str(int(x)) for x in binarylist])
    
    try:
        temp = listminor[trigint]
    except KeyError:
        temp = None
    
    return temp
