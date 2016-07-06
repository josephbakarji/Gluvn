'''
Created on Mar 31, 2016

@author: josephbakarji
'''
import mido
import csv

rtmidi = mido.Backend('mido.backends.rtmidi')
output = rtmidi.open_output(IACDriver)

note2num_dict = {}
for key, val in csv.reader(open('data/tables/note2num.csv')):
    note2num_dict[key] = int(val)


def signswitch2note(switch, sensarr, notearr):
    for i in range(len(switch)):
        if(switch[i] == 1):
            message = mido.Message('note_on', note= note2num_dict[notearr[i]], velocity=sens2vol(sensarr[i], 'linear'))
            output.send(message)
        if(switch[i] == -1):
            message = mido.Message('note_off', note= note2num_dict[notearr[i]], velocity = 0)
            output.send(message)


def sens2vol(sensval, method):
    if(method == 'linear'):
        return int(sensval * 100/256)
    
    if(method == 'chord'):
        return int(abs(sensval) * 100/40)
    

    
def TriggerChordTest(trigsens, pitchvolume, state):
    chord = trig2chord(trigsens)
    if(chord != None):
        if(state =='off'):
            for i in range(len(chord)):
                message = mido.Message('note_off', note = note2num_dict(chord[i]), velocity = 80)# sens2vol(pitchvolume, 'chord'))
                output.send(message)
        
        if(state == 'on'):
            print(chord)
            for i in range(len(chord)):
                print(chord[i])
                message = mido.Message('note_on', note = note2num_dict(chord[i]), velocity = 80)# sens2vol(pitchvolume, 'chord'))
                output.send(message)
            

# send data as a single byte 11101
def trig2chord(binarylist):
    list = {'01111': ['C3', 'E3', 'G3'], 
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
