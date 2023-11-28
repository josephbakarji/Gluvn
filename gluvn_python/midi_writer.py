'''
Created on Mar 31, 2016

@author: josephbakarji
'''
import mido
import csv
from __init__ import IACDriver
import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

class MidiWriter:
    def __init__(self):
        self.rtmidi = mido.Backend('mido.backends.rtmidi')
        self.midiout = self.rtmidi.open_output(IACDriver) 

    # Takes an analog sensor vector stream as input and sends midi on or off according to threshold
    def trig_notes_array_playing(self, switch, playing_notes, notearr, vel=80):
        for i in range(len(switch)):
            if(switch[i] == 1):
                self.trig_note( notearr[i], vel)
                playing_notes[i] = notearr[i]
            if(switch[i] == -1):
                self.trig_note( playing_notes[i], 0)
                playing_notes[i] = None
        return playing_notes 

    def turn_off_all_playing(self, playing_notes):
        for i in range(len(playing_notes)):
            if playing_notes[i] is not None:
                self.trig_note( playing_notes[i], 0)
                playing_notes[i] = None
        return playing_notes 

    def trig_note_array(self, switch, notearr, vel=80):
        for i in range(len(switch)):
            if(switch[i] == 1):
                self.trig_note( notearr[i], vel)
            if(switch[i] == -1):
                self.trig_note( notearr[i], 0)

    # Takes an analog sensor vector stream as input and sends midi on or off according to threshold
    def trig_note(self, notemidi, vel):
        message = mido.Message('note_on', note=notemidi, velocity=vel )#sens2vol(sensarr[i], 'linear'))
        self.midiout.send(message)

    def pitch_bend(self, pitch_change):
        #pitch = note2numdict[notename]
        message = mido.Message('pitchwheel', pitch=pitch_change)
        self.midiout.send(message)

    def aftertouch(self, val, channel=1):
        message = mido.Message('aftertouch', channel=channel, value=val)
        self.midiout.send(message)


    def control_change(self, val, control, channel=1):
        message = mido.Message('control_change', channel=channel, control=control, value=val)
        self.midiout.send(message)

    def polytouch(self, notename, val, channel=1):
        message = mido.Message('polytouch', channel=channel, note=note2numdict[notename], value=val)
        self.midiout.send(message)

    def stop(self):
        self.midiout.reset()

    # def trigNote_midinum(self, notenum, vel):
    #     message = mido.Message('note_on', note=notenum, velocity=vel )#sens2vol(sensarr[i], 'linear'))
    #     self.midiout.send(message)

    # Map analog sensor value to midi volume
    # def sens2vol(sensval, method):
    #     if(method == 'linear'):
    #         return int(sensval * 100/256)
        
    #     if(method == 'chord'):
    #         return int(abs(sensval) * 100/40)
    

    # # Send on off midi messages for chord triggers.
    # def TriggerChordTest(self, trigsens, pitchvolume, state):
    #     chord = trig2chord(trigsens)
    #     if(chord != None):
    #         if(state =='off'):
    #             for i in range(len(chord)):
    #                 message = mido.Message('note_off', note=chord[i], velocity=80)# sens2vol(pitchvolume, 'chord'))
    #                 self.midiout.send(message)
            
    #         if(state == 'on'):
    #             print(chord)
    #             for i in range(len(chord)):
    #                 print(chord[i])
    #                 message = mido.Message('note_on', note=chord[i], velocity=80)# sens2vol(pitchvolume, 'chord')) 
    #                 self.midiout.send(message)
            

    # # TODO: Write arrays in separate file - Add to mapper
    # # Input 5 binary numbers deciding whether a finger is on or off, and outputs a list a notes to be played (chord).
    # def trig2chord(binarylist):
    #     list1 = {'01111': ['C3', 'E3', 'G3'], 
    #             '00111': ['D3', 'F3', 'A3'], 
    #             '00011': ['E3', 'G3', 'B3'], 
    #             '00001': ['F3', 'A3', 'C4'], 
    #             '00000': ['G3', 'B3', 'D4'], 
    #             '10000': ['A3', 'C4', 'E4'], 
    #             '11000': ['B3', 'D4', 'F4']}
        
    #     listminor = {'01111': ['A2', 'C3', 'E3'], 
    #                 '00111': ['B2', 'D3', 'F3'], 
    #                 '00011': ['C3', 'E3', 'G3'], 
    #                 '00001': ['D3', 'F3', 'A3'], 
    #                 '00000': ['E3', 'G#3', 'B3'], 
    #                 '10000': ['F3', 'A3', 'C4'], 
    #                 '11000': ['G3', 'B3', 'D4']}
        
    #     trigint = ''.join([str(int(x)) for x in binarylist])
        
    #     try:
    #         temp = listminor[trigint]
    #     except KeyError:
    #         temp = None
        
    #     return temp
