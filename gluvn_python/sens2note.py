from __future__ import division
from __init__ import portL, portR, baud, testDir, simDir, EXPDIR, learnDir, keyboard_portname
from port_read import ReadSerial, ParseSerial, printSens, ReadKeyboard, ParseFile
from data_analysis import ReadWrite, Analyze, Analyze2Hands
import time
import numpy as np
import random
from threading import Thread
import binascii
from struct import *
from collections import namedtuple
import matplotlib.pyplot as plt
import math
import queue
from collections import deque
import csv, sys, os
import mido
import json

class BaseApp(Thread):
    def __init__(self, 
                 root_note='C',
                 scale='major',
                 thresholds={'flex': 140, 'press': 20},
                 trigger_sensors={'l': 'flex', 'r': 'flex'},
                 hysteresis=5,
                 directory=testDir,
                 filename='test0'):
        Thread.__init__(self)
        self.directory = directory
        self.filename = filename
        self.root_note = root_note
        self.scale = scale
        self.thresholds = thresholds
        self.trigger_sensors = trigger_sensors
        self.hysteresis = hysteresis
        self.time0 = time.time()
        self.daemon = True

        # Initialize MIDI output
        self.midi_out = mido.open_output()
        self.midi_out.send(mido.Message('program_change', program=0))

        # Initialize sensor reading
        self.sensor_config = {'l': {'flex': True, 'press': True, 'imu': True},
                            'r': {'flex': True, 'press': True, 'imu': True}}
        self.reader = Reader(sensor_config=self.sensor_config, save=False)
        self.hands = list(self.sensor_config.keys())
        self.triggers = {}

        # Initialize note mapping
        self.notes = self.generate_notes()
        self.active_notes = {hand: {} for hand in self.hands}

    def generate_notes(self):
        notes = {}
        for hand in self.hands:
            if hand == 'l':
                notes[hand] = ['C3', 'D3', 'E3', 'F3', 'G3']
            else:
                notes[hand] = ['C4', 'D4', 'E4', 'F4', 'G4']
        return notes

    def run(self):
        self.reader.start_readers()
        for hand in self.hands:
            self.triggers[hand] = MovingWindow(
                sensorq=self.reader.threads[hand]['parser'].getQ(),
                hand=hand,
                threshold=self.thresholds[self.trigger_sensors[hand]],
                hysteresis=self.hysteresis,
                notes=self.notes[hand]
            )
            self.triggers[hand].start()

    def stop(self):
        self.reader.stop_readers()
        for hand in self.hands:
            self.triggers[hand].join(timeout=.1)
        self.join(timeout=.1)

class MovingWindow(Thread):
    def __init__(self, sensorq, hand, threshold, hysteresis, notes):
        Thread.__init__(self)
        self.sensorq = sensorq
        self.hand = hand
        self.threshold = threshold
        self.hysteresis = hysteresis
        self.notes = notes
        self.daemon = True
        self.window_size = 5
        self.window = deque(maxlen=self.window_size)
        self.last_state = np.zeros(5, dtype=bool)
        self.active_notes = {}

    def run(self):
        while True:
            try:
                data = self.sensorq.get(block=True)
                self.window.append(data)
                
                if len(self.window) == self.window_size:
                    avg_data = self.calculate_average()
                    current_state = self.process_data(avg_data)
                    
                    # Handle note on/off events
                    for i in range(5):
                        if current_state[i] and not self.last_state[i]:
                            # Note on
                            note = self.notes[i]
                            self.active_notes[i] = note
                            self.send_midi('note_on', note=note, velocity=80)
                        elif not current_state[i] and self.last_state[i]:
                            # Note off
                            if i in self.active_notes:
                                self.send_midi('note_off', note=self.active_notes[i], velocity=0)
                                del self.active_notes[i]
                    
                    self.last_state = current_state
            except Exception as e:
                print(f"Error in MovingWindow: {e}")

    def calculate_average(self):
        if not self.window:
            return None
        
        # Convert window to numpy array for easier averaging
        window_array = np.array([d[self.trigger_sensors[self.hand]] for d in self.window])
        return np.mean(window_array, axis=0)

    def process_data(self, data):
        if data is None:
            return np.zeros(5, dtype=bool)
        
        # Apply threshold with hysteresis
        state = np.zeros(5, dtype=bool)
        for i in range(5):
            if data[i] > self.threshold + self.hysteresis:
                state[i] = True
            elif data[i] < self.threshold - self.hysteresis:
                state[i] = False
            else:
                state[i] = self.last_state[i]
        
        return state

    def send_midi(self, msg_type, note, velocity):
        try:
            if msg_type == 'note_on':
                msg = mido.Message('note_on', note=self.note_to_midi(note), velocity=velocity)
            else:
                msg = mido.Message('note_off', note=self.note_to_midi(note), velocity=velocity)
            self.midi_out.send(msg)
        except Exception as e:
            print(f"Error sending MIDI message: {e}")

    def note_to_midi(self, note):
        # Convert note name to MIDI number
        notes = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
        note_name = note[:-1]
        octave = int(note[-1])
        return notes.index(note_name) + (octave + 1) * 12

if __name__=="__main__":
    root_note = 'D'
    scale = 'minor'
    thresholds = {'flex': 140, 'press': 20}
    trigger_sensors = {'l': 'flex', 'r': 'flex'}
    hysteresis = 5

    app = BaseApp(root_note=root_note,
                scale=scale,
                thresholds=thresholds,
                trigger_sensors=trigger_sensors,
                hysteresis=hysteresis)

    app.start()

    key = input('press any key to finish \n')
    print('Shutting down...')

    app.reader.stop_readers()
    for hand in app.hands:
        app.triggers[hand].join(timeout=.1)
    app.join(timeout=.1) 