from senstonote_new import BaseApp

import numpy as np
# from code.gluvn_python.midi_writer import TrigNote, TrigNote_midinum, signswitch2note, TriggerChordTest, make_C2midi
from __init__ import settingsDir
from learning import Learn
from threading import Thread
from collections import deque
from mapper import NoteMapper
from port_read import Reader
from midi_writer import MidiWriter
from itertools import repeat, islice
import queue

# Constants

TWO_BYTE = 65535
BYTE = 255
ZERO_GYRIN = 32767.0
MAX_BEND = 2000
ZERO_ACCEL = TWO_BYTE / 4.0 - 680.0


class MovingWindow(BaseApp):
    def __init__(self, *args, 
                 volume_controller=None, 
                 pitch_bender=None, 
                 averaging_window_controller=None,
                 num_lh_fingers=5, 
                 num_rh_fingers=5, 
                 averaging_window_size=10,
                 base_volume=20,
                 instrument='double_flex',
                 **kwargs):

        super().__init__(*args, **kwargs)
        self.num_lh_fingers = num_lh_fingers 
        self.num_rh_fingers = num_rh_fingers 
        self.playing_notes = [None]*num_rh_fingers
        self.volume_controller = volume_controller
        self.pitch_bender = pitch_bender
        self.averaging_window_size_max = averaging_window_size
        self.averaging_window_controller = averaging_window_controller
        self.averaging_queue = deque(repeat(0, self.averaging_window_size_max), maxlen=self.averaging_window_size_max)
        self.base_volume = base_volume
        self.instrument = self.set_instrument(instrument)
        
        self.pitch_bend_limit = 8192
        self.global_volume = 10
        self.averaging_window_size = self.averaging_window_size_max
        self.note_array = None
    
    def set_instrument(self, instrument):
        if instrument == 'double_flex':
            self.window_trigger, self.note_windows = self.mapper.moving_window(num_lhf=self.num_lh_fingers, num_rhf=self.num_rh_fingers)
            self.note_array, note_array_idx = self.mapper.window_map([0]*self.num_lh_fingers, self.window_trigger, self.note_windows)
            return self.double_flex_instrument
        elif instrument == 'ten_finger':
            self.note_windows = self.mapper.basic_map_2hands()
            return self.ten_finger_instrument
        else:
            raise ValueError('Instrument not recognized. Please choose from double_flex or ten_finger')

    def input_scaling(self, input, min_output=0, max_output=127, shift=0, min_input=0, max_input=BYTE):
        output = int( min_output + (max_output - min_output) * ((input + shift) - min_input) / (max_input - min_input))
        return min(output, max_output)

    def scaling_modulo(self, input, min_output=0, max_output=127, min_input=0, max_input=BYTE):
        return int((( max_output + (max_output - min_output) * (input - min_input)/(max_input - min_input)) % (max_output - min_output)) - max_output)

    def pitch_bend(self, reading_dict):
        if self.pitch_bender == 'imu2':
            return self.scaling_modulo(reading_dict[self.pitch_bender], min_output=-8192, max_output=8192, min_input=0, max_input=TWO_BYTE)

    def volume_control(self, reading_dict):
        if self.volume_controller == 'imu1':
            return self.input_scaling(reading_dict[self.volume_controller], max_output=127, min_input=0, max_input=TWO_BYTE)
        
        if self.volume_controller == 'accel_mag':
            norm = np.sqrt((reading_dict['imu3']-TWO_BYTE/2.0)**2 + (reading_dict['imu4']-TWO_BYTE/2.0)**2 + (reading_dict['imu5']-TWO_BYTE/2.0)**2 )
            volume = max(norm - ZERO_ACCEL, 0)
            self.averaging_queue.append(volume)
            mean = sum(islice(self.averaging_queue, self.averaging_window_size_max - self.averaging_window_size, self.averaging_window_size_max))/self.averaging_window_size
            return self.base_volume + self.input_scaling(mean, max_output=127-self.base_volume, min_input=0, max_input=15000)
            

    def window_averaging_conrol(self, reading_dict):
        return self.input_scaling(reading_dict[self.averaging_window_controller], max_output=self.averaging_window_size_max, min_input=0, max_input=TWO_BYTE) 


    def double_flex_instrument(self, reading_dict):
        if reading_dict['hand'] == 'l':
            # assumes left hand is only responsible for window shifting
            temp = self.mapper.window_map(self.triggers['l'].turn_state[:self.num_lh_fingers], self.window_trigger, self.note_windows)
            if temp is not None:
                self.note_array, note_array_idx = temp
        else:
            if np.any(reading_dict['switch']):
                self.playing_notes = self.midi_writer.trig_notes_array_playing(reading_dict['switch'], self.playing_notes, self.note_array, vel=self.global_volume)
            else:
                if self.volume_controller is not None:
                    self.global_volume = self.volume_control(reading_dict)
                    self.midi_writer.aftertouch(self.global_volume)#, channel=1)

                if self.pitch_bender is not None:
                    pitchbend_val = self.pitch_bend(reading_dict) 
                    self.midi_writer.pitch_bend(pitchbend_val)


    def ten_finger_instrument(self, reading_dict):
        self.midi_writer.trig_note_array(reading_dict['switch'], self.note_windows[reading_dict['hand']], vel=self.global_volume)
        # self.playing_notes = self.midi_writer.trig_notes_array_playing(reading_dict['switch'], self.playing_notes, self.note_array, vel=self.global_volume)

        if self.averaging_window_controller is not None and reading_dict['hand']=='l':
            self.averaging_window_size = self.window_averaging_conrol(reading_dict) 
            print(self.averaging_window_size)

        # print(reading_dict)
        if self.volume_controller is not None:
            self.global_volume = self.volume_control(reading_dict)
            # print(self.global_volume)
            self.midi_writer.aftertouch(self.global_volume)#, channel=1)

        if self.pitch_bender is not None:
            pitchbend_val = self.pitch_bend(reading_dict) 
            self.midi_writer.pitch_bend(pitchbend_val)

    def run(self):
        self.reader.start_readers()
        self.initialize_triggers()
        
        while True:
            reading_dict = self.collect_q.get(block=True)
            self.instrument(reading_dict)

if __name__ == "__main__":
    root_note = 'C'
    scale = 'major'
    thresholds = {'flex': 180, 'press': 20}

    mods = ['imu']*4
    modidx = [2, 3, 4, 5]
    num_lh_fingers = 3 
    trigger_sensors = {'l': 'flex', 'r': 'flex'}
    mod_sensors = {'r':mods, 'l':mods} ## Make sure to use a list even if only one sensor
    mod_idx = {'r':modidx, 'l':modidx} # [yaw, pitch, roll] -> [0, 1, 2] ## TODO: assumes this order arduino-side must check.

    volume_controller = 'accel_mag'
    averaging_window_controller = 'imu2'
    pitch_bender = None 
    averaging_window_size = 600
    base_volume = 50
    instrument = 'ten_finger'

    app = MovingWindow(trigger_sensors=trigger_sensors, 
                        root_note=root_note, 
                        scale=scale, 
                        mod_sensors=mod_sensors, 
                        thresholds=thresholds,
                        mod_idx=mod_idx,
                        base_volume=base_volume,
                        volume_controller=volume_controller,
                        pitch_bender=pitch_bender,
                        averaging_window_size=averaging_window_size,
                        averaging_window_controller=averaging_window_controller,
                        instrument=instrument)
    app.start()

    key = input('press any key to finish \n')
    print('Shutting down...')

    app.reader.stop_readers()
    for hand in app.hands:
        app.triggers[hand].join(timeout=.1)
    app.join(timeout=.1)  
