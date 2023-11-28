'''
Created on May 20, 2016

@author: josephbakarji
'''

import numpy as np
# from code.gluvn_python.midi_writer import TrigNote, TrigNote_midinum, signswitch2note, TriggerChordTest, make_C2midi
from __init__ import settingsDir
from learning import Learn
from threading import Thread
from collections import deque
from mapper import NoteMapper
from port_read import Reader
from midi_writer import MidiWriter
import queue

# Constants

TWO_BYTE = 65535
BYTE = 255
ZERO_GYRIN = 32767.0
MAX_BEND = 2000
ZERO_ACCEL = TWO_BYTE / 4.0 - 680.0


class SensorProcess(Thread):
    def __init__(self, 
                 hand, 
                 sensor_q, 
                 collect_q, 
                 trigger_sensor='flex', 
                 trigger_thresh=180, 
                 trigger_hysteresis=5,
                 mod_sensors=None,
                 mod_idx=None,
                 send_all_data=False):

        super().__init__()
        self.hand = hand
        self.sensor_q = sensor_q
        self.trigger_sensor = trigger_sensor # a list if doing continuous reading
        self.mod_sensors = mod_sensors
        self.mod_idx = mod_idx
        self.send_all_data = send_all_data
        self.collect_q = collect_q
        self.trigger_thresh = trigger_thresh
        self.trigger_hysteresis = trigger_hysteresis
        self.turn_state = np.zeros(5, dtype=bool)
        self.trig_on = np.zeros(5, dtype=bool)
        self.trig_off = np.zeros(5, dtype=bool)


    def trigger_logic(self, sensor_values):
        trigon_prev = self.trig_on
        trigoff_prev = self.trig_off
        sensarr = np.asarray(sensor_values) # Transforming to numpy array everytime might be inefficient
        sdiff = sensarr - self.trigger_thresh        # subtract threshold from readings
        trigon = sdiff - self.trigger_hysteresis > 0
        trigoff = sdiff + self.trigger_hysteresis < 0 
        turnon = np.logical_and(np.logical_and( trigon , np.logical_not( trigon_prev ) ) , np.logical_not(self.turn_state))
        turnoff = np.logical_and(np.logical_and( trigoff , np.logical_not( trigoff_prev ) ) , self.turn_state )
        n_switch = turnon.astype(int) - turnoff.astype(int) # Turn on if 1, Turn off if -1, No action if 0. - Also works 1*turnon - 1*turnoff
        self.turn_state = n_switch + self.turn_state
        self.trig_on, self.trig_off = trigon, trigoff

        return n_switch


    def run(self):
        send_dict = {'hand': self.hand}
        while True:
            # Collect reading queue
            sensor_dict = self.sensor_q.get(block=True)
            
            if self.send_all_data:
                ## NOTE: Can't do triggering for multiple tigger sensors at a time (see app_jacob_choir)
                sensor_dict['hand'] = self.hand
                self.collect_q.put(sensor_dict)
            
            else:

                n_switch = self.trigger_logic(sensor_dict[self.trigger_sensor])
                ## append modulation (contiuous) sensor data to reading_collect
                # Note: might be compared to passing entire list to be processed in app (?)
                mod_sensors_h = self.mod_sensors
                if mod_sensors_h[0] is not None: ## Fix: breaks if mod_sensors is None
                    for idx, sensor in enumerate(mod_sensors_h):
                        mod_sens_data = sensor_dict[sensor]
                        if self.mod_idx is not None: 
                            mod_sens_data = mod_sens_data[self.mod_idx[idx]]
                            sensor = sensor + str(self.mod_idx[idx])
                        send_dict[sensor] = mod_sens_data

                ## might be inefficient 

                if (np.any(n_switch) or (mod_sensors_h is not None and np.any(self.turn_state))): 
                    send_dict['switch'] = n_switch
                    self.collect_q.put(send_dict)



class BaseApp(Thread):
    def __init__(self, 
                 root_note='D', 
                 scale='minor',
                 sensor_config=None, 
                 thresholds=None, 
                 trigger_sensors=None, 
                 mod_sensors=None,
                 mod_idx=None,
                 send_all_data=False,
                 hands=['r', 'l'],
                 hysteresis=5):
        super().__init__()
        self.daemon = True
        self.hysteresis = hysteresis
        self.thresholds = thresholds or {'flex': 200, 'press': 15}
        self.trigger_sensors = trigger_sensors or {'l': 'flex', 'r': 'press'}
        self.mod_sensors = mod_sensors or {'l': [None], 'r': [None]}
        self.mod_idx = mod_idx or {'l': [None], 'r': [None]}
        self.send_all_data = send_all_data
        self.sensor_config = self._get_sensor_config(sensor_config) 
        self.collect_q = queue.Queue(maxsize=20)
        self.hands = hands 
        self.reader = Reader(sensor_config=self.sensor_config)
        self.mapper = NoteMapper(root_note=root_note, scale=scale)
        self.midi_writer = MidiWriter()

    def _get_sensor_config(self, sensor_config):
        if self.send_all_data: 
            sensor_config={'r': {'flex': True, 'press': True, 'imu': True},
                        'l': {'flex': True, 'press': True, 'imu': True}}

        elif sensor_config is None:
            sensor_config={'r': {'flex': False, 'press': False, 'imu': False},
                        'l': {'flex': False, 'press': False, 'imu': False}}
            for hand in ['r', 'l']:
                for sensor in sensor_config[hand].keys():
                    if sensor in self.mod_sensors[hand]:
                            sensor_config[hand][sensor] = True
                    if sensor == self.trigger_sensors[hand]:
                        sensor_config[hand][sensor] = True


        return sensor_config

    def initialize_triggers(self):
        triggers = {}
        for hand in self.hands:
            triggers[hand] = SensorProcess(
                hand, self.reader.threads[hand]['parser'].getQ(), self.collect_q, 
                trigger_sensor=self.trigger_sensors[hand], trigger_thresh=self.thresholds[self.trigger_sensors[hand]], 
                trigger_hysteresis=self.hysteresis, mod_sensors=self.mod_sensors[hand], mod_idx=self.mod_idx[hand], 
                send_all_data=self.send_all_data
            )
            triggers[hand].start()
        self.triggers = triggers

    def run(self):
        self.reader.start_readers()
        notemaps = self.mapper.basic_map_2hands()  # Assumes a method that maps notes for both hands
        self.initialize_triggers()


        while True: 
            reading_dict = self.collect_q.get(block=True)
            self.midi_writer.trig_note_array(reading_dict['switch'], notemaps[reading_dict['hand']])



class MovingWindow(BaseApp):
    def __init__(self, *args, 
                 volume_controller=None, 
                 pitch_bender=None, 
                 num_lh_fingers=5, 
                 num_rh_fingers=5, 
                 avg_window_size=10,
                 base_volume=20,
                 **kwargs):
        super().__init__(*args, **kwargs)
        self.window_trigger, self.note_windows = self.mapper.moving_window(num_lhf=num_lh_fingers, num_rhf=num_rh_fingers)
        self.num_lh_fingers = num_lh_fingers 
        self.num_rh_fingers = num_rh_fingers 
        self.playing_notes = [None]*num_rh_fingers
        self.volume_controller = volume_controller
        self.pitch_bender = pitch_bender
        self.averaging_queue = deque(maxlen=avg_window_size)
        self.base_volume = base_volume
        
        self.pitch_bend_limit = 8192
        self.global_volume = 10
        self.note_array = None

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
            return self.base_volume + self.input_scaling(np.mean(self.averaging_queue), max_output=127-self.base_volume, min_input=0, max_input=15000)
            

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


    def run(self):
        self.reader.start_readers()
        self.initialize_triggers()
        self.note_array, note_array_idx = self.mapper.window_map([0]*self.num_lh_fingers, self.window_trigger, self.note_windows)
        
        
        while True:
            reading_dict = self.collect_q.get(block=True)
            self.double_flex_instrument(reading_dict)


##############################################################################
##############################################################################

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