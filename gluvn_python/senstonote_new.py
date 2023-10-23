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

ZERO_GYRIN = 32767.0
MAX_BEND = 2000
HIGH_JUMP_THRESHOLD = 100

#######################################################################################################################################
#######################################################################################################################################



    # def continuous_input(self):
    #     sensor_dict = self.sensor_q.get(block=True)
    #     n_switch = self.trigger_logic(sensor_dict['press'])
    #     reading_collect = [self.hand]
    #     for sensor in self.sensor:
    #         reading_collect.append(sensor_dict[sensor])

    #     switch = False
    #     if np.any(n_switch):
    #         switch = True

    #         # Get index of triggered, untriggered, and turned on sensors
    #         idx_triggered = np.where(n_switch==1)[0] #Assuming it returns one value
    #         idx_untriggered = np.where(n_switch==-1)[0] #Assuming it returns one value
    #         idx_turned_on = np.where(self.turn_state==1)[0]
    #         reading_collect += [idx_triggered, idx_untriggered, idx_turned_on]

    #         # If new sensor is triggered, turn it on, and turn off any turned on sensors
    #         if len(idx_triggered) > 0:
    #             self.turn_state[idx_triggered[0]] = 1
    #             if len(idx_turned_on) > 0:
    #                 self.turn_state[idx_turned_on[0]] = 0
            
    #         # If new sensor is untriggered and turned on, turn it off 
    #         if len(idx_untriggered) > 0 and self.turn_state[idx_untriggered[0]] == 1:
    #             self.turn_state[idx_untriggered[0]] = 0

    #     reading_collect += [switch]
    #     self.collect_q.put(reading_collect)



# class SensorToTrigger(Thread):
#     def __init__(self, hand, sensor_q, collect_q, 
#                  processing='trigger', sensor='press', thresh=180, hysteresis=5):
#         super().__init__()
#         self.hand = hand
#         self.sensor_q = sensor_q
#         self.sensor = sensor # a list if doing continuous reading
#         self.collect_q = collect_q
#         self.thresh = thresh
#         self.hysteresis = hysteresis
#         self.turn_state = np.zeros(5, dtype=bool)
#         self.trig_on = np.zeros(5, dtype=bool)
#         self.trig_off = np.zeros(5, dtype=bool)


#     def trigger_logic(self, sensor_values):
#         trigon_prev = self.trig_on
#         trigoff_prev = self.trig_off
#         sensarr = np.asarray(sensor_values) # Transforming to numpy array everytime might be inefficient
#         sdiff = sensarr - self.thresh        # subtract threshold from readings
#         trigon = sdiff - self.hysteresis > 0
#         trigoff = sdiff + self.hysteresis < 0 
#         turnon = np.logical_and(np.logical_and( trigon , np.logical_not( trigon_prev ) ) , np.logical_not(self.turn_state))
#         turnoff = np.logical_and(np.logical_and( trigoff , np.logical_not( trigoff_prev ) ) , self.turn_state )
#         n_switch = turnon.astype(int) - turnoff.astype(int) # Turn on if 1, Turn off if -1, No action if 0. - Also works 1*turnon - 1*turnoff
#         self.turn_state = n_switch + self.turn_state
#         self.trig_on, self.trig_off = trigon, trigoff

#         return n_switch

#     def run(self):
#         while True:
#             sensor_values = self.sensor_q.get(block=True)[self.sensor]
#             n_switch = self.trigger_logic(sensor_values)
#             # pre-process gyro/accel data
#             if np.any(n_switch): 
#                 state = [n_switch, self.hand]
#                 self.collect_q.put(state)



class SensorProcess(Thread):
    def __init__(self, 
                 hand, 
                 sensor_q, 
                 collect_q, 
                 trigger_sensor='flex', 
                 trigger_thresh=180, 
                 trigger_hysteresis=5,
                 mod_sensors=None,
                 mod_idx=None):

        super().__init__()
        self.hand = hand
        self.sensor_q = sensor_q
        self.trigger_sensor = trigger_sensor # a list if doing continuous reading
        self.mod_sensors = mod_sensors
        self.mod_idx = mod_idx
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
    
    def rescale_input(self, value, old_min, old_max, new_min, new_max):
        pass

    def run(self):
        send_dict = {'hand': self.hand}
        while True:
            # Collect reading queue
            sensor_dict = self.sensor_q.get(block=True)
            

            # Process trigger sensor
            n_switch = self.trigger_logic(sensor_dict[self.trigger_sensor])

            ## append modulation (contiuous) sensor data to reading_collect
            # Note: might be compared to passing entire list to be processed in app (?)
            mod_sensors_h = self.mod_sensors
            if mod_sensors_h is not None:
                for idx, sensor in enumerate(mod_sensors_h):
                    mod_sens_data = sensor_dict[sensor]
                    if self.mod_idx is not None: 
                        mod_sens_data = mod_sens_data[self.mod_idx[idx]]
                        sensor = sensor + str(self.mod_idx[idx])
                    send_dict[sensor] = mod_sens_data

            if np.any(n_switch) or (mod_sensors_h is not None and np.any(self.turn_state)): 
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
                 hysteresis=5):
        super().__init__()
        self.daemon = True
        self.hysteresis = hysteresis
        self.thresholds = thresholds or {'flex': 150, 'press': 15}
        self.sensor_config = sensor_config or {'r': {'flex': True, 'press': True, 'imu': True}}
        self.trigger_sensors = trigger_sensors or {'l': 'flex', 'r': 'press'}
        self.mod_sensors = mod_sensors or {'l': None, 'r': None}
        self.mod_idx = mod_idx or {'l': None, 'r': None}
        self.collect_q = queue.Queue(maxsize=20)
        self.hands = list(self.sensor_config.keys())
        self.reader = Reader(sensor_config=self.sensor_config)
        self.mapper = NoteMapper(root_note=root_note, scale=scale)
        self.midi_writer = MidiWriter()

    def initialize_triggers(self):
        triggers = {}
        for hand in self.hands:
            triggers[hand] = SensorProcess(
                hand, self.reader.threads[hand]['parser'].getQ(), self.collect_q, 
                trigger_sensor=self.trigger_sensors[hand], trigger_thresh=self.thresholds[self.trigger_sensors[hand]], 
                trigger_hysteresis=self.hysteresis, mod_sensors=self.mod_sensors[hand], mod_idx=self.mod_idx[hand]
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





class MovingWindowInstrument(BaseApp):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.window_trigger, self.note_windows = self.mapper.moving_window()
    
    def run(self):
        self.reader.start_readers()
        self.initialize_triggers()
        playing_notes = [None]*5
        note_array, note_array_idx = self.mapper.window_map([0, 0, 0, 0, 0], self.window_trigger, self.note_windows)
        
        while True:
            reading_dict = self.collect_q.get(block=True)
            print(reading_dict)
            if reading_dict['hand'] == 'l':
                temp = self.mapper.window_map(self.triggers['l'].turn_state, self.window_trigger, self.note_windows)
                if temp is not None:
                    note_array, note_array_idx = temp
            else:
                playing_notes = self.midi_writer.trig_notes_array_playing(reading_dict['switch'], playing_notes, note_array)




class MovingWindowContinous(BaseApp):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.window_trigger, self.note_windows = self.mapper.moving_window()

    def run(self):
        self.reader.start_readers()
        self.initialize_triggers()
        playing_notes = [None]*5
        note_array, note_array_idx = self.mapper.window_map([0, 0, 0, 0, 0], self.window_trigger, self.note_windows)
        
        while True:
            reading_dict = self.collect_q.get(block=True)
            if reading_dict['hand'] == 'l':
                temp = self.mapper.window_map(self.triggers['l'].turn_state, self.window_trigger, self.note_windows)
                if temp is not None:
                    note_array, note_array_idx = temp
            else:
                playing_notes = self.midi_writer.trig_notes_array_playing(reading_dict['switch'], playing_notes, note_array)


##############################################################################
##############################################################################

if __name__=="__main__":

    # sensor_config={'r': {'flex': False, 'press': True, 'imu': False},
    #                'l': {'flex': True, 'press': False, 'imu': False}}
    # app = BaseApp(sensor_config=sensor_config)
    # app.start()

    # key = input('press any key to finish \n')
    # print('Shutting down...')

    # app.reader.stop_readers()
    # for hand in app.hands:
    #     app.triggers[hand].join(timeout=.1)
    # app.join(timeout=.1)  

    root_note = 'D'
    scale = 'major'
    sensor_config={'r': {'flex': True, 'press': False, 'imu': False},
                   'l': {'flex': True, 'press': False, 'imu': False}}
    

    trigger_sensors = {'l': 'flex', 'r': 'flex'}
    mod_sensors = {'r':['flex'], 'l':None} ## Make sure to use a list even if only one sensor
    app = MovingWindowInstrument(sensor_config=sensor_config, trigger_sensors=trigger_sensors, 
                                 root_note=root_note, scale=scale, mod_sensors=mod_sensors)
    app.start()

    key = input('press any key to finish \n')
    print('Shutting down...')

    app.reader.stop_readers()
    for hand in app.hands:
        app.triggers[hand].join(timeout=.1)
    app.join(timeout=.1)  