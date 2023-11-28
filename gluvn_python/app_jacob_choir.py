
import numpy as np
# from code.gluvn_python.midi_writer import TrigNote, TrigNote_midinum, signswitch2note, TriggerChordTest, make_C2midi
from __init__ import settingsDir
from learning import Learn
from threading import Thread
from collections import deque
from mapper import NoteMapper
from port_read import Reader
from midi_writer import MidiWriter
# from senstonote_new import BaseApp
import queue

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
                 trigger_sensor=['flex'], 
                 trigger_thresh={'flex':180}, 
                 trigger_hysteresis={'flex':5},
                 mod_sensors=None,
                 mod_idx=None,
                 send_all_data=False):

        super().__init__()
        self.hand = hand
        self.sensor_q = sensor_q
        self.trigger_sensors = trigger_sensor # a list if doing continuous reading
        self.mod_sensors = mod_sensors
        self.mod_idx = mod_idx
        self.send_all_data = send_all_data
        self.collect_q = collect_q

        self.trigger_thresh = {}
        self.trigger_hysteresis = {}
        self.turn_state = {}
        self.trig_on = {}
        self.trig_off = {}
        self.switch = {}
        for trigger_sensor in self.trigger_sensors:
            self.trigger_thresh[trigger_sensor] = trigger_thresh[trigger_sensor]
            self.trigger_hysteresis[trigger_sensor] = trigger_hysteresis[trigger_sensor]
            self.turn_state[trigger_sensor] = np.zeros(5, dtype=bool)
            self.trig_on[trigger_sensor] = np.zeros(5, dtype=bool)
            self.trig_off[trigger_sensor] = np.zeros(5, dtype=bool)
            self.switch[trigger_sensor] = np.zeros(5, dtype=bool)


    def trigger_logic(self, sensor_values, trig_sens):
        trigon_prev = self.trig_on[trig_sens]
        trigoff_prev = self.trig_off[trig_sens]
        sensarr = np.asarray(sensor_values[trig_sens]) # Transforming to numpy array everytime might be inefficient
        sdiff = sensarr - self.trigger_thresh[trig_sens]        # subtract threshold from readings
        trigon = sdiff - self.trigger_hysteresis[trig_sens] > 0
        trigoff = sdiff + self.trigger_hysteresis[trig_sens] < 0 
        turnon = np.logical_and(np.logical_and( trigon , np.logical_not( trigon_prev ) ) , np.logical_not(self.turn_state[trig_sens]))
        turnoff = np.logical_and(np.logical_and( trigoff , np.logical_not( trigoff_prev ) ) , self.turn_state[trig_sens] )
        n_switch = turnon.astype(int) - turnoff.astype(int) # Turn on if 1, Turn off if -1, No action if 0. - Also works 1*turnon - 1*turnoff
        self.turn_state[trig_sens] = n_switch + self.turn_state[trig_sens]
        self.trig_on[trig_sens], self.trig_off[trig_sens], self.switch[trig_sens] = trigon, trigoff, n_switch

        return n_switch


    def run(self):
        send_dict = {'hand': self.hand}
        while True:
            # Collect reading queue
            sensor_dict = self.sensor_q.get(block=True)
            
            if self.send_all_data:
                for trig_sens in self.trigger_sensors:
                    n_switch = self.trigger_logic(sensor_dict, trig_sens)
                sensor_dict['hand'] = self.hand
                self.collect_q.put(sensor_dict)
            
            # else:
            #     ### STILL ASSUMES trigger_sensors is a string
            #     n_switch = self.trigger_logic(sensor_dict[self.trigger_sensors])
            #     ## append modulation (contiuous) sensor data to reading_collect
            #     # Note: might be compared to passing entire list to be processed in app (?)
            #     mod_sensors_h = self.mod_sensors
            #     if mod_sensors_h[0] is not None: ## Fix: breaks if mod_sensors is None
            #         for idx, sensor in enumerate(mod_sensors_h):
            #             mod_sens_data = sensor_dict[sensor]
            #             if self.mod_idx is not None: 
            #                 mod_sens_data = mod_sens_data[self.mod_idx[idx]]
            #                 sensor = sensor + str(self.mod_idx[idx])
            #             send_dict[sensor] = mod_sens_data

            #     ## might be inefficient 

            #     if (np.any(n_switch) or (mod_sensors_h is not None and np.any(self.turn_state))): 
            #         send_dict['switch'] = n_switch
            #         self.collect_q.put(send_dict)



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
                 hysteresis=None):
        super().__init__()
        self.daemon = True
        self.hysteresis = hysteresis or {'flex': 5, 'press': 5}
        self.thresholds = thresholds or {'flex': 200, 'press': 15}
        self.trigger_sensors = trigger_sensors or {'l': ['flex'], 'r': ['press']}
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
                trigger_sensor=self.trigger_sensors[hand], trigger_thresh=self.thresholds, 
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
        # self.window_trigger, self.note_windows = self.mapper.moving_window(num_lhf=num_lh_fingers, num_rhf=num_rh_fingers)
        self.num_lh_fingers = num_lh_fingers 
        self.num_rh_fingers = num_rh_fingers 
        self.playing_notes = [None]*num_rh_fingers
        self.volume_controller = volume_controller
        self.pitch_bender = pitch_bender
        self.averaging_queue = {'r': deque(maxlen=avg_window_size), 'l': deque(maxlen=avg_window_size)}
        self.base_volume = base_volume
        
        self.pitch_bend_limit = 8192
        self.global_volume = 60
        self.note_array = None

    def input_scaling(self, input, min_output=0, max_output=127, shift=0, min_input=0, max_input=BYTE):
        output = int( min_output + (max_output - min_output) * ((input + shift) - min_input) / (max_input - min_input))
        return min(output, max_output)

    def scaling_modulo(self, input, min_output=0, max_output=127, min_input=0, max_input=BYTE):
        return int((( max_output + (max_output - min_output) * (input - min_input)/(max_input - min_input)) % (max_output - min_output)) - max_output)
    
    def shift_modulo(self, input, shift, range=128):
        return int((input + shift) % range)

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
        
        if self.volume_controller == 'press_pitch':
            # If that pressure sensor is pressed, track pitch and increment/decrease volume accordingly.
            pass

    def single_trigger_logic(self, sensor_value, trig_on, trig_off, turn_state, trigger_thresh, trigger_hysteresis):
        trigon_prev = trig_on
        trigoff_prev = trig_off
        sdiff = sensor_value - trigger_thresh        # subtract threshold from readings
        trigon = sdiff - trigger_hysteresis > 0
        trigoff = sdiff + trigger_hysteresis < 0 
        turnon = np.logical_and(np.logical_and( trigon , np.logical_not( trigon_prev ) ) , np.logical_not(turn_state))
        turnoff = np.logical_and(np.logical_and( trigoff , np.logical_not( trigoff_prev ) ) , turn_state )
        n_switch = turnon.astype(int) - turnoff.astype(int) # Turn on if 1, Turn off if -1, No action if 0. - Also works 1*turnon - 1*turnoff
        turn_state = n_switch + turn_state

        return trigon, trigoff, turn_state, n_switch

    def wrap_around_count(self, prev, curr, threshold=100):
        if np.abs(curr - prev) > threshold:
            return np.sign(prev - curr)
        else:
            return 0

    def run(self):
        self.reader.start_readers()
        self.initialize_triggers()
        # self.note_array, note_array_idx = self.mapper.window_map([0]*self.num_lh_fingers, self.window_trigger, self.note_windows)
        notemaps = self.mapper.basic_map_2hands()  # Assumes a method that maps notes for both hands
        
        accel_trigger_thresh = 110 
        accel_trigger_hysteresis = 10
        roll_trigger_thresh_range = 32 
        roll_trigger_hysteresis = 5
      
        accel_trig_on = {'r': 0, 'l': 0}
        accel_trig_off = {'r': 0, 'l': 0}
        accel_turn_state = {'r': 0, 'l': 0}
        scaled_accel = {'r': 0, 'l': 0}
        accel_switch = {'r': 0, 'l': 0}

        pitch_trigger_hysteresis = 5  # adjust this value as per your requirement
        yaw_window = 10

        scaled_yaw = {'r': 0, 'l': 0}
        scaled_yaw_reading = {'r': 0, 'l': 0}
        scaled_yaw_prev = {'r': 0, 'l': 0}
        num_revolution_yaw = {'r': 0, 'l': 0}

        pitch_trig_on = {'r': 0, 'l': 0}
        pitch_trig_off = {'r': 0, 'l': 0}
        pitch_turn_state = {'r': 0, 'l': 0}
        pitch_trigger_thresh = {'r': 0, 'l': 0}
        scaled_pitch = {'r': 0, 'l': 0}
        scaled_pitch_reading = {'r': 0, 'l': 0}
        pitch_switch = {'r': 0, 'l': 0}
        scaled_pitch_prev = {'r': 0, 'l': 0}
        scaled_pitch_change = {'r': 0, 'l': 0}
        num_revolution_pitch = {'r': 0, 'l': 0}
        
        roll_trig_on = {'r': 0, 'l': 0}
        roll_trig_off = {'r': 0, 'l': 0}
        roll_turn_state = {'r': 0, 'l': 0}
        scaled_roll = {'r': 0, 'l': 0}
        scaled_roll_reading = {'r': 0, 'l': 0}
        roll_trigger_thresh = {'r': 0, 'l': 0}
        roll_switch = {'r': 0, 'l': 0}
        scaled_roll_prev = {'r': 0, 'l': 0}
        num_revolution_roll = {'r': 0, 'l': 0}

        yaw0 = {'r': 0, 'l': 0}
        pitch0 = {'r': 0, 'l': 0}
        roll0 = {'r': 0, 'l': 0}
        yaw0_list = {'r': [], 'l': []}
        pitch0_list = {'r': [], 'l': []}
        roll0_list = {'r': [], 'l': []}
        yaw_trigger_thresh = {'r': [], 'l': []}

        # Initialize reference roll pitch and yaw
        hand = None
        sens_count = 0
        print('Initializing reference roll pitch and yaw')
        while sens_count < 500:
            reading_dict = self.collect_q.get(block=True) 
            hand = reading_dict['hand']
            yaw0_list[hand].append( self.input_scaling(reading_dict['imu'][0], max_output=127, min_input=0, max_input=TWO_BYTE) )
            pitch0_list[hand].append( self.input_scaling(reading_dict['imu'][1], max_output=127, min_input=0, max_input=TWO_BYTE) )
            roll0_list[hand].append( self.input_scaling(reading_dict['imu'][2], max_output=127, min_input=0, max_input=TWO_BYTE) )
            sens_count += 1
        
        for hand in self.hands:
            yaw0[hand] = int(sum(yaw0_list[hand])/len(yaw0_list[hand]))
            pitch0[hand] = int(sum(pitch0_list[hand])/len(pitch0_list[hand]))
            roll0[hand] = int(sum(roll0_list[hand])/len(roll0_list[hand]))
            yaw_trigger_thresh[hand] = [yaw0[hand] - yaw_window, yaw0[hand] + yaw_window]  # adjust this value as per your requirement
            pitch_trigger_thresh[hand] = pitch0[hand] 

        roll_trigger_thresh['r'] = roll0['r'] + roll_trigger_thresh_range
        roll_trigger_thresh['l'] = roll0['l'] - roll_trigger_thresh_range

        # Main loop
        while True:
            # Get sensor data from queue
            reading_dict = self.collect_q.get(block=True)
            hand = reading_dict['hand']

            ## Compute acceleration magnitude and scale it 
            ## TODO: double check calibration
            accel_norm = np.sqrt((reading_dict['imu'][3]-TWO_BYTE/2.0)**2 + (reading_dict['imu'][4]-TWO_BYTE/2.0)**2 + (reading_dict['imu'][5]-TWO_BYTE/2.0)**2 )
            accel_norm = max(accel_norm - ZERO_ACCEL, 0) 
            self.averaging_queue[hand].append(accel_norm)
            scaled_accel[hand] = self.input_scaling(np.mean(self.averaging_queue[hand]), max_output=127, min_input=0, max_input=15000)

            # scale IMU readings 
            scaled_roll_reading[hand] = self.input_scaling(reading_dict['imu'][2], max_output=127, min_input=0, max_input=TWO_BYTE) 
            scaled_pitch_reading[hand] = self.input_scaling(reading_dict['imu'][1], max_output=127, min_input=0, max_input=TWO_BYTE) 
            scaled_yaw_reading[hand] = self.input_scaling(reading_dict['imu'][0], max_output=127, min_input=0, max_input=TWO_BYTE) 

            # Computer number of revolutions wrap around
            num_revolution_roll[hand] += self.wrap_around_count(scaled_roll_prev[hand], scaled_roll_reading[hand], threshold=100)
            num_revolution_pitch[hand] += self.wrap_around_count(scaled_pitch_prev[hand], scaled_pitch_reading[hand], threshold=100)
            num_revolution_yaw[hand] += self.wrap_around_count(scaled_yaw_prev[hand], scaled_yaw_reading[hand], threshold=100)

            # shift roll pitch yaw based on number of revolutions
            scaled_roll[hand] = scaled_roll_reading[hand] + num_revolution_roll[hand] * 127
            scaled_pitch[hand] = scaled_pitch_reading[hand] + num_revolution_pitch[hand] * 127
            scaled_yaw[hand] = scaled_yaw_reading[hand] + num_revolution_yaw[hand] * 127

            scaled_pitch_change[hand] = scaled_pitch[hand] - scaled_pitch_prev[hand] + num_revolution_pitch[hand] * 127
                
            scaled_roll_prev[hand] = scaled_roll_reading[hand]
            scaled_pitch_prev[hand] = scaled_pitch_reading[hand]
            scaled_yaw_prev[hand] = scaled_yaw_reading[hand]


            accel_trig_on[hand], accel_trig_off[hand], accel_turn_state[hand], accel_switch[hand] = \
                self.single_trigger_logic(scaled_accel[hand], accel_trig_on[hand], accel_trig_off[hand], accel_turn_state[hand], accel_trigger_thresh, accel_trigger_hysteresis)

            roll_trig_on[hand], roll_trig_off[hand], roll_turn_state[hand], roll_switch[hand] = \
                self.single_trigger_logic(scaled_roll[hand], roll_trig_on[hand], roll_trig_off[hand], roll_turn_state[hand], roll_trigger_thresh[hand], roll_trigger_hysteresis)

            pitch_trig_on[hand], pitch_trig_off[hand], pitch_turn_state[hand], pitch_switch[hand] = \
                self.single_trigger_logic(scaled_pitch[hand], pitch_trig_on[hand], pitch_trig_off[hand], pitch_turn_state[hand], pitch_trigger_thresh[hand], pitch_trigger_hysteresis)


            # Fix to use both hands for calculation of acceleration
            ## Trigger notes
            if accel_switch[hand]==1:
                print('Triggering Notes..')
                print(self.triggers[hand].turn_state)

                # reset reference yaw0
                for hand in self.hands:
                    yaw0[hand] = scaled_yaw[hand]
                    yaw_trigger_thresh[hand] = [yaw0[hand] - yaw_window, yaw0[hand] + yaw_window]  

                print('yaw windows: ', yaw_trigger_thresh)

                # Turn off playing notes
                self.playing_notes = self.midi_writer.turn_off_all_playing(self.playing_notes)

                # Trigger first three notes in hand 
                def trig_notes():
                    playing_note_count = 0
                    for hand in self.hands[::-1]:
                        for i, turn_state_finger in enumerate(self.triggers[hand].turn_state['flex']):
                            if turn_state_finger:
                                self.midi_writer.trig_note(notemaps[hand][i], vel=self.global_volume)
                                self.playing_notes[playing_note_count] = notemaps[hand][i]
                                playing_note_count += 1

                            # If too many playing notes
                            if playing_note_count >= 3:
                                return   
                trig_notes()

                print('playing notes: ', self.playing_notes)

            ## Track pitch when playing note finger is on


            # Turn off playing notes
            if self.triggers[hand].switch['press'][4]==1:
                print('Turning off notes..')
                self.playing_notes = self.midi_writer.turn_off_all_playing(self.playing_notes)

            # Javob switch: change note based on yaw, pitch and roll
            if pitch_switch[hand] != 0:
                
                # locate note
                note_idx = -1 
                for i in range(len(yaw_trigger_thresh[hand])):
                    if scaled_yaw[hand] < yaw_trigger_thresh[hand][i]:
                        note_idx = i
                        print('yaw sector idx: ', note_idx)
                        break 
                
                # Make sure roll is in the right direction
                # move_palm_dir = (roll_turn_state[hand] == max(pitch_switch[hand], 0)) # unused and will depend on yaw
                switch_condition = self.triggers[hand].turn_state['press'][2] and self.playing_notes[note_idx] is not None

                if switch_condition:
                    # Turn previous note off
                    self.midi_writer.trig_note(self.playing_notes[note_idx], 0)
                    
                    # Get new note
                    new_note_idx = self.mapper.notes_in_scale.index(self.playing_notes[note_idx]) + pitch_switch[hand]
                    new_note = self.mapper.notes_in_scale[new_note_idx]

                    # Turn new note on
                    self.midi_writer.trig_note(new_note, vel=self.global_volume) 

                    self.playing_notes[note_idx] = new_note
                
                    print('##################')
                    print(self.playing_notes)
                    print('##################')

            # Change volume if 4th finger is pressed
            if self.triggers[hand].turn_state['press'][3]==1:
                self.global_volume += int(scaled_pitch_change[hand])
                if self.global_volume > 127: self.global_volume = 127
                if self.global_volume < 0: self.global_volume = 0
                print('changing volume', scaled_pitch_change[hand], self.global_volume)
                self.midi_writer.aftertouch(self.global_volume) 


            # If pitch is switching
            # if pitch_switch['r'] != 0: 
            #     print('pitch: ', pitch_switch)
            #     print('yaw threshold: ', yaw_trigger_thresh)
            #     print('yaw: ', scaled_yaw)
            #     print('roll: ', scaled_roll)
            #     print('roll turn: ', roll_turn_state)
            #     print('----')

            # loopcount += 1
            # if loopcount % 200 == 0:
            #     print('yaw: ', scaled_yaw)
            #     print('rol: ', scaled_roll)
            #     print('pit: ', scaled_pitch)
            #     print('-----------')

            # print(scaled_yaw)


if __name__ == "__main__":

    root_note = 'D'
    scale = 'minor'

    num_rh_fingers = 3 
    trigger_sensors = {'l': ['press', 'flex'], 'r': ['press', 'flex']}
    thresholds = {'press': 18, 'flex': 180}
    histeresis = {'press': 4, 'flex': 20}

    # mod_sensors = {'r':['imu', 'imu', 'imu'], 'l':[None]} ## Make sure to use a list even if only one sensor
    # mod_idx = {'r':[3, 4, 5], 'l':None} # [yaw, pitch, roll] -> [0, 1, 2] ## TODO: assumes this order arduino-side must check.
    send_all_data = True

    volume_controller = 'press_pitch'
    pitch_bender = None 
    avg_window_size = 10

    app = MovingWindow(root_note=root_note, 
                        scale=scale, 
                        volume_controller=volume_controller,
                        pitch_bender=pitch_bender,
                        num_rh_fingers=num_rh_fingers,
                        avg_window_size=avg_window_size,
                        send_all_data=send_all_data,
                        trigger_sensors=trigger_sensors,
                        thresholds=thresholds)
    app.start()

    key = input('press any key to finish \n')
    print('Shutting down...')

    app.reader.stop_readers()
    for hand in app.hands:
        app.triggers[hand].join(timeout=.1)
    app.join(timeout=.1)  

