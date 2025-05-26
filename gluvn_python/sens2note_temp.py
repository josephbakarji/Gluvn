'''
Created on May 20, 2016
Updated April 2024

@author: josephbakarji
'''

import numpy as np
from __init__ import settingsDir
from gluvn_python.legacy.learning import Learn
from threading import Thread
from collections import deque
from mapper import NoteMapper
from port_read import Reader
from midi_writer import MidiWriter
import queue
from itertools import islice, repeat

# Import constants from base config
from configs.base_config import TWO_BYTE, BYTE, ZERO_GYRIN, MAX_BEND, ZERO_ACCEL

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
        """Implement hysteresis-based triggering logic"""
        trigon_prev = self.trig_on
        trigoff_prev = self.trig_off
        sensarr = np.asarray(sensor_values)
        sdiff = sensarr - self.trigger_thresh        # subtract threshold from readings
        trigon = sdiff - self.trigger_hysteresis > 0
        trigoff = sdiff + self.trigger_hysteresis < 0 
        turnon = np.logical_and(np.logical_and(trigon, np.logical_not(trigon_prev)), np.logical_not(self.turn_state))
        turnoff = np.logical_and(np.logical_and(trigoff, np.logical_not(trigoff_prev)), self.turn_state)
        n_switch = turnon.astype(int) - turnoff.astype(int) # Turn on if 1, Turn off if -1, No action if 0
        self.turn_state = n_switch + self.turn_state
        self.trig_on, self.trig_off = trigon, trigoff

        return n_switch


    def run(self):
        """Process sensor data and apply trigger logic"""
        send_dict = {'hand': self.hand}
        while True:
            # Collect reading queue
            sensor_dict = self.sensor_q.get(block=True)
            
            if self.send_all_data:
                # Send all data without processing
                sensor_dict['hand'] = self.hand
                self.collect_q.put(sensor_dict)
            
            else:
                # Process trigger logic
                n_switch = self.trigger_logic(sensor_dict[self.trigger_sensor])
                
                # Append modulation (continuous) sensor data to reading_collect
                mod_sensors_h = self.mod_sensors
                if mod_sensors_h and mod_sensors_h[0] is not None:
                    for idx, sensor in enumerate(mod_sensors_h):
                        mod_sens_data = sensor_dict[sensor]
                        if self.mod_idx is not None and self.mod_idx[idx] is not None: 
                            mod_sens_data = mod_sens_data[self.mod_idx[idx]]
                            sensor = sensor + str(self.mod_idx[idx])
                        send_dict[sensor] = mod_sens_data

                # Send data if there's a trigger or if we're in a triggered state
                if (np.any(n_switch) or (mod_sensors_h is not None and np.any(self.turn_state))): 
                    send_dict['switch'] = n_switch
                    self.collect_q.put(send_dict.copy())  # Use copy to avoid reference issues



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
        
        # Debug flag
        self.debug = False

    def _get_sensor_config(self, sensor_config):
        """Generate sensor configuration based on the app's needs"""
        if self.send_all_data: 
            # Enable all sensors if sending all data
            sensor_config = {
                'r': {'flex': True, 'press': True, 'imu': True},
                'l': {'flex': True, 'press': True, 'imu': True}
            }
        elif sensor_config is None:
            # Enable only the sensors needed for triggers and modulation
            sensor_config = {
                'r': {'flex': False, 'press': False, 'imu': False},
                'l': {'flex': False, 'press': False, 'imu': False}
            }
            for hand in ['r', 'l']:
                if hand in self.mod_sensors and self.mod_sensors[hand]:
                    for sensor in self.mod_sensors[hand]:
                        if sensor and sensor in sensor_config[hand]:
                            sensor_config[hand][sensor] = True
                            
                if hand in self.trigger_sensors:
                    trigger = self.trigger_sensors[hand]
                    if trigger and trigger in sensor_config[hand]:
                        sensor_config[hand][trigger] = True

        return sensor_config

    def initialize_triggers(self):
        """Create and start sensor processing threads for each hand"""
        triggers = {}
        for hand in self.hands:
            if hand not in self.trigger_sensors:
                continue
                
            trigger_sensor = self.trigger_sensors[hand]
            if not trigger_sensor:
                continue
                
            if self.debug:
                print(f"Initializing trigger for {hand} hand using {trigger_sensor}")
                
            triggers[hand] = SensorProcess(
                hand, 
                self.reader.threads[hand]['parser'].getQ(), 
                self.collect_q, 
                trigger_sensor=trigger_sensor,
                trigger_thresh=self.thresholds[trigger_sensor], 
                trigger_hysteresis=self.hysteresis[trigger_sensor], 
                mod_sensors=self.mod_sensors[hand], 
                mod_idx=self.mod_idx[hand], 
                send_all_data=self.send_all_data
            )
            triggers[hand].start()
        self.triggers = triggers

    def run(self):
        """Main application loop - start reader, create note maps, handle triggers"""
        if self.debug:
            print("Starting BaseApp")
            print(f"Sensor config: {self.sensor_config}")
            
        self.reader.start_readers()
        notemaps = self.mapper.basic_map_2hands()
        self.initialize_triggers()

        while True: 
            reading_dict = self.collect_q.get(block=True)
            hand = reading_dict.get('hand')
            if 'switch' in reading_dict and hand in notemaps:
                self.midi_writer.trig_note_array(reading_dict['switch'], notemaps[hand])



class MovingWindow(BaseApp):
    def __init__(self, *args, 
                 volume_controller=None, 
                 pitch_bender=None, 
                 num_lh_fingers=5, 
                 num_rh_fingers=5, 
                 averaging_window_size=10,
                 base_volume=20,
                 instrument='double_flex',
                 averaging_window_controller=None,
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
        self.instrument = instrument
        
        self.pitch_bend_limit = 8192
        self.global_volume = base_volume
        self.averaging_window_size = self.averaging_window_size_max
        self.note_array = None
        
        # Initialize note windows based on instrument type
        if self.instrument == 'double_flex':
            # Double flex instrument uses left hand fingers to control note windows
            self.window_trigger, self.note_windows = self.mapper.moving_window(
                num_lhf=num_lh_fingers, 
                num_rhf=num_rh_fingers
            )
            self.note_array, note_array_idx = self.mapper.window_map(
                [0]*self.num_lh_fingers, 
                self.window_trigger, 
                self.note_windows
            )
        elif self.instrument == 'ten_finger':
            # Ten finger instrument maps each finger to a note
            self.note_windows = self.mapper.basic_map_2hands()
        else:
            raise ValueError('Instrument not recognized. Please choose from double_flex or ten_finger')

    def input_scaling(self, input, min_output=0, max_output=127, shift=0, min_input=0, max_input=BYTE):
        """Scale an input value to the specified output range"""
        output = int(min_output + (max_output - min_output) * ((input + shift) - min_input) / (max_input - min_input))
        return min(max(output, min_output), max_output)  # Ensure output is within bounds

    def scaling_modulo(self, input, min_output=0, max_output=127, min_input=0, max_input=BYTE):
        """Scale and wrap an input value within the specified output range"""
        return int(((max_output + (max_output - min_output) * (input - min_input)/(max_input - min_input)) % (max_output - min_output)) - max_output)

    def pitch_bend(self, reading_dict):
        """Calculate pitch bend value based on sensor readings"""
        if not self.pitch_bender or self.pitch_bender not in reading_dict:
            return 0
            
        # Use modulo scaling for pitch bend to allow wrapping around the full range
        if self.pitch_bender == 'imu2':
            return self.scaling_modulo(reading_dict[self.pitch_bender], 
                                      min_output=-self.pitch_bend_limit, 
                                      max_output=self.pitch_bend_limit, 
                                      min_input=0, 
                                      max_input=TWO_BYTE)
        return 0

    def volume_control(self, reading_dict):
        """Calculate volume based on sensor readings"""
        if not self.volume_controller:
            return self.base_volume
            
        # Basic volume control using a single sensor
        if self.volume_controller == 'imu1' and self.volume_controller in reading_dict:
            return self.input_scaling(reading_dict[self.volume_controller], 
                                     max_output=127, 
                                     min_input=0, 
                                     max_input=TWO_BYTE)
        
        # Volume control based on accelerometer magnitude (with averaging)
        if self.volume_controller == 'accel_mag' and all(k in reading_dict for k in ['imu3', 'imu4', 'imu5']):
            # Calculate the magnitude of acceleration
            norm = np.sqrt((reading_dict['imu3']-TWO_BYTE/2.0)**2 + 
                          (reading_dict['imu4']-TWO_BYTE/2.0)**2 + 
                          (reading_dict['imu5']-TWO_BYTE/2.0)**2)
            
            # Apply offset and add to averaging queue
            volume = max(norm - ZERO_ACCEL, 0)
            self.averaging_queue.append(volume)
            
            # Calculate average over the current window size
            mean = sum(islice(self.averaging_queue, 
                            self.averaging_window_size_max - self.averaging_window_size, 
                            self.averaging_window_size_max)) / self.averaging_window_size
            
            # Scale the average to volume range, adding the base volume
            return self.base_volume + self.input_scaling(
                mean, 
                max_output=127-self.base_volume, 
                min_input=0, 
                max_input=15000
            )
            
        return self.base_volume

    def window_averaging_control(self, reading_dict):
        """Calculate averaging window size based on sensor readings"""
        if not self.averaging_window_controller or self.averaging_window_controller not in reading_dict:
            return self.averaging_window_size_max
            
        # Scale sensor value to window size range (1 to max)
        size = self.input_scaling(
            reading_dict[self.averaging_window_controller],
            min_output=1,  # Minimum window size of 1
            max_output=self.averaging_window_size_max, 
            min_input=0, 
            max_input=TWO_BYTE
        )
        
        return size

    def ten_finger_instrument(self, reading_dict):
        """Handle ten finger instrument where each finger triggers a note"""
        hand = reading_dict.get('hand')
        if 'switch' in reading_dict and hand in self.note_windows:
            # Trigger notes based on finger movements
            self.midi_writer.trig_note_array(
                reading_dict['switch'], 
                self.note_windows[hand], 
                vel=self.global_volume
            )

        # Handle averaging window control (typically left hand)
        if self.averaging_window_controller is not None and hand == 'l':
            new_size = self.window_averaging_control(reading_dict) 
            if new_size != self.averaging_window_size:
                self.averaging_window_size = new_size
                if self.debug:
                    print(f"Window size: {self.averaging_window_size}")

        # Update volume if controller is present
        if self.volume_controller is not None:
            new_volume = self.volume_control(reading_dict)
            if new_volume != self.global_volume:
                self.global_volume = new_volume
                if self.debug:
                    print(f"Volume: {self.global_volume}")
                self.midi_writer.aftertouch(self.global_volume)
                
        # Apply pitch bend if controller is present
        if self.pitch_bender is not None:
            bend_value = self.pitch_bend(reading_dict)
            if bend_value != 0:
                self.midi_writer.pitch_bend(bend_value)

    def double_flex_instrument(self, reading_dict):
        """Handle double flex instrument where left hand selects note window and right hand plays notes"""
        hand = reading_dict.get('hand')
        
        # Left hand controls note selection
        if hand == 'l' and 'switch' in reading_dict:
            # Calculate new note array based on left hand finger positions
            idx_array = []
            for i, sw in enumerate(reading_dict['switch']):
                if sw == 1:  # finger down
                    idx_array.append(i)
                elif sw == -1:  # finger up
                    if i in idx_array:
                        idx_array.remove(i)
                        
            if idx_array:
                # Update note array based on the selected window
                self.note_array, note_array_idx = self.mapper.window_map(
                    idx_array, 
                    self.window_trigger, 
                    self.note_windows
                )
                if self.debug:
                    print(f"Selected notes: {self.note_array}")
        
        # Right hand plays notes
        elif hand == 'r' and 'switch' in reading_dict and self.note_array is not None:
            # Trigger notes based on right hand finger movements and current note array
            self.midi_writer.trig_note_array(
                reading_dict['switch'], 
                self.note_array, 
                vel=self.global_volume
            )
            
        # Handle volume, averaging window, and pitch bend
        if self.volume_controller is not None:
            self.global_volume = self.volume_control(reading_dict)
            self.midi_writer.aftertouch(self.global_volume)
            
        if self.averaging_window_controller is not None and hand == 'l':
            self.averaging_window_size = self.window_averaging_control(reading_dict)
            
        if self.pitch_bender is not None:
            bend_value = self.pitch_bend(reading_dict)
            if bend_value != 0:
                self.midi_writer.pitch_bend(bend_value)

    def run(self):
        """Main application loop for MovingWindow"""
        self.reader.start_readers()
        self.initialize_triggers()
        
        if self.debug:
            print(f"Starting MovingWindow with {self.instrument} instrument")
            print(f"Volume controller: {self.volume_controller}, Pitch bender: {self.pitch_bender}")
            print(f"Averaging window size: {self.averaging_window_size}")

        while True:
            reading_dict = self.collect_q.get(block=True)
            
            # Process based on the selected instrument
            if self.instrument == 'ten_finger':
                self.ten_finger_instrument(reading_dict)
            elif self.instrument == 'double_flex':
                self.double_flex_instrument(reading_dict)


# Add Harmonizer and other specialized classes as needed
class Harmonizer(MovingWindow):
    """Harmonizer implementation"""
    def __init__(self, *args, chord_type='major', **kwargs):
        super().__init__(*args, **kwargs)
        self.chord_type = chord_type
        # Add harmonizer-specific initialization here 