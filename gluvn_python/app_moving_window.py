from senstonote_new import BaseApp
import numpy as np 

TWO_BYTE = 65535
BYTE = 255

class MovingWindowContinous(BaseApp):
    def __init__(self, *args, volume_controller=None, pitch_bender=None, num_lh_fingers=5, num_rh_fingers=5, **kwargs):
        super().__init__(*args, **kwargs)
        self.window_trigger, self.note_windows = self.mapper.moving_window(num_lhf=num_lh_fingers, num_rhf=num_rh_fingers)
        self.num_lh_fingers = num_lh_fingers 
        self.num_rh_fingers = num_rh_fingers 
        self.playing_notes = [None]*num_rh_fingers
        self.volume_controller = volume_controller
        self.pitch_bender = pitch_bender
        self.pitch_bend_limit = 8192
        self.global_volume = 10

    def input_scaling(self, input, min_output=0, max_output=127, shift=0, min_input=0, max_input=BYTE):
        return int( min_output + (max_output - min_output) * ((input + shift) - min_input) / (max_input - min_input))

    def scaling_modulo(self, input, min_output=0, max_output=127, min_input=0, max_input=BYTE):
        return int((( max_output + (max_output - min_output) * (input - min_input)/(max_input - min_input)) % (max_output - min_output)) - max_output)
            

    def run(self):
        self.reader.start_readers()
        self.initialize_triggers()
        note_array, note_array_idx = self.mapper.window_map([0]*self.num_lh_fingers, self.window_trigger, self.note_windows)
        print(self.note_windows)
        
        while True:
            reading_dict = self.collect_q.get(block=True)
            if reading_dict['hand'] == 'l':
                # assumes left hand is only responsible for window shifting
                temp = self.mapper.window_map(self.triggers['l'].turn_state[:self.num_lh_fingers], self.window_trigger, self.note_windows)
                if temp is not None:
                    note_array, note_array_idx = temp
            else:
                if np.any(reading_dict['switch']):
                    self.playing_notes = self.midi_writer.trig_notes_array_playing(reading_dict['switch'], self.playing_notes, note_array, vel=self.global_volume)
                else:
                    self.global_volume = self.input_scaling(reading_dict[self.volume_controller], max_output=127, min_input=0, max_input=TWO_BYTE)
                    pitchbend_val = self.scaling_modulo(reading_dict[self.pitch_bender], min_output=-self.pitch_bend_limit, 
                                                       max_output=self.pitch_bend_limit, min_input=0, max_input=TWO_BYTE)
                    print(reading_dict[self.pitch_bender], pitchbend_val)
                    # print(self.global_volume)
                    self.midi_writer.aftertouch(self.global_volume, channel=1)
                    self.midi_writer.pitch_bend(pitchbend_val)




if __name__ == '__main__':
    root_note = 'C'
    scale = 'minor'

    num_lh_fingers = 3
    trigger_sensors = {'l': 'flex', 'r': 'flex'}
    mod_sensors = {'r':['imu', 'imu'], 'l':None} ## Make sure to use a list even if only one sensor
    mod_idx = {'r':[1, 2], 'l':None} # [yaw, pitch, roll] -> [0, 1, 2]
    volume_controller = 'imu1'
    pitch_bender = 'imu2'

    # Make sensor_config based on trigger_sensors and mod_sensors
    sensor_config={'r': {'flex': True, 'press': False, 'imu': True},
                   'l': {'flex': True, 'press': False, 'imu': False}}
    

    app = MovingWindowContinous(sensor_config=sensor_config, 
                                trigger_sensors=trigger_sensors, 
                                 root_note=root_note, 
                                 scale=scale, 
                                 mod_sensors=mod_sensors, 
                                 mod_idx=mod_idx,
                                 volume_controller=volume_controller,
                                 pitch_bender=pitch_bender,
                                 num_lh_fingers=num_lh_fingers)
    app.start()

    key = input('press any key to finish \n')
    print('Shutting down...')

    app.reader.stop_readers()
    for hand in app.hands:
        app.triggers[hand].join(timeout=.1)
    app.join(timeout=.1)  
