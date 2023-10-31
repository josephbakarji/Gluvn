from senstonote_new import BaseApp, MovingWindow
import numpy as np 


root_note = 'D'
scale = 'minor'

num_lh_fingers = 3 
trigger_sensors = {'l': 'flex', 'r': 'flex'}
mod_sensors = {'r':['imu', 'imu', 'imu'], 'l':[None]} ## Make sure to use a list even if only one sensor
mod_idx = {'r':[3, 4, 5], 'l':None} # [yaw, pitch, roll] -> [0, 1, 2] ## TODO: assumes this order arduino-side must check.

volume_controller = 'accel_mag'
pitch_bender = None 
avg_window_size = 10

app = MovingWindow(trigger_sensors=trigger_sensors, 
                    root_note=root_note, 
                    scale=scale, 
                    mod_sensors=mod_sensors, 
                    mod_idx=mod_idx,
                    volume_controller=volume_controller,
                    pitch_bender=pitch_bender,
                    num_lh_fingers=num_lh_fingers,
                    avg_window_size=avg_window_size)
app.start()

key = input('press any key to finish \n')
print('Shutting down...')

app.reader.stop_readers()
for hand in app.hands:
    app.triggers[hand].join(timeout=.1)
app.join(timeout=.1)  
