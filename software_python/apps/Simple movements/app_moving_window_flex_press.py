from software_python.core.app_core import BaseApp, MovingWindow
import numpy as np 


root_note = 'A'
scale = 'minor' 

num_lh_fingers = 3 
trigger_sensors = {'l': 'flex', 'r': 'press'}
mod_sensors = {'r':['imu'], 'l':[None]}
mod_idx = {'r':[2], 'l':None}  # roll (Mahony-fused) — change to [1] for pitch if that's the intended axis

volume_controller = None
pitch_bender = 'imu2'   # roll; matches mod_idx above
thresholds = {'flex': 150, 'press': 15}

app = MovingWindow(trigger_sensors=trigger_sensors, 
                    root_note=root_note, 
                    scale=scale, 
                    mod_sensors=mod_sensors, 
                    mod_idx=mod_idx,
                    thresholds=thresholds,
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
