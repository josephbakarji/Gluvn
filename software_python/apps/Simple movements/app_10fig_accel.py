"""
double-flex instrument with
accelerometer-magnitude volume envelope and gyro-roll pitch bend.

Trigger: hysteresis on flex (both hands).
Volume:  ||a|| envelope, moving-average smoothed, gz-driven window length.
Pitch:   modulo-wrapped roll (imu2) -> pitch bend.

Assumes firmware use_gyro=False (imu[3:6] = accel triplet). If use_gyro=True
on the glove, mod_idx below must be re-derived — accel_mag becomes undefined.
"""

from software_python.core.app_core import MovingWindow, launch_app

CONFIG = dict(
    root_note='C', scale='major',
    thresholds={'flex': 130, 'press': 20}, hysteresis=5,
    instrument='ten_finger',
    volume_controller='accel_mag', pitch_bender=None,
    averaging_window_controller='imu2', averaging_window_size=600,
    base_volume=0, num_lh_fingers=3,
)

if __name__ == "__main__":
    launch_app(CONFIG, mod_idx=[2, 3, 4, 5])   # roll, ax, ay, az