"""
app_harmonizer.py — ten-finger instrument, flex-triggered, accel-magnitude
volume envelope with gz-driven averaging window.

Assumes firmware use_gyro=False. IMU index convention:
imu = [yaw, pitch, roll, i3, i4, i5] -> mod_idx=[2,3,4,5] = roll, ax, ay, az.
"""

from software_python.core.app_core import MovingWindow, launch_app

CONFIG = dict(
    root_note='C', scale='major',
    thresholds={'flex': 180, 'press': 20}, hysteresis=5,
    instrument='ten_finger',
    volume_controller='accel_mag', pitch_bender=None,
    averaging_window_controller='imu2', averaging_window_size=600,
    base_volume=50, num_lh_fingers=3,
)

if __name__ == "__main__":
    launch_app(CONFIG, mod_idx=[2, 3, 4, 5])