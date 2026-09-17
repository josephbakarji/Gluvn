"""
Ten-finger instrument, flex-triggered, pitch-bend
driven by roll (imu2), averaging window length driven by pitch (imu1).
No amplitude envelope (volume_controller=None) — fixed velocity at base_volume.

Assumes firmware use_gyro=False (imu[3:6] unused here). IMU index convention:
imu = [yaw, pitch, roll, i3, i4, i5].
"""

from software_python.core.app_core import MovingWindow, launch_app

CONFIG = dict(
    root_note='C', scale='major',
    thresholds={'flex': 130, 'press': 20}, hysteresis=5,
    instrument='ten_finger',
    volume_controller=None, pitch_bender='imu2',
    averaging_window_controller='imu1', averaging_window_size=600,
    base_volume=60, num_lh_fingers=3,
)

if __name__ == "__main__":
    launch_app(CONFIG, mod_idx=[1, 2])   # pitch, roll