"""
Per-channel time series for a saved Gluvn recording.

Reads a session directory (ReadWrite/Analyze convention), plots each sensor
group in its own subplot on a shared time axis, plus ||a|| (accel magnitude,
physically-grounded envelope — not a learned feature).

Assumes firmware use_gyro=False (imu[3:6] = accel). Toggle IMU_MODE below to
match the glove's actual configuration at capture time.
"""

import os
import pandas as pd
import time
import queue

from core.port_read import Reader

sensor_config = {
    'r': {
        'flex': True,
        'press': True,
        'imu': True
    }
}

reader = Reader(
    sensor_config=sensor_config,
    use_ble=False      # USB firmware
)

reader.start_readers()

q = reader.threads['r']['parser'].getQ()

rows = []

print("Recording...")
print("Press CTRL+C to stop")

t0 = time.time()

try:

    while True:

        try:
            data = q.get(timeout=1)
        except queue.Empty:
            continue

        row = {
            'time': time.time() - t0
        }

        # flex sensors
        if 'flex' in data:
            for i, v in enumerate(data['flex']):
                row[f'flex{i+1}'] = v

        # pressure sensors
        if 'press' in data:
            for i, v in enumerate(data['press']):
                row[f'press{i+1}'] = v

        # imu
        if 'imu' in data:

            row['yaw']   = data['imu'][0]
            row['pitch'] = data['imu'][1]
            row['roll']  = data['imu'][2]

            if len(data['imu']) >= 6:
                row['imu4'] = data['imu'][3]
                row['imu5'] = data['imu'][4]
                row['imu6'] = data['imu'][5]

        rows.append(row)

except KeyboardInterrupt:

    print("\nStopping...")

    df = pd.DataFrame(rows)

    # Resolve absolute path to the directory containing this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = f"session_{int(time.time())}.csv"
    filepath = os.path.join(script_dir, filename)

    df.to_csv(filepath, index=False)

    print(f"Saved {len(df)} samples")
    print(f"File: {filepath}")

finally:

    reader.stop_readers()