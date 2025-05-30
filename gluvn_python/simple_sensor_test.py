#!/usr/bin/env python3
"""
Simple sensor test to diagnose sensor reading issues
"""

import time
import queue
from port_read import Reader

def test_sensor_changes():
    print("GLUVN Simple Sensor Test")
    print("=" * 40)
    print("This will monitor for ANY changes in sensor values")
    print("Try flexing, pressing, moving fingers, and even disconnecting/reconnecting cables")
    print("Press Ctrl+C to stop")
    print()
    
    # Use exact same config as plot_sensors.py
    sensor_config = {
        'l': {'flex': True, 'press': True, 'imu': True},
        'r': {'flex': True, 'press': True, 'imu': True}
    }
    
    reader = Reader(sensor_config=sensor_config, save=False)
    reader.start_readers()
    
    # Store baseline values
    baseline = {'l': {}, 'r': {}}
    first_reading = True
    reading_count = 0
    
    try:
        while True:
            any_data_received = False
            
            for hand in ['l', 'r']:
                if hand not in reader.threads or 'parser' not in reader.threads[hand]:
                    continue
                
                parser_queue = reader.threads[hand]['parser'].getQ()
                
                # Get latest data
                latest_data = None
                while True:
                    try:
                        data = parser_queue.get(block=False)
                        latest_data = data
                        any_data_received = True
                    except queue.Empty:
                        break
                
                if latest_data:
                    reading_count += 1
                    
                    # Store baseline on first reading
                    if first_reading and hand not in baseline:
                        baseline[hand] = latest_data.copy()
                        print(f"Baseline {hand.upper()}: {latest_data}")
                    
                    # Check for changes from baseline
                    elif hand in baseline:
                        changes_detected = False
                        
                        for sensor_type in ['flex', 'press', 'imu']:
                            if sensor_type in latest_data and sensor_type in baseline[hand]:
                                current = latest_data[sensor_type]
                                base = baseline[hand][sensor_type]
                                
                                # Check if any value changed significantly (more than 5 units)
                                for i, (curr_val, base_val) in enumerate(zip(current, base)):
                                    if abs(curr_val - base_val) > 5:
                                        print(f"🔥 CHANGE DETECTED! {hand.upper()} {sensor_type}[{i}]: {base_val} → {curr_val} (Δ{curr_val-base_val})")
                                        changes_detected = True
                        
                        # Print current values every 10 readings even if no changes
                        if reading_count % 10 == 0:
                            print(f"Reading #{reading_count} - {hand.upper()}: {latest_data}")
            
            if first_reading and any_data_received:
                first_reading = False
                print("\n✅ Data acquisition working. Now watching for changes...")
                print("👉 Try flexing your fingers, pressing sensors, or moving the glove")
                print()
            
            time.sleep(0.1)  # 100ms between checks
            
    except KeyboardInterrupt:
        print(f"\n\n📊 Test Summary:")
        print(f"Total readings: {reading_count}")
        print("Final values:")
        for hand in baseline:
            print(f"  {hand.upper()}: {baseline[hand]}")
    
    finally:
        reader.stop_readers()

if __name__ == "__main__":
    test_sensor_changes() 