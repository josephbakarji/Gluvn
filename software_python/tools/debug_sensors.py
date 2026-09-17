#!/usr/bin/env python3
"""
Debug script to test sensor readings
Mimics the exact approach used in plot_sensors.py
"""

import time
import queue
from core.port_read import Reader

def main():
    print("Starting sensor debug test...")
    print("Flex your fingers and watch for changing values")
    print("Press Ctrl+C to stop")
    print("=" * 50)
    
    # Initialize sensor reading exactly like plot_sensors.py
    sensor_config = {
        'l': {'flex': True, 'press': True, 'imu': True},
        'r': {'flex': True, 'press': True, 'imu': True}
    }
    
    reader = Reader(sensor_config=sensor_config, save=False)
    reader.start_readers()
    
    print(f"Available reader threads: {list(reader.threads.keys())}")
    for hand in reader.threads:
        print(f"Hand {hand} threads: {list(reader.threads[hand].keys())}")
    
    print("\nReading sensor data (flex your fingers!):")
    print("=" * 50)
    
    try:
        while True:
            for hand in ['l', 'r']:
                if hand not in reader.threads or 'parser' not in reader.threads[hand]:
                    continue
                
                parser_queue = reader.threads[hand]['parser'].getQ()
                
                # Try to get all available data (like plot_sensors.py does)
                data_count = 0
                latest_data = None
                
                while True:
                    try:
                        data = parser_queue.get(block=False)
                        latest_data = data
                        data_count += 1
                    except queue.Empty:
                        break
                
                # Print the latest data if we got any
                if latest_data:
                    flex_values = latest_data.get('flex', [])
                    press_values = latest_data.get('press', [])
                    
                    print(f"{hand.upper()} Hand - Flex: {flex_values[:5] if len(flex_values) >= 5 else flex_values}, "
                          f"Press: {press_values[:5] if len(press_values) >= 5 else press_values}")
            
            print()  # Empty line for readability
            time.sleep(0.5)  # Update every 500ms for readability
            
    except KeyboardInterrupt:
        print("\nStopping sensor debug test...")
    finally:
        reader.stop_readers()

if __name__ == "__main__":
    main() 