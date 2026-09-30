#!/usr/bin/env python3
"""Stream decoded sensor values to the console for hardware checks."""

import time
import queue
from core.port_read import Reader

def main():
    print("Starting sensor monitor...")
    print("Flex your fingers and watch for changing values")
    print("Press Ctrl+C to stop")
    print("=" * 50)
    
    sensor_config = {
        'l': {'flex': True, 'press': True, 'imu': True},
        'r': {'flex': True, 'press': True, 'imu': True}
    }
    
    reader = Reader(sensor_config=sensor_config)
    reader.start_readers()
    
    print(f"Available hands: {list(reader.threads.keys())}")
    
    print("\nReading sensor data (flex your fingers!):")
    print("=" * 50)
    
    try:
        while True:
            for hand in ['l', 'r']:
                if hand not in reader.threads or 'parser' not in reader.threads[hand]:
                    continue
                
                parser_queue = reader.threads[hand]['parser'].getQ()
                
                latest_data = None
                
                while True:
                    try:
                        data = parser_queue.get(block=False)
                        latest_data = data
                    except queue.Empty:
                        break
                
                # Print the latest data if we got any
                if latest_data:
                    flex_values = latest_data.get('flex', [])
                    press_values = latest_data.get('press', [])
                    
                    print(f"{hand.upper()} Hand - Flex: {flex_values[:5] if len(flex_values) >= 5 else flex_values}, "
                          f"Press: {press_values[:5] if len(press_values) >= 5 else press_values}")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nStopping sensor monitor...")
    finally:
        reader.stop_readers()

if __name__ == "__main__":
    main() 