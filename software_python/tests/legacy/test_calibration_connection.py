#!/usr/bin/env python3
"""
Arduino Connection and Command Interface Test

This script tests the basic connection to Arduino and verifies that
the serial command interface is working properly.

Usage:
    python test_calibration_connection.py
"""

import serial
import time
from __init__ import portL, portR

def test_arduino_connection(port, hand_name):
    print(f"\n🔌 Testing {hand_name} hand on {port}")
    
    try:
        # Connect to Arduino
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(2)  # Wait for Arduino to initialize
        
        # Clear any existing data
        ser.read_all()
        
        print("1. Getting Arduino status...")
        ser.write(b"STATUS\n")
        time.sleep(0.5)
        response = ser.read_all().decode('utf-8', errors='ignore')
        print(f"   Status response: {response.strip()}")
        
        print("2. Testing hand switching...")
        hand_char = 'R' if 'right' in hand_name.lower() else 'L'
        ser.write(f"HAND_{hand_char}\n".encode())
        time.sleep(0.5)
        response = ser.read_all().decode('utf-8', errors='ignore')
        print(f"   Hand switch response: {response.strip()}")
        
        print("3. Testing calibration mode...")
        ser.write(b"CAL_START\n")
        time.sleep(0.5)
        response = ser.read_all().decode('utf-8', errors='ignore')
        print(f"   Calibration start response: {response.strip()}")
        
        print("4. Switching to RAW mode...")
        ser.write(b"RAW_MODE\n")
        time.sleep(0.5)
        response = ser.read_all().decode('utf-8', errors='ignore')
        print(f"   RAW mode response: {response.strip()}")
        
        print("5. Clearing buffer after mode switch...")
        time.sleep(1)
        ser.read_all()
        print("   ✅ Buffer cleared")
        
        print("6. Testing data reception for 5 seconds...")
        start_time = time.time()
        data_count = 0
        
        while time.time() - start_time < 5:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line and '\t' in line:  # Raw data has tabs
                    parts = line.split('\t')
                    if len(parts) >= 10:  # Should have at least 10 sensor values
                        flex_data = parts[0:5]
                        press_data = parts[5:10]
                        data_count += 1
                        if data_count <= 3:  # Show first 3 samples
                            print(f"   Data #{data_count}: Flex={flex_data}, Press={press_data}")
            except:
                continue
        
        print(f"   ✅ Received {data_count} valid data samples")
        
        print("7. Returning to normal mode...")
        ser.write(b"NORMAL_MODE\n")
        time.sleep(0.5)
        
        ser.close()
        print(f"   ✅ Disconnected")
        return True
        
    except Exception as e:
        print(f"   ❌ Error: {e}")
        return False

def main():
    print("🧪 Arduino Connection and Command Interface Test")
    print("=" * 60)
    
    # Test both hands
    hands = [
        (portL, "LEFT"),
        (portR, "RIGHT")
    ]
    
    results = []
    for port, hand_name in hands:
        try:
            result = test_arduino_connection(port, hand_name)
            results.append((hand_name, result))
        except Exception as e:
            print(f"❌ Failed to test {hand_name} hand: {e}")
            results.append((hand_name, False))
    
    print("\n" + "=" * 60)
    print("🎯 Connection Test Results:")
    for hand_name, success in results:
        status = "✅ Working" if success else "❌ Failed"
        print(f"{hand_name} hand:  {status}")

if __name__ == "__main__":
    main() 