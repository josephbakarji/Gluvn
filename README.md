# 🎼 M5-GLUVN - Wireless Digital Musical Glove System

The next evolution of GLUVN: **wireless, portable, and powered by M5Stick C Plus 1.1**

**✨ NEW**: This is the **M5-GLUVN** project—a complete redesign of the original GLUVN with wireless BLE connectivity, built-in IMU, and a custom PCB with multiplexer support. For the original Arduino-based USB version, see [README_WIRED.md](README_WIRED.md).

## 📋 Project Overview

**M5-GLUVN** transforms hand movements and finger gestures into wireless MIDI music. Improvements over the original:

### What's New in M5-GLUVN
- 🔋 **Wireless via Bluetooth Low Energy (BLE)** - Play untethered
- 🎯 **M5Stick C Plus 1.1 Microcontroller** - Compact, built-in IMU
- 📡 **Custom Multiplexer PCB** - CD4067 multiplexer enables reading all 10 sensors from limited ports
- ⚡ **Built-in IMU** - M5Stick's BMI270/MPU6886 for volume and pitch control (no external IMU needed)
- 🔧 **Custom Electronics** - Optimized capacitors/resistors for better sensor accuracy
- 📊 **Dual Connectivity** - BLE wireless + USB fallback for development/charging
- 🎮 **Bandwidth Optimized** - Efficient binary frames for wireless transmission
- 📱 **Real-time Calibration** - BLE-based calibration tool for on-the-fly adjustments

## 🏗️ Hardware Architecture

### Microcontroller
- **M5Stick C Plus 1.1** - Compact ESP32-based device with:
  - Built-in 6-DOF IMU (BMI270 or MPU6886)
  - 12-bit ADC (4096 levels) for sensor reading
  - BLE and WiFi capabilities
  - 540 mAh battery for portable play
  - USB-C for charging and serial communication

### Sensor Configuration

**Left Hand Sensors** (5x per sensor type):
- 5x Flex Sensors (finger bending)
- 5x FSR/Pressure Sensors (finger force)

**Right Hand Sensors** (5x per sensor type):
- 5x Flex Sensors (finger bending)
- 5x FSR/Pressure Sensors (finger force)

**IMU** (built-in to M5Stick):
- 3-axis Accelerometer (default for volume control)
- 3-axis Gyroscope (optional for pitch bend)
- Selectable for different modulation modes

### Custom PCB with CD4067 Multiplexer

**Problem Solved**: M5Stick has limited analog input ports (~3), but we need to read 10 sensors (5 flex + 5 FSR) per hand.

**Solution**: CD4067 16-channel analog multiplexer handles sensor switching:

```
CD4067 Multiplexer Configuration:
├─ CH0-4   → 5x FSR Sensors (10K pull-down, 10nF cap)
├─ CH5-9   → 5x Flex Sensors (47K pull-down, 100nF cap)
├─ CH10-14 → Unused
├─ CH15    → Ground/Reference
└─ MUX Control: 4 GPIO pins (S0-S3) select active channel

PCB Features:
├─ Separate filter networks for flex vs FSR
│  ├─ Flex: 47K + 100nF (slower response)
│  └─ FSR: 10K + 10nF (faster response)
├─ Multiplexer settling time: 1000µs
└─ Binary serial protocol for efficient data transmission
```

**Wire Connections**:
- MUX_SIG → GPIO36 (ADC0)
- MUX_S0 → GPIO26
- MUX_S1 → GPIO32
- MUX_S2 → GPIO33
- MUX_S3 → GPIO0

## 📁 Project Structure

```
GLUVN-M5/
├── gluvn_python/              # Main Python application (shared with original)
│   ├── core/                  # Core business logic
│   ├── configs/               # Configuration management
│   ├── visualization/         # GUI and real-time display (PyQt5)
│   ├── examples/              # Pre-built applications
│   ├── tests/                 # Unit tests
│   ├── port_read.py           # USB serial reader (original)
│   ├── port_read_BLE.py       # ⭐ NEW: BLE reader for M5
│   ├── calibrate_eeprom.py    # USB calibration
│   ├── calibrate_eeprom_BLE.py # ⭐ NEW: BLE calibration
│   ├── benchmark_bandwidth.py # ⭐ NEW: Benchmark BLE vs USB
│   └── senstonote_modern.py   # Core application framework
│
├── arduino/
│   └── sendsens/
│       ├── sendsens.ino       # ⭐ UPDATED: M5Stick + BLE + Multiplexer firmware
│       └── calibration.h      # Generated calibration constants
│
├── docs/                      # Documentation
│   ├── Datasheets/
│   └── Pins and sensr mapping.pdf
│   └── ...
```

## 🚀 Quick Start - M5-GLUVN

### Prerequisites
- **M5Stick C Plus 1.1** device
- **Custom PCB** with CD4067 multiplexer
- **10 Flex Sensors** + **10 FSR/Pressure Sensors**
- **Python 3.7+**
- **Arduino IDE** for firmware

### Step 1: Upload M5 Firmware

```bash
# Using Arduino IDE:
# 1. Install M5Unified library
# 2. Install ESP32 board support
# 3. Open: arduino/sendsens/sendsens.ino
# 4. Board: M5Stack-Core2 (or M5Stick C Plus)
# 5. Upload

```

**Verify firmware**:
```bash
# Connect M5 to USB, check serial output:
# Should show: "M5Unified init success", "BLE init success"
```

### Step 2: Install Python Dependencies

```bash
cd gluvn_python

# Install core dependencies
pip install numpy scipy

# Install for GUI + BLE support
pip install PyQt5 pygame
pip install bleak  # ⭐ BLE library (NEW)

# Test BLE functionality
python -c "from bleak import BleakScanner; print('✅ BLE working')"
```

### Step 3: Calibrate with BLE

```bash
# Make sure M5Stick is powered on and near computer
python calibrate_eeprom_BLE.py
```

**What to do**:
1. Select hand (L or R)
2. For each finger: Flex sensor value reaches max, then release
3. Apply light pressure on FSR, then heavy pressure
4. Calibration values saved to M5 NVS (non-volatile storage)

### Step 4: Run the Application

```bash
# For BLE mode (wireless):
python examples/ten_finger_gui_with_sensors.py --ble

# For USB fallback (if BLE unavailable):
python examples/ten_finger_gui_with_sensors.py --usb

# With BLE bandwidth monitoring:
python benchmark_bandwidth.py --duration 30 --mode ble
```

## 🔌 Wiring & PCB Assembly

### Sensor Connections to CD4067

```
Flex Sensors (CH5-9):
  Flex_L0 → MUX_CH5
  Flex_L1 → MUX_CH6
  Flex_L2 → MUX_CH7
  Flex_L3 → MUX_CH8
  Flex_L4 → MUX_CH9

FSR/Pressure (CH0-4):
  FSR_L0 → MUX_CH0
  FSR_L1 → MUX_CH1
  FSR_L2 → MUX_CH2
  FSR_L3 → MUX_CH3
  FSR_L4 → MUX_CH4
```

### Filter Components

**For Flex Sensors**:
- Pull-down: 47kΩ resistor
- Capacitor: 100nF
- Time constant: RC = 47k × 100n = 4.7ms

**For FSR/Pressure**:
- Pull-down: 10kΩ resistor
- Capacitor: 10nF
- Time constant: RC = 10k × 10n = 100µs

## 📊 Communication Protocols

### BLE Protocol (Wireless)

**Nordic UART Service (NUS)**:
```
Service UUID:  6E400001-B5A3-F393-E0A9-E50E24DCCA9E
RX Char UUID:  6E400002-B5A3-F393-E0A9-E50E24DCCA9E (write)
TX Char UUID:  6E400003-B5A3-F393-E0A9-E50E24DCCA9E (notify)
```

**Binary Frame Format** (25 bytes):
```
[0]     = Hand ('R' or 'L')
[1]     = Sequence number (0-255)
[2-3]   = Yaw (uint16 big-endian)
[4-5]   = Pitch (uint16)
[6-7]   = Roll (uint16)
[8-9]   = IMU0 (accel_x or gyro_x)
[10-11] = IMU1 (accel_y or gyro_y)
[12-13] = IMU2 (accel_z or gyro_z)
[14-18] = Flex sensors [0-4] (5 bytes, calibrated 0-255)
[19-23] = FSR sensors [0-4] (5 bytes, calibrated 0-255)
[24]    = '\n' (frame terminator)
```

**Bitrate**: ~100 Hz × 25 bytes × 8 bits = **20 kbps** (well within BLE 1 Mbps limit)

### USB Serial Fallback

Same binary protocol as original GLUVN, available on USB-C port for:
- Development and debugging
- Firmware uploading
- Charging while playing

## 📖 Documentation

- **[GETTING_STARTED.md](gluvn_python/GETTING_STARTED.md)** - Comprehensive quickstart guide
- **[USER_CONFIGURABLE_PARAMETERS.md](gluvn_python/USER_CONFIGURABLE_PARAMETERS.md)** - Detailed parameter tuning
- **[ARCHITECTURE_VERIFICATION.md](gluvn_python/ARCHITECTURE_VERIFICATION.md)** - Technical architecture details
- **[CLEAN_ARCHITECTURE_SUCCESS.md](gluvn_python/CLEAN_ARCHITECTURE_SUCCESS.md)** - Design patterns verification
- **[examples/README.md](gluvn_python/examples/README.md)** - Application examples guide


## 🔧 Configuration & Tuning

### Musical Settings (same as original)

```
Default Note Layout (C Major):
Left Hand:          Right Hand:
Thumb:  G2 (55)     Thumb:  C4 (60) ← Middle C
Index:  A2 (57)     Index:  D4 (62)
Middle: B2 (59)     Middle: E4 (64)
Ring:   C3 (52)     Ring:   F4 (65)
Pinky:  D3 (53)     Pinky:  G4 (67)

Supported Scales:
- Major (bright)
- Minor (dark)
- Pentatonic (improvisation-friendly)
```

## 🐛 Troubleshooting

### BLE Connection Issues

**Problem**: M5Stick not discovered
```bash
# 1. Check M5 is powered on and firmware uploaded
# 2. Restart M5 by pressing side button
# 3. Check BLE name in __init__.py matches device

# If still not found:
python -c "from bleak import BleakScanner; asyncio.run(BleakScanner.discover())"
```

**Problem**: BLE connects but no data flowing
```bash
# 1. Check calibration was saved: 
#    - M5 should display "CAL: OK" on startup
# 2. Send "STATUS" command via BLE
# 3. Verify binary format matches struct definition
```

### Sensor Reading Issues

**Problem**: Some sensor channels read 0
```bash
# Check MUX channel selection
# Verify wiring to CD4067 inputs
# Test with raw ADC read (not multiplexed):
python simple_sensor_test.py --mux-disable
```

### Performance Issues

**Problem**: BLE drops packets
```bash
# Check battery level (low battery = weak signal)
# Move closer to computer (typical range 5-10m indoors)
# Check for WiFi interference on 2.4 GHz
# Reduce GUI update rate in examples
```

**Problem**: High latency (>100ms)
```bash
# Disable debug printing (set print_mode=0 in firmware)
# Switch to lower latency GUI mode
# Reduce PyQt5 update rate to 50 Hz
```


## 🔄 Comparison with Original GLUVN

| Feature | Original | M5-GLUVN |
|---------|----------|----------|
| **Microcontroller** | Arduino (Micro) | M5Stick C Plus 1.1 |
| **Connectivity** | USB Serial | BLE + USB fallback |
| **IMU** | External (separate) | Built-in (BMI270) |
| **ADC Resolution** | 10-bit | 12-bit |
| **Sensor Reading** | Direct GPIO | CD4067 Multiplexer |
| **Portability** | Tethered | Wireless (4-6h battery) |


## 📝 License

MIT License - Same as original GLUVN

---

**🎵 M5-GLUVN: The future of portable, wireless musical expression**

*Transform your hand into a wireless musical instrument.*
