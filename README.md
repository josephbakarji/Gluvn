# GLUVN - Sensor Glove MIDI Controller

A hand-based musical instrument using flex sensors, pressure sensors, and IMU data to generate MIDI messages for digital audio workstations (DAWs).

## Hardware Overview

The glove contains:
- **5 flex sensors**: Measure finger bending angles
- **5 force sensitive resistors (FSRs)**: Measure fingertip pressure intensity  
- **1 IMU (MPU-6050)**: 3-axis gyroscope + 3-axis accelerometer for orientation and motion

## Current Repository Structure Analysis

### Arduino Code (`arduino/`)
- **`sendsens/sendsens.ino`**: ⭐ **ENHANCED** - Main Arduino sketch with EEPROM calibration support and serial command interface
- **`sendsens_simple/sendsens_simple.ino`**: Simplified Arduino version without EEPROM for basic functionality
- **`sendsens/calibration.h`**: Default calibration constants (now serves as intelligent fallback for EEPROM)
- **`sendsens/mpu.h/cpp`**: MPU-6050 IMU interface and processing
- **`sendsens/I2Cdev.h/cpp`**: I2C communication library for IMU
- **Other files**: DMP motion processing libraries for IMU

### Python Core (`gluvn_python/`)

#### **Data Acquisition & Communication**
- **`port_read.py`**: ⭐ **CORE** - Multi-threaded serial port reader with parsing for both hands
- **`calibrate.py`**: ⭐ **LEGACY** - Traditional calibration tool (still functional)
- **`calibrate_eeprom.py`**: ⭐ **NEW** - Advanced EEPROM calibration with real-time visualization and GUI

#### **Sensor-to-Note Transformation (Legacy Evolution)**
- **`senstonote_modern.py`**: ⭐ **CURRENT** - Modern unified sensor processing (renamed from senstonote_new.py)

#### **MIDI Output**
- **`midi_writer.py`**: ⭐ **CORE** - MIDI message generation and DAW communication

#### **Musical Mapping & Theory**
- **`mapper.py`**: ⭐ **CORE** - Scale generation, chord mapping, note window management

#### **Machine Learning & Gesture Recognition**
- **`learning.py`**: ML models for gesture-to-note mapping (thumb-under detection, transition matrices)

#### **Application Modules (Sensor-to-Note Mappings)**
- **`app_10fig_accel.py`**: 10-finger instrument with acceleration-based volume control
- **`app_10fig_pitch.py`**: 10-finger instrument with pitch-based modulation
- **`app_10fig_inst.py`**: Basic 10-finger instrument
- **`app_harmonizer.py`**: Harmonic chord generation application
- **`app_jacob_choir.py`**: Complex choir-like application
- **`app_moving_window_*.py`**: Moving window applications for different sensors

#### **Execution & Orchestration**
- **`exec.py`**: Main execution script with multiple operation modes
- **`app_runner.py`**: Application runner utility

#### **Data Analysis & Visualization**
- **`data_analysis.py`**: Comprehensive data analysis, plotting, and file I/O
- **`plot_sensors.py`**: ⭐ **RECENTLY UPDATED** - Real-time sensor visualization GUI

#### **Testing & Diagnostics**
- **`test_calibration_connection.py`**: Arduino connection and command interface testing

#### **Utilities & Configuration**
- **`utils.py`**: Utility functions
- **`__init__.py`**: Global configuration (ports, directories, MIDI settings)
- **`configs/`**: JSON configuration files for different applications

#### **Data Storage**
- **`data/`**: Organized data storage
  - `calibration/`: Sensor calibration files
  - `experiments/`: Experimental recordings
  - `keyboard_learning/`: ML training data
  - `settings/`: Application settings
  - `sim/`: Simulation data
  - `tables/`: Note-to-MIDI lookup tables

## 🎯 **NEW: Enhanced EEPROM Calibration System**

### **Major Improvements**
The calibration system has been completely overhauled to eliminate the tedious manual process of editing Arduino code and recompiling for calibration.

#### **Before (Legacy System)**
❌ Manual Arduino code editing (`calibration_mode = true/false`)  
❌ Multiple code uploads and recompilations  
❌ Text-only prompts with no visual feedback  
❌ No real-time sensor monitoring  
❌ Error-prone manual process  

#### **After (EEPROM System)**
✅ **Dynamic mode switching** via serial commands  
✅ **Real-time sensor visualization** during calibration  
✅ **Interactive GUI** with one-click calibration steps  
✅ **EEPROM storage** - calibration persists across power cycles  
✅ **No recompilation needed** - ever!  
✅ **Both hands supported** without reconnection  
✅ **Intelligent defaults** from calibration.h as fallback  

### **Arduino Enhancements**

#### **Serial Command Interface**
The Arduino now responds to real-time commands:
```
CAL_START     - Switch to calibration mode (raw sensor data)
CAL_STOP      - Switch to normal mode (calibrated data)
HAND_R        - Set Arduino to right hand mode
HAND_L        - Set Arduino to left hand mode
STATUS        - Get current Arduino status
EEPROM_SAVE   - Save calibration data to EEPROM
EEPROM_LOAD   - Load calibration data from EEPROM
EEPROM_RESET  - Reset to calibration.h defaults
```

#### **EEPROM Calibration Data Storage**
```
SET_CAL:R:MIN_FLEX:200,210,220,230,240   - Set right hand flex minimums
SET_CAL:R:MAX_FLEX:800,810,820,830,840   - Set right hand flex maximums
SET_CAL:L:MIN_PRESS:600,610,620,630,640  - Set left hand pressure minimums
SET_CAL:L:MAX_PRESS:100,110,120,130,140  - Set left hand pressure maximums
```

#### **Dual Storage Strategy**
1. **EEPROM (Primary)**: Runtime calibration data, persists across power cycles
2. **calibration.h (Fallback)**: Default values when EEPROM is empty or corrupted

### **Python Calibration Tools**

#### **🎯 Main Tool: `calibrate_eeprom.py`**
Advanced GUI calibration tool with real-time visualization:

**Features:**
- **Live sensor plotting** with pyqtgraph
- **Hand selection dropdown** (Right/Left)
- **Mode toggle** (CALIBRATION/NORMAL) with visual feedback
- **Individual finger calibration** buttons
- **Real-time data recording** with 3-second averaging
- **Automatic EEPROM storage** + backup file generation
- **Reset to defaults** functionality

**Usage:**
```bash
cd gluvn_python
python calibrate_eeprom.py
```

**Calibration Process:**
1. **Select Hand**: Choose Right or Left from dropdown
2. **Open Hand**: Click "Open Hand" → straighten fingers → data recorded automatically
3. **Close Fist**: Click "Close Fist" → make tight fist → data recorded automatically
4. **Individual Fingers**: Click each finger button → press that finger → data recorded
5. **Save**: Click "Save Calibration" → data sent to Arduino EEPROM + backup file created

#### **🔧 Legacy Tool: `calibrate.py`**
Traditional calibration tool (still functional):
```bash
python calibrate.py
```

#### **🧪 Testing Tool: `test_calibration_connection.py`**
Verify Arduino connection and command interface:
```bash
python test_calibration_connection.py
```

### **Technical Implementation**

#### **Arduino EEPROM Memory Layout**
```
Address 0-1:   Magic number (0xCAFE) - validates EEPROM data
Address 2-11:  Left hand MIN_FLEX values (5 × 2 bytes)
Address 12-21: Left hand MAX_FLEX values (5 × 2 bytes)
Address 22-31: Left hand MIN_PRESS values (5 × 2 bytes)
Address 32-41: Left hand MAX_PRESS values (5 × 2 bytes)
Address 42-51: Right hand MIN_FLEX values (5 × 2 bytes)
Address 52-61: Right hand MAX_FLEX values (5 × 2 bytes)
Address 62-71: Right hand MIN_PRESS values (5 × 2 bytes)
Address 72-81: Right hand MAX_PRESS values (5 × 2 bytes)
```

#### **Data Flow**
1. **Startup**: Arduino checks EEPROM magic number
2. **Valid EEPROM**: Load calibration from EEPROM
3. **Invalid/Empty EEPROM**: Load defaults from calibration.h
4. **Runtime**: Accept calibration updates via serial commands
5. **Save**: Store new calibration to EEPROM with magic number

#### **Error Handling**
- **EEPROM corruption**: Automatic fallback to calibration.h defaults
- **Invalid commands**: Arduino responds with error messages
- **Connection issues**: Python tools provide clear error messages
- **Data validation**: Range checking on calibration values

### **Benefits of EEPROM System**

1. **🚀 Workflow Efficiency**: Calibration time reduced from ~10 minutes to ~2 minutes
2. **🔧 No Recompilation**: Never need to edit and recompile Arduino code
3. **👀 Visual Feedback**: Real-time sensor monitoring during calibration
4. **💾 Persistent Storage**: Calibration survives power cycles and Arduino resets
5. **🔄 Easy Updates**: Recalibrate anytime without code changes
6. **🛡️ Robust Fallbacks**: Intelligent defaults prevent system failure
7. **🤝 Dual Hand Support**: Seamless switching between left and right hands

## Issues Identified

### 🔴 **Critical Issues**
1. **Code Duplication**: Multiple versions of sensor-to-note logic (`senstonote*.py`)
2. **Inconsistent Architecture**: Mix of old procedural and new OOP approaches
3. **Unclear Dependencies**: Complex import relationships between modules
4. **Temporary Files**: `sens2note_temp.py`, `exec_temp.py` should be cleaned up
5. **Legacy Code**: Old `senstonote.py` and `senstonote_2h.py` still present

### 🟡 **Moderate Issues**
1. **Configuration Management**: Inconsistent config handling across apps
2. **Documentation**: Minimal inline documentation
3. **Error Handling**: Limited error handling in core modules
4. **Testing**: No apparent test suite

## Reorganization Plan

### Phase 1: Core Architecture Cleanup

#### 1.1 **Consolidate Sensor-to-Note Logic**
```
REMOVE:
- senstonote.py (legacy)
- senstonote_2h.py (legacy) 
- sens2note_temp.py (temporary)

KEEP & ENHANCE:
- senstonote_new.py → rename to sensor_processor.py
```

#### 1.2 **Standardize Application Structure**
```
CREATE: apps/ directory
MOVE: app_*.py → apps/
STANDARDIZE: All apps inherit from BaseApp in sensor_processor.py
```

#### 1.3 **Unify Execution Layer**
```
MERGE: exec.py + exec_temp.py → main.py
ENHANCE: run_glove.py as primary orchestrator
REMOVE: Duplicate execution logic
```

### Phase 2: Module Organization

#### 2.1 **Core Modules** (`core/`)
```
core/
├── sensor_reader.py      (port_read.py)
├── sensor_processor.py   (senstonote_new.py)
├── midi_writer.py        (unchanged)
├── mapper.py            (unchanged)
└── calibration.py       (calibrate.py)
```

#### 2.2 **Applications** (`apps/`)
```
apps/
├── base_app.py          (BaseApp from sensor_processor.py)
├── ten_finger.py        (app_10fig_*.py consolidated)
├── moving_window.py     (app_moving_window_*.py consolidated)
├── harmonizer.py        (app_harmonizer.py)
└── choir.py            (app_jacob_choir.py)
```

#### 2.3 **Machine Learning** (`ml/`)
```
ml/
├── gesture_learning.py  (learning.py)
├── ml_processor.py      (ml_senstonote.py)
└── models/             (trained model storage)
```

#### 2.4 **Analysis & Visualization** (`analysis/`)
```
analysis/
├── data_analysis.py    (unchanged)
├── sensor_plotter.py   (plot_sensors.py)
└── visualization/      (plotting utilities)
```

### Phase 3: Configuration & Documentation

#### 3.1 **Unified Configuration System**
```
config/
├── base_config.yaml    (replace JSON configs)
├── app_configs/        (per-app configurations)
└── hardware_config.yaml (sensor/hardware settings)
```

#### 3.2 **Documentation Enhancement**
- Add comprehensive docstrings to all modules
- Create API documentation
- Add inline comments for complex algorithms
- Update README with usage examples

### Phase 4: Quality Improvements

#### 4.1 **Error Handling & Logging**
- Implement proper exception handling
- Add logging throughout the system
- Create graceful shutdown procedures

#### 4.2 **Testing Framework**
- Unit tests for core modules
- Integration tests for sensor pipeline
- Mock sensor data for testing

## Recommended File Actions

### 🗑️ **DELETE**
- `senstonote.py` (legacy)
- `senstonote_2h.py` (legacy)
- `sens2note_temp.py` (temporary)
- `exec_temp.py` (duplicate)
- `debug_function.py` (if unused)

### 🔄 **MERGE/CONSOLIDATE**
- `app_10fig_*.py` → single configurable ten_finger app
- `app_moving_window_*.py` → single configurable moving_window app
- `exec.py` + `run_glove.py` → unified execution system

### ✏️ **RENAME/REFACTOR**
- `senstonote_new.py` → `sensor_processor.py`
- `port_read.py` → `sensor_reader.py`
- `calibrate.py` → `calibration.py`

### 📝 **ENHANCE**
- Add comprehensive documentation to all files
- Implement consistent error handling
- Add configuration validation
- Create proper logging system

## Implementation Priority

1. **High Priority**: Remove legacy files, consolidate sensor-to-note logic
2. **Medium Priority**: Reorganize directory structure, standardize apps
3. **Low Priority**: Enhanced documentation, testing framework

This reorganization will create a cleaner, more maintainable codebase that's easier to extend with new applications and features.

---

## Setup & Installation

### Hardware Requirements
- Arduino-compatible microcontroller (tested with Arduino Uno/Nano)
- 5 flex sensors (Adafruit #1070 or similar)
- 5 force sensitive resistors (Adafruit #166 or similar)
- MPU-6050 IMU module
- Appropriate resistors for sensor conditioning
- Glove or mounting system for sensors

### Software Requirements
- Python 3.7+ 
- Arduino IDE
- Digital Audio Workstation (DAW) - GarageBand, Logic Pro, Ableton Live, etc.

### Python Dependencies
```bash
pip install mido python-rtmidi pyqt5 numpy matplotlib scikit-learn pyserial pyqtgraph
```

### Virtual MIDI Setup
**macOS**: Use Audio MIDI Setup to create an IAC Driver
**Windows**: Install loopMIDI or similar virtual MIDI driver
**Linux**: Use ALSA virtual MIDI ports

Update the `IACDriver` variable in `gluvn_python/__init__.py` with your virtual MIDI port name.

## Quick Start

### 1. Hardware Setup
1. Upload `arduino/sendsens/sendsens.ino` to your Arduino
2. Connect sensors according to pin definitions in the Arduino code
3. Set the `hand` variable in the Arduino code ('r' or 'l') - **this is the last time you'll need to edit Arduino code!**

### 2. EEPROM Calibration (Recommended)

The new EEPROM calibration system provides the best user experience with real-time visualization and no need to recompile Arduino code.

#### **Quick Calibration:**
```bash
cd gluvn_python

# Test Arduino connection first
python test_calibration_connection.py

# Run EEPROM calibration tool
python calibrate_eeprom.py
```

#### **Calibration Steps:**
1. **Select Hand**: Choose Right or Left from dropdown
2. **Open Hand**: Click "Open Hand" and straighten all fingers for 3 seconds
3. **Close Fist**: Click "Close Fist" and make a tight fist for 3 seconds  
4. **Press Fingers**: Click individual finger buttons and press each finger for 3 seconds
5. **Save**: Click "Save Calibration" to store data in Arduino EEPROM

#### **Advanced Features:**
- **Mode Toggle**: Switch between CALIBRATION (raw data) and NORMAL (calibrated data) modes
- **Real-time Plots**: Monitor all 10 sensors in real-time during calibration
- **Reset to Defaults**: Restore calibration.h defaults if needed
- **Backup Files**: Automatic generation of calibration.h backup files

### 3. Legacy Calibration (if needed)
```bash
# Traditional calibration method (still functional)
python calibrate.py
```

### 4. Test Sensor Reading
```bash
python plot_sensors.py
```
This opens a real-time GUI showing all sensor data.

### 5. Run a Basic Application
```bash
# Ten-finger instrument with acceleration volume control
python app_10fig_accel.py

# Moving window scale application  
python app_moving_window_flex.py

# Basic execution with multiple modes
python exec.py
```

## Current Status & Next Steps

### ✅ **Working Features**
- ⭐ **NEW**: EEPROM calibration system with real-time GUI
- ⭐ **NEW**: Dynamic Arduino mode switching via serial commands
- ⭐ **NEW**: Persistent calibration storage across power cycles
- Multi-threaded sensor data acquisition
- Real-time MIDI output to DAWs
- Multiple instrument applications
- Real-time sensor visualization
- Machine learning gesture recognition

### 🚧 **In Progress**
- Code reorganization and cleanup
- Unified configuration system
- Enhanced documentation

### 📋 **TODO**
- Remove legacy code files
- Implement proper error handling
- Add comprehensive test suite
- Create unified execution interface
- Improve ML model accuracy
- Add wireless communication support

## Contributing

This project is in active development and reorganization. Before contributing:
1. Review the reorganization plan above
2. Focus on the modern modules (`senstonote_modern.py`, `port_read.py`, etc.)
3. Avoid modifying legacy files marked for deletion
4. Add documentation to any new code

### Machine Learning Ideas:
- READUNDANCY: learn from flex sensors in file 
- Learning trigger threshold and velocity 
- Learning transition matrix for next note
- Use RNN
- Set up HMM for learning notes fingers.
- Problem with learning trigger is need for false labels: can I find a law (without learning)? Learn a constant across velcities.
- Use scales to label thumb-under for note and finger (when finger goes down but note up then its a thumb-under).
- Add sensor bending and IMU as conditions in a Bayesian Network
- Learn trigger on/off and velocity
- make transition matrix with features (previous_finger, present_finger, thumb_under (0, 1))

## Hardware Improvements:
- Test new resistors
- Make new board for both gloves
- Make wireless

## Potential Applications
- String instrument (e.g. violin) with pressure sensors for notes and accelerometer for vibrato
- pressure sensors with accelerometer data triggering percussion instruments by shaking the hand 
(audio using PyAudio)
- Accelerometer conductor baton for controlling playback speed in realtime (requires addition of 
playback capabilities)
- General language (gesture sequences) for triggering specific musical events
- Incorporating VMO for improvising with the gluvn!
- Simulating percussion

## Pitch Bend Mapping Reference

The pitch bend value is derived from the IMU2 (pitch) sensor, which is mapped on the Arduino as follows:

- Arduino: `pitch_cal = (pitch + 90) * 32767 / 90`  (so pitch in [-90, 90] maps to [0, 65535])

In the GUI, the pitch bend value is mapped to the MIDI range [-8192, 8192] using:

- `pitch_bend = (imu2 / 65535.0) * 16384 - 8192`

This ensures the pitch bend in the GUI matches the physical sensor mapping from the Arduino firmware.

