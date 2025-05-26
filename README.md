# GLUVN - Sensor Glove MIDI Controller

A hand-based musical instrument using flex sensors, pressure sensors, and IMU data to generate MIDI messages for digital audio workstations (DAWs).

## Hardware Overview

The glove contains:
- **5 flex sensors**: Measure finger bending angles
- **5 force sensitive resistors (FSRs)**: Measure fingertip pressure intensity  
- **1 IMU (MPU-6050)**: 3-axis gyroscope + 3-axis accelerometer for orientation and motion

## Current Repository Structure Analysis

### Arduino Code (`arduino/`)
- **`sendsens/sendsens.ino`**: Main Arduino sketch that reads sensors and transmits data via serial
- **`sendsens/calibration.h`**: Calibration constants for sensor normalization
- **`sendsens/mpu.h/cpp`**: MPU-6050 IMU interface and processing
- **`sendsens/I2Cdev.h/cpp`**: I2C communication library for IMU
- **Other files**: DMP motion processing libraries for IMU

### Python Core (`gluvn_python/`)

#### **Data Acquisition & Communication**
- **`port_read.py`**: ⭐ **CORE** - Multi-threaded serial port reader with parsing for both hands
- **`calibrate.py`**: Sensor calibration tool that generates calibration.h for Arduino

#### **Sensor-to-Note Transformation (Legacy Evolution)**
- **`senstonote.py`**: 🔴 **LEGACY** - Original sensor-to-note transformation logic
- **`senstonote_2h.py`**: 🔴 **LEGACY** - Two-hand version of senstonote
- **`senstonote_new.py`**: ⭐ **CURRENT** - Modern unified sensor processing with BaseApp class
- **`sens2note_temp.py`**: 🔴 **TEMPORARY** - Likely experimental/temporary file

#### **MIDI Output**
- **`midi_writer.py`**: ⭐ **CORE** - MIDI message generation and DAW communication

#### **Musical Mapping & Theory**
- **`mapper.py`**: ⭐ **CORE** - Scale generation, chord mapping, note window management

#### **Machine Learning & Gesture Recognition**
- **`learning.py`**: ML models for gesture-to-note mapping (thumb-under detection, transition matrices)
- **`ml_senstonote.py`**: ML-based sensor-to-note execution engine

#### **Application Modules (Sensor-to-Note Mappings)**
- **`app_10fig_accel.py`**: 10-finger instrument with acceleration-based volume control
- **`app_10fig_pitch.py`**: 10-finger instrument with pitch-based modulation
- **`app_10fig_inst.py`**: Basic 10-finger instrument
- **`app_harmonizer.py`**: Harmonic chord generation application
- **`app_jacob_choir.py`**: Complex choir-like application
- **`app_moving_window_*.py`**: Moving window applications for different sensors

#### **Execution & Orchestration**
- **`run_glove.py`**: ⭐ **CORE** - Main glove orchestration class (recording, printing, various apps)
- **`run2hands.py`**: Two-hand specific runner
- **`exec.py`**: Main execution script with multiple operation modes
- **`exec_temp.py`**: 🔴 **DUPLICATE** - Copy of exec.py
- **`app_runner.py`**: Application runner utility

#### **Data Analysis & Visualization**
- **`data_analysis.py`**: Comprehensive data analysis, plotting, and file I/O
- **`plot_sensors.py`**: ⭐ **RECENTLY UPDATED** - Real-time sensor visualization GUI

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
pip install mido python-rtmidi pyqt5 numpy matplotlib scikit-learn pyserial
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
3. Modify the `hand` variable in the Arduino code ('r' or 'l')

### 2. Calibration
```bash
cd gluvn_python
python calibrate.py
```
Follow the interactive prompts to calibrate your sensors.

### 3. Test Sensor Reading
```bash
python plot_sensors.py
```
This opens a real-time GUI showing all sensor data.

### 4. Run a Basic Application
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
- Multi-threaded sensor data acquisition
- Real-time MIDI output to DAWs
- Sensor calibration system
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
2. Focus on the modern modules (`senstonote_new.py`, `port_read.py`, etc.)
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

