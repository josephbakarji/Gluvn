# 🎼 GLUVN - Original Wired Arduino System

This file documents the **legacy wired GLUVN stack**: the Arduino-based glove system that sends sensor data over USB serial to the Python runtime, which then maps gestures to MIDI notes.

**⚠️ NOTE**: This guide covers the **original wired setup only**. BLE/M5 files belong to the wireless rewrite and are intentionally excluded here.

## 📋 Project Overview

**GLUVN** is a wearable glove system that enables users to play music through intuitive hand gestures. The system combines:

- **10 Flex Sensors** (5 per hand) for finger touch/bend detection
- **10 Pressure Sensors** (5 per hand) for dynamic expression control
- **External IMU** (Inertial Measurement Unit) for volume/modulation control
- **Arduino-based Hardware** with USB serial connectivity
- **Hysteresis-based Trigger Logic** for robust note triggering
- **Python-based Processing** with configurable musical scales and parameters
- **MIDI Output** for integration with DAWs and music software

## 🧭 Legacy Wired Data Flow

```
Arduino sensor glove → USB serial → port_read.py → senstonote_modern.py → mapper.py → midi_writer.py → DAW
```

That path is the core of the wired system:
- the Arduino firmware samples sensors and streams values,
- the Python reader parses the stream,
- the application layer converts triggers into musical events,
- the MIDI writer sends the result to external music software.

## 📚 Wired Code Inventory

### 1. Core runtime files

| File | Purpose |
|---|---|
| `gluvn_python/__init__.py` | Central configuration for the wired system: COM ports, baud rate, filesystem paths, and MIDI port names. |
| `gluvn_python/port_read.py` | Main serial reader for the wired gloves. Opens USB ports, runs reader/parser threads, supports both hands, and can replay saved sensor files. |
| `gluvn_python/data_analysis.py` | Data persistence and analysis utilities. Saves recordings, reads experiment folders, and drives plotting/statistics workflows. |
| `gluvn_python/mapper.py` | Musical mapping engine. Converts note names to MIDI numbers, generates scales, and builds fixed or moving-window note layouts. |
| `gluvn_python/midi_writer.py` | MIDI output layer. Sends note on/off, pitch bend, aftertouch, and control-change messages to the DAW or virtual MIDI port. |
| `gluvn_python/senstonote_modern.py` | Main sensor-to-note application framework. Contains the trigger thread, note mapping orchestration, and the `BaseApp` / `MovingWindow` classes. |
| `gluvn_python/app_runner.py` | Command-line launcher for running wired app variants from configuration files. |
| `gluvn_python/exec.py` | Convenience script for running analysis, learning, plotting, and recording tasks from one place. |

### 2. Main wired application files

| File | Purpose |
|---|---|
| `gluvn_python/app_10fig_inst.py` | Basic ten-finger instrument demo. Uses the wired glove as a simple note trigger surface. |
| `gluvn_python/app_10fig_accel.py` | Ten-finger app that adds accelerometer-based volume control to the wired setup. |
| `gluvn_python/app_10fig_pitch.py` | Ten-finger app that adds pitch bend driven by IMU data. |
| `gluvn_python/app_moving_window_flex.py` | Moving-window version where finger motion selects the active note window. |
| `gluvn_python/app_moving_window_accel.py` | Moving-window version with accelerometer-based expression control. |
| `gluvn_python/app_moving_window_flex_press3.py` | Experimental moving-window variant that combines flex and pressure behavior for richer triggering. |
| `gluvn_python/app_harmonizer.py` | Harmony/chord-focused app for turning finger gestures into richer musical structures. |
| `gluvn_python/app_jacob_choir.py` | Choir-style performance app that explores multi-voice interaction and broader musical control. |

### 3. Calibration, debugging, and analysis tools

| File | Purpose |
|---|---|
| `gluvn_python/calibrate_eeprom.py` | Wired calibration helper for collecting sensor ranges and storing calibration data. |
| `gluvn_python/debug_sensors.py` | Console diagnostic script that prints live sensor values to verify wiring and responsiveness. |
| `gluvn_python/plot_sensors.py` | Real-time plotting GUI for inspecting glove channels and debugging signal quality. |
| `gluvn_python/simple_sensor_test.py` | Minimal change-detection test for quickly confirming that any sensor movement is being seen. |
| `gluvn_python/musicFun/MusicFunction.py` | Small musical helper functions, including note-name parsing and MIDI-number conversion. |
| `gluvn_python/musicFun/GenerateDict.py` | Generates note↔MIDI lookup tables and writes them to CSV for later reuse. |

### 4. Legacy and reference scripts

| File | Purpose |
|---|---|
| `gluvn_python/legacy/*.py` | Historical prototypes, older app versions, and experiment scripts kept as reference material. |
| `gluvn_python/testfiles/*.py` | Experimental test harnesses for plotting, MIDI reading, Qt prototypes, and thread experiments. |
| `gluvn_python/testfiles/readmidi_test/*.py` | Small MIDI-reading proof-of-concept scripts. |

### 5. Original firmware

| File | Purpose |
|---|---|
| `arduino/sendsens/sendsens.ino` | Arduino firmware for the wired glove. Reads the glove sensors and streams sensor frames to the Python side over USB serial. |

## 🔎 What each part does in the wired stack

### `__init__.py`
Defines the shared constants used by the wired system: port names, baud rate, directories for experiments and figures, and the MIDI driver name.

### `port_read.py`
Owns the sensor acquisition pipeline. It opens the serial link, creates reader/parser threads, supports both left and right gloves, and can also replay saved datasets when no hardware is attached.

### `senstonote_modern.py`
Contains the core behavior of the wired instrument:
- `SensorProcess` turns raw sensor values into trigger events with hysteresis.
- `BaseApp` coordinates reader threads, note mapping, and MIDI output.
- `MovingWindow` adds the more advanced window-based musical behavior.

### `mapper.py`
Turns sensor-triggered finger states into actual note numbers. This is where scales, root notes, and note windows are defined.

### `midi_writer.py`
Sends musical actions to external software. It is the final output layer for note on/off, pitch bend, aftertouch, and control changes.

### `data_analysis.py`
Handles saved sessions, plots, and statistics. It is used for tuning thresholds, analyzing recordings, and comparing note behavior against sensor input.

### `app_*.py`
Each `app_*.py` file is a focused performance or experiment entry point built on top of the shared wired runtime. They differ mainly in how they map fingers, motion, pitch, and volume into music.

### `legacy/`
Contains older experiments and alternate implementations. These files are useful for historical reference, but they are not the main runtime path.

### `testfiles/`
Contains throwaway or experimental scripts used while developing plots, MIDI readers, threading tests, and GUI prototypes.

### `arduino/sendsens/sendsens.ino`
Reads the glove hardware and produces the serial stream consumed by `port_read.py`. In the wired system, this firmware is the bridge between the sensors and the Python application.

## 🎯 Key Features

### Core Functionality
- ✅ **Real-time Sensor Processing** - Low-latency finger detection and MIDI generation via USB serial
- ✅ **Musical Scale Support** - Major, Minor, Pentatonic scales with customizable root notes
- ✅ **Volume Control** - External accelerometer-based dynamic volume modulation
- ✅ **Pitch Bend** - IMU-controlled pitch modulation for expressive playing
- ✅ **Configurable Thresholds** - Per-hand and per-sensor sensitivity adjustment
- ✅ **Hysteresis Filtering** - Eliminates sensor noise and false triggers
- ✅ **Clean Architecture** - Strategy pattern for modular, extensible design
- ✅ **USB Connectivity** - Wired connection for reliable, low-latency performance

### Software Features
- 📊 **GUI Dashboard** - Real-time visualization of all sensor inputs
- 🎹 **Demo Mode** - Learn the system without hardware
- ⚙️ **User-Configurable Parameters** - Adjust all settings via GUI
- 📈 **Data Analysis Tools** - Benchmarking, plotting, and debugging utilities
- 🧪 **Test Suite** - Unit tests for trigger strategies and components
- 🔄 **Multiple Application Variants** - Base, moving window, harmonizer modes

## 📁 Project Structure

```
GLUVN-M5/
├── gluvn_python/              # Main Python application
│   ├── core/                  # Core business logic
│   │   └── strategies/        # Trigger strategies and MIDI mapping
│   ├── configs/               # Configuration management
│   ├── visualization/         # GUI and real-time display (PyQt5)
│   ├── examples/              # Pre-built applications
│   ├── tests/                 # Unit tests
│   ├── mapper.py              # MIDI note mapping engine
│   ├── app_runner.py          # Unified application launcher
│   ├── port_read.py           # Serial sensor reader
│   └── senstonote_modern.py   # Core application framework
│
├── arduino/                   # Arduino firmware and sensor code
│   └── sendsens/              # Sensor reading firmware
│
├── docs/                      # Project documentation
│   ├── Gluvn idea.txt         # Original concept notes
│   ├── chords-combos.txt      # Musical chord definitions
│
├── figures/                   # Design documents and diagrams
│   ├── gluvn_dataflow.pdf
│   ├── gluvn_design.pdf
│   └── glove_pins.graffle
│
└── literature/                # Research papers and references
```

## 🚀 Quick Start

### Prerequisites
- Python 3.7+
- Arduino (for hardware version)
- MIDI software (GarageBand, LoopMIDI, etc.)

### Installation

```bash
# Clone the repository
cd gluvn_python

# Install Python dependencies
pip install numpy scipy
pip install PyQt5          # For GUI
pip install pygame         # For MIDI output

# Verify installation
python test_basic_imports.py
```

### Run the Demo

```bash
cd gluvn_python
python examples/ten_finger_demo_pyqt.py
```

**What to expect:**
- GUI with 10 finger status indicators
- Configuration controls for threshold, hysteresis, scale, root note
- Start/Stop/Demo buttons
- Click finger buttons to trigger notes

### Run with Hardware

```bash
# Connect GLUVN glove via USB
python examples/ten_finger_gui_with_sensors.py
```

## 🎛️ Configuration & Tuning

### Key Parameters

All parameters are adjustable via the GUI dashboard:

| Parameter | Range | Default | Purpose |
|-----------|-------|---------|---------|
| **Threshold** | 50-500 | 120 | Minimum sensor value to trigger a note |
| **Hysteresis** | 1-50 | 10 | Prevents flickering near trigger threshold |
| **Sensor Type** | flex/press | flex | Which sensor type triggers notes |
| **Base Volume** | 0-100 | 20 | Minimum volume (always present) |
| **Volume Sensitivity** | 5000-30000 | 15000 | Accelerometer → Volume mapping |
| **Pitch Bend Range** | 1000-8192 | 8192 | Max pitch bend (MIDI units) |

### Default Note Layout (C Major)

```
Left Hand:                Right Hand:
Thumb:  G2 (55)          Thumb:  C4 (60) ← Middle C
Index:  A2 (57)          Index:  D4 (62)
Middle: B2 (59)          Middle: E4 (64)
Ring:   C3 (52)          Ring:   F4 (65)
Pinky:  D3 (53)          Pinky:  G4 (67)
```

### Calibration Steps

1. **Find Trigger Threshold**
   - Start demo/hardware mode
   - Adjust threshold until finger bends trigger reliably
   - Typical range: 100-250 for flex sensors

2. **Optimize Hysteresis**
   - Set hysteresis to 10 initially
   - Increase if lights flicker on/off (15-30)
   - Decrease if response feels sluggish (5-8)

3. **Test Musical Settings**
   - Try different scales (Major/Minor/Pentatonic)
   - Use C Major for familiar sound
   - Use Pentatonic for improvisation

## 🏗️ Architecture

### Design Principles
- **Strategy Pattern** - Pluggable trigger, mapping, and processing strategies
- **Separation of Concerns** - Core logic, visualization, GUI cleanly separated
- **Configuration-Driven** - All behavior controlled via JSON configs
- **Test-Driven** - Unit tests verify strategy implementations

### Data Flow

```
Sensor Hardware (Arduino)
         ↓
Serial Port Reader (port_read.py)
         ↓
Sensor Processing Thread (visualization/sensor_threads.py)
         ↓
Trigger Strategy (core/strategies/trigger_strategies.py)
    ├─ HysteresisTrigger
    └─ (extensible for custom strategies)
         ↓
Mapper Strategy (mapper.py)
    ├─ BasicMapper (fixed note layout)
    ├─ HarmonizerMapper (chord generation)
    └─ (extensible for custom mappings)
         ↓
MIDI Output (pygame, midiutil, or system MIDI)
         ↓
Music Software (DAW/synthesizer)
```

### Core Components

#### **TriggerStrategy** (`core/strategies/`)
Converts sensor data to discrete on/off events:
- Applies threshold filtering
- Implements hysteresis to eliminate noise
- Manages trigger state machine
- Returns switch events (1/-1/0)

#### **MappingStrategy** (`mapper.py`)
Maps triggers to musical notes:
- BasicMapper: Fixed note layout
- HarmonizerMapper: Chord-based generation
- Supports Major/Minor/Pentatonic scales
- Handles both hands independently

#### **Application Framework** (`senstonote_modern.py`)
Coordinates all components:
- Manages sensor threads
- Handles MIDI output
- Processes GUI signals
- Implements application lifecycle

## 📊 Available Applications

### Pre-built Examples

| Application | Purpose | Features |
|-------------|---------|----------|
| `ten_finger_demo_pyqt.py` | No hardware demo | UI learning, manual triggering |
| `ten_finger_gui_with_sensors.py` | Basic hardware mode | Sensor reading, basic note triggering |
| `ten_finger_gui_with_modulation.py` | Volume/Pitch control | Accelerometer modulation |
| `ten_finger_gui_with_pitchbend.py` | Pitch bend mode | IMU pitch control |
| `single_glove_stage_gui.py` | Single hand performance | Optimized for one-handed play |

### Custom Applications

Build your own application using `AppRunner`:

```python
from app_runner import AppRunner
from configs.base_config import BaseConfig

# Create config
config = BaseConfig(
    threshold=150,
    hysteresis=10,
    scale='minor',
    root_note='A'
)

# Run application
AppRunner.run_app('base', config)
```

## 🧪 Testing

Run the test suite to verify system functionality:

```bash
# Test imports and basic setup
python test_basic_imports.py

# Run unit tests
python -m pytest tests/

# Test trigger strategies specifically
python -m pytest tests/test_trigger_strategies.py -v
```

## 📖 Documentation

- **[GETTING_STARTED.md](gluvn_python/GETTING_STARTED.md)** - Comprehensive quickstart guide
- **[USER_CONFIGURABLE_PARAMETERS.md](gluvn_python/USER_CONFIGURABLE_PARAMETERS.md)** - Detailed parameter tuning
- **[ARCHITECTURE_VERIFICATION.md](gluvn_python/ARCHITECTURE_VERIFICATION.md)** - Technical architecture details
- **[CLEAN_ARCHITECTURE_SUCCESS.md](gluvn_python/CLEAN_ARCHITECTURE_SUCCESS.md)** - Design patterns verification
- **[examples/README.md](gluvn_python/examples/README.md)** - Application examples guide

## 🔧 Hardware Setup

### Sensor Hardware Requirements

- **Microcontroller**: Arduino or compatible
- **Flex Sensors**: 5 per hand (10 total)
- **Pressure Sensors**: Optional, for expressive control
- **IMU**: 5-axis accelerometer/gyroscope (optional, for volume/pitch)
- **ADC**: Analog-to-Digital Converter (Arduino built-in or external)
- **USB**: For serial communication with PC

### Arduino Firmware

Located in `arduino/sendsens/`:
- Reads all analog sensors
- Sends comma-separated values over serial
- Baud rate: 9600 (configurable)

### Wiring

See `figures/micropins.png` and `figures/glove_pins.graffle` for detailed pinouts.

## 🎵 Musical Features

### Supported Scales
- **Major**: Bright, happy sound (intervals: 2-2-1-2-2-2-1)
- **Minor**: Dark, introspective sound (intervals: 2-1-2-2-1-2-2)
- **Pentatonic**: Perfect for improvisation (5-note scale)
- **Custom**: Add your own scales by modifying `mapper.py`

### MIDI Output Options
- System MIDI output (Windows, macOS, Linux)
- Virtual MIDI ports (LoopMIDI, etc.)
- pygame for quick testing
- midiutil for recording

### Expression Control
- **Volume**: Controlled by hand acceleration (0-127 MIDI velocity)
- **Pitch Bend**: Controlled by IMU rotation data
- **Modulation**: Accelerometer-based vibrato effects


## 📝 License

MIT License - See [gluvn_python/LICENSE](gluvn_python/LICENSE)

Copyright (c) 2016 josephbakarji


**Made with 🎵 by the GLUVN team**

*Transform your hand into a musical instrument.*
