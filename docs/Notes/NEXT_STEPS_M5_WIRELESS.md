# 🎯 GLUVN-M5 Wireless: Consolidated Next Steps

**Last Updated**: June 2026

**Focus**: Adapt legacy GLUVN Arduino code to M5Stick + wireless BLE

---

## Overview

This document consolidates all pending work across the project. The main goal is to complete the **wireless M5-GLUVN adaptation** and ensure all legacy code works seamlessly with BLE connectivity while maintaining compatibility with the original USB setup.

### Key Principles
- ✅ **Wireless First**: Primary target is M5Stick with BLE
- ✅ **USB Fallback**: Support USB serial for development/charging
- ✅ **No Code Duplication**: Reuse strategies and frameworks
- ✅ **Real Hardware**: All development/testing on actual M5Stick + PCB

---

## 🔴 CRITICAL PATH: Phase 1

### 1.1 Multiplexer Firmware Validation

**Tasks**:
- [x] **1.2.1** Test all 20 sensor channels (10 flex + 10 FSR) read correctly
  - Create a test to verify each channel
  - Check for channel cross-talk (one channel affecting another)
  - Validate 1000µs multiplexer settling time is sufficient
  
- [x] **1.2.2** Verify calibration data persists correctly across power cycles
  - Save calibration on M5 via NVS
  - Power cycle M5 and verify calibration loads
  - Test with different hand configurations
  
- [ ] **1.2.3** Performance validation on BLE link
  - Measure actual latency: sensor → M5 → BLE → PC
  - Compare BLE latency vs USB (should be <50ms typical)
  - Run `benchmark_bandwidth.py` in BLE mode for 1+ minute
  - Verify <1% packet loss

---

### 1.2 Complete BLE Integration for All Applications

**Tasks**:
- [ ] **1.1.1** Update `ten_finger_gui_with_sensors.py` to support BLE mode with command-line flag
  ```bash
  python examples/ten_finger_gui_with_sensors.py --ble
  python examples/ten_finger_gui_with_sensors.py --usb  # fallback
  ```
  - Add BLE device discovery UI (BleakScanner)
  - Add connection status indicator
  - Handle BLE disconnection gracefully
  
- [ ] **1.1.2** Create BLE wrapper for `Reader` class in `port_read.py`
  - Make Reader class detect and auto-select BLE vs USB
  - Implement fallback logic (try BLE, fall back to USB if unavailable)
  - Test with both left and right M5 devices simultaneously
  
- [ ] **1.1.3** Update `app_runner.py` to accept BLE mode parameter
  ```python
  AppRunner.run_app('base', config_path, use_ble=True)
  ```
  
- [ ] **1.1.4** Verify BLE calibration tool works end-to-end
  - Test `calibrate_eeprom_BLE.py` with actual M5Stick
  - Verify calibration data persists in M5 NVS
  - Confirm calibration loads on next boot

---


### 1.3 Update All Example Applications to Support BLE

**Examples to Update**:
- [ ] **1.3.1** `ten_finger_gui_with_modulation.py` → Add `--ble` flag
- [ ] **1.3.2** `ten_finger_gui_with_pitchbend.py` → Add `--ble` flag
- [ ] **1.3.3** `single_glove_stage_gui.py` → Add `--ble` flag (performance variant)
- [ ] **1.3.4** Create `examples/ten_finger_gui_ble_only.py` (optimized for wireless)

**Each example needs**:
- BLE device discovery
- Graceful connection/disconnection handling
- Status indicator (connected/searching/error)
- Clear error messages if BLE unavailable

---

## 🟠 HIGH PRIORITY: Phase 2

### 2.1 Configuration System for M5-Specific Hardware

**Tasks**:
- [ ] **2.1.1** Create `configs/m5_hardware_config.py` with M5-specific defaults
  ```python
  class M5HardwareConfig(BaseConfig):
      def __init__(self):
          super().__init__()
          self.adc_resolution = 12  # 4096 levels vs 10-bit original
          self.multiplexer_enabled = True
          self.settling_time_us = 100
          self.sensor_averaging = 2  # oversampling
          self.ble_mode = True
          self.ble_latency_compensation = True  # smooth out BLE jitter
  ```
  
- [ ] **2.1.2** Create presets for common M5-GLUVN scenarios
  - `configs/presets/m5_basic.json` - Simple flex trigger
  - `configs/presets/m5_modulation.json` - With volume/pitch control
  - `configs/presets/m5_dual_hand.json` - Optimized for two M5Sticks
  - `configs/presets/m5_performance.json` - High-performance stage mode

- [ ] **2.1.3** Update `__init__.py` to use M5-appropriate defaults
  - Check if running on M5 vs Arduino based on port names
  - Auto-load M5 config if M5 device detected

---

### 2.2 Optimize for BLE Latency & Jitter

**Tasks**:
- [ ] **2.2.1** Implement BLE latency compensation in `SensorProcessingThread`
  - Add predictive smoothing for jittery BLE data
  - Increase internal buffer to handle variable packet timing
  - Test with moving average of 2-3 frames
  
- [ ] **2.2.2** Create BLE-specific GUI update strategy
  - Use rolling buffer instead of single frame
  - Reduce GUI update rate to match typical BLE frame arrival (~100 Hz)
  - Add visual indicator of BLE data freshness
  
- [ ] **2.2.3** Battery monitoring
  - Read M5 battery level via M5Unified
  - Display battery % in GUI
  - Warn when battery <20%
  - Consider reducing BLE update rate when on battery saver mode

---

### 2.3 Multiplexer Documentation & Troubleshooting

**Tasks**:
- [ ] **2.3.1** Create detailed PCB assembly guide: `docs/PCB_ASSEMBLY.md`
  - Schematics with annotated part placement
  - Multiplexer channel-to-sensor mapping diagram
  - Wiring checklist (all 20 sensors)
  - Common soldering issues and fixes
  
- [ ] **2.3.2** Create multiplexer debugging guide: `docs/MULTIPLEXER_TROUBLESHOOTING.md`
  - How to test each MUX channel independently
  - Common failure modes (open/short circuits)
  - ADC reading ranges for healthy sensors
  - Settling time requirements and verification
  
- [ ] **2.3.3** Create filter network design document: `docs/FILTER_NETWORKS.md`
  - Derivation of 47K/100nF vs 10K/10nF values
  - Time constant calculations
  - PCB trace impedance considerations
  - When to adjust capacitor values

---

## 🟡 MEDIUM PRIORITY: Phase 3

### 3.1 Refactor Legacy Application Files
**Status**: Multiple `app_*.py` files have duplicated code

**Current Situation**:
- `app_10fig_accel.py` - Uses old AccelVolume approach
- `app_10fig_flex.py` - Old flex-only triggering
- `app_harmonizer.py` - Custom MovingWindow implementation
- `app_jacob_choir.py` - Choir modulation (different from current system)

**Tasks**:
- [ ] **3.1.1** Audit all `app_*.py` files for M5 compatibility
  - Identify which apps still work with USB
  - Which can be migrated to BLE easily
  - Which require significant refactoring

- [ ] **3.1.2** Migrate `app_harmonizer.py` to new modulation system
  - Extract chord logic into reusable mapper
  - Use CompositeModulation for multi-voice control
  - Update to use M5 built-in IMU
  
- [ ] **3.1.3** Update `app_jacob_choir.py` for M5 IMU
  - Adapt to use M5Stick's gyroscope/accelerometer
  - Test choir modulation with actual M5 hardware
  
- [ ] **3.1.4** Create unified launcher for all apps
  ```python
  # python run_app.py --app harmonizer --ble --hand r
  # python run_app.py --app choir --ble --hand l --config preset
  ```

---

### 3.2 Data Recording & Playback for M5-BLE
**Status**: Recording logic exists, but not BLE-optimized

**Tasks**:
- [ ] **3.2.1** Create BLE-specific data recorder
  - Record BLE frames directly (with sequence numbers for packet loss detection)
  - Add timestamp for each frame
  - Store metadata (hand, battery %, calibration data)
  - Compress recordings (BLE frames are small already)

- [ ] **3.2.2** Implement playback system
  - Replay recorded BLE sessions through GUI
  - Use timestamps to simulate real latency
  - Useful for debugging connection issues
  - Generate test datasets for algorithm development

- [ ] **3.2.3** Create analysis tools for BLE performance
  - Analyze packet loss patterns
  - Measure actual latency vs theoretical
  - Detect connection quality issues
  - Export metrics for reports

---

### 3.3 Performance Profiling for M5
**Status**: Bandwidth benchmark exists ✅, but CPU/memory profiling missing

**Tasks**:
- [ ] **3.3.1** Profile M5 CPU usage during operation
  - Monitor in Arduino firmware or via M5Unified
  - Report CPU % to GUI via BLE status message
  - Identify bottlenecks (multiplexer, BLE, sensor processing)
  
- [ ] **3.3.2** Profile memory usage on M5
  - Heap usage over time (detect memory leaks)
  - Stack depth for each thread
  - PSRAM usage if available
  
- [ ] **3.3.3** PC-side profiling for Python GUI
  - Profile latency of BLE frame processing
  - Identify GUI bottlenecks
  - Optimize PyQt5 rendering for real-time updates

---

## 🟢 MEDIUM-LOW PRIORITY: Phase 4

### 4.1 Advanced Features: M5-Specific Capabilities
**Status**: Not started

**Possibilities** (choose based on priority):

- [ ] **4.1.1** WiFi fallback mode
  - If BLE unreliable in environment, switch to WiFi MIDI
  - Useful for crowded 2.4 GHz environments
  - Lower latency than BLE

- [ ] **4.1.2** Gesture recognition using M5 IMU
  - Detect hand poses (closed fist, open hand, pointing)
  - Use as mode selector or expression control
  - Extends musical vocabulary

- [ ] **4.1.3** Multi-glove synchronization
  - Left and right M5 devices synchronize timing
  - Detect if one glove disconnects
  - Automatic re-sync when reconnecting

- [ ] **4.1.4** Machine learning calibration
  - Auto-detect flex sensor range on startup
  - Learn individual playing style
  - Adaptive trigger thresholds per performer

---

### 4.2 Alternative Connectivity Options
**Status**: BLE is primary, but alternatives could be useful

**Optional Tasks**:
- [ ] **4.2.1** Implement WiFi MIDI support (for unreliable BLE environments)
- [ ] **4.2.2** Add USB MIDI output to M5 firmware (USB as MIDI device)
- [ ] **4.2.3** Create network mode (OSC over UDP for remote playing)

---

## 🔵 LOW PRIORITY: Phase 5 (Future)

### 5.1 GUI Enhancements
- [ ] Visualize BLE signal strength
- [ ] Real-time IMU visualization (orientation, acceleration)
- [ ] Waveform display of sensor raw data
- [ ] Gesture recording and playback UI

### 5.2 Integration with DAWs
- [ ] Native VST plugin integration
- [ ] Ableton Live controller support
- [ ] Logic Pro compatibility
- [ ] ReWire support for multi-app sync

### 5.3 Mobile Companion App
- [ ] iOS/Android BLE controller
- [ ] Remote configuration and calibration
- [ ] Performance analytics dashboard

---

## 📚 DOCUMENTATION: Phase 6 (Ongoing, end of each feature)

### Critical Documentation Needs (Must Complete by Final Release)

- [ ] **6.1** `docs/BLE_SETUP.md` - Complete BLE connection guide
  - M5 device discovery
  - Pairing process
  - Connection troubleshooting
  - Multi-device setup (left + right)

- [ ] **6.2** `docs/M5_HARDWARE_GUIDE.md` - Complete hardware setup
  - M5Stick specs and pin assignments
  - PCB assembly with schematics
  - Sensor connections and wiring
  - Power management and charging

- [ ] **6.3** `docs/CALIBRATION_GUIDE_M5.md` - Wireless calibration
  - BLE calibration tool usage
  - Interpreting calibration values
  - Re-calibration procedures
  - Troubleshooting calibration issues

- [ ] **6.4** `docs/ARCHITECTURE_M5.md` - System architecture for wireless
  - Data flow diagram (sensor → M5 → BLE → PC)
  - Frame format specification
  - Latency analysis and optimization
  - Failure modes and recovery

- [ ] **6.5** `docs/PERFORMANCE_METRICS.md` - Benchmarks and profiling
  - Latency measurements (BLE vs USB)
  - Throughput and bandwidth usage
  - CPU/memory profiling results
  - Battery life estimates

- [ ] **6.6** `docs/FIRMWARE_DEVELOPMENT.md` - M5 firmware guide
  - Building and flashing firmware
  - BLE service implementation details
  - Adding new sensor types
  - Extending multiplexer support

- [ ] **6.7** `docs/TROUBLESHOOTING_M5_BLE.md` - Common issues
  - BLE connection failures
  - Multiplexer channel failures
  - Calibration issues
  - Performance problems
  - Known limitations

- [ ] **6.8** `docs/MIGRATION_FROM_ARDUINO.md` - For existing GLUVN users
  - What's different in M5-GLUVN
  - How to adapt existing code
  - Configuration file migration
  - Playing styles differences (BLE latency)

- [ ] **6.9** Update main READMEs
  - README.md - Add M5 compatibility note
  - README_M5_GLUVN.md - Ensure complete and current
  - Link all documentation appropriately

- [ ] **6.10** Code documentation
  - Docstrings for all BLE-related functions
  - Inline comments for multiplexer logic
  - API documentation for port_read_BLE
  - Examples in code comments

---

## ✅ Quick Reference: What's Already Done

### Architecture & Core Framework
- ✅ Core trigger strategies (HysteresisTrigger, MultiSensor, etc.)
- ✅ Modulation strategies (Volume, Pitch, Window, Choir)
- ✅ Visualization components (PyQt5 widgets)
- ✅ Configuration system (Base, Advanced configs)
- ✅ Unified app framework

### M5 Specific
- ✅ M5 firmware with BLE support (`sendsens.ino`)
- ✅ Custom PCB with CD4067 multiplexer
- ✅ BLE reader implementation (`port_read_BLE.py`)
- ✅ BLE calibration tool (`calibrate_eeprom_BLE.py`)
- ✅ Bandwidth benchmarking tool (`benchmark_bandwidth.py`)
- ✅ BLE device configuration (`__init__.py` with BLE names/UUIDs)

### Documentation
- ✅ README_WIRED.md (original Arduino version)
- ✅ README.md (wireless M5 version)
- ✅ Existing guides

---
