# GLUVN Ten Finger GUI Applications

This directory contains GUI applications for testing and using the GLUVN ten finger trigger system with visual feedback.

## Applications

### 1. `ten_finger_demo.py` - Demo Version (tkinter)
**No hardware required** - Perfect for testing the interface and functionality.

**Features:**
- Simulated sensor data with realistic patterns
- Click-to-trigger finger lights for manual testing
- Auto demo sequences showing different patterns
- All configuration options functional
- MIDI output (if MIDI device available)

**Usage:**
```bash
cd gluvn_python/examples
python ten_finger_demo.py
```

### 2. `ten_finger_demo_pyqt.py` - Demo Version (PyQt5) ⭐ **Recommended**
**No hardware required** - PyQt5 version matching existing sensor plotting applications.

**Features:**
- Same functionality as tkinter demo
- PyQt5 interface consistent with other GLUVN tools
- Better performance and visual appearance
- Professional look and feel

**Usage:**
```bash
cd gluvn_python/examples
python ten_finger_demo_pyqt.py
```

### 3. `ten_finger_gui.py` - Hardware Version (tkinter)
**Requires GLUVN hardware** - For use with actual sensor gloves.

**Features:**
- Real-time sensor data processing
- Live trigger visualization
- MIDI note output
- Configurable thresholds and musical settings

**Usage:**
```bash
cd gluvn_python/examples
python ten_finger_gui.py
```

### 4. `ten_finger_gui_pyqt.py` - Hardware Version (PyQt5) ⭐ **Recommended**
**Requires GLUVN hardware** - PyQt5 version for use with actual sensor gloves.

**Features:**
- Same functionality as tkinter hardware version
- PyQt5 interface consistent with other GLUVN tools
- Better error handling and user feedback
- Professional appearance

**Usage:**
```bash
cd gluvn_python/examples
python ten_finger_gui_pyqt.py
```

## Recommended Workflow

### For Testing (No Hardware)
1. **Start with PyQt5 demo**: `python ten_finger_demo_pyqt.py`
2. **Alternative**: `python ten_finger_demo.py` (if PyQt5 issues)

### For Hardware Use
1. **Use PyQt5 version**: `python ten_finger_gui_pyqt.py`
2. **Alternative**: `python ten_finger_gui.py` (if PyQt5 issues)

## GUI Interface

### Main Display
- **10 Finger Lights**: Visual indicators for each finger (5 per hand)
  - 🔴 Dark Red: Finger not triggered
  - 🟢 Green: Finger triggered
- **Note Names**: Shows which MIDI note each finger will play
- **Hand Labels**: Left Hand and Right Hand sections

### Control Panel

#### Buttons
- **Start Demo/Sensors**: Begin processing sensor data
- **Stop Demo/Sensors**: Stop processing and reset all lights
- **Auto Demo**: Run automated demonstration sequences (demo versions only)

#### Configuration Options
- **Threshold**: Sensor value threshold for triggering (50-500)
  - Higher values = need more finger bend to trigger
  - Lower values = more sensitive triggering
- **Hysteresis**: Noise reduction factor (1-50)
  - Higher values = more stable triggering, less sensitive to noise
  - Lower values = more responsive but may have false triggers
- **Scale**: Musical scale selection
  - Major, Minor, Pentatonic
- **Root Note**: Base note for the scale
  - C, C#, D, D#, E, F, F#, G, G#, A, A#, B

### Status Bar
Shows current system status and recent actions.

## Getting Started

### 1. Start with the PyQt5 Demo
```bash
cd gluvn_python/examples
python ten_finger_demo_pyqt.py
```

1. Click "Start Demo" to begin processing
2. Click on individual finger lights to trigger them manually
3. Try the "Auto Demo" button to see automated sequences
4. Experiment with different threshold and scale settings

### 2. Test with Hardware (if available)
```bash
cd gluvn_python/examples
python ten_finger_gui_pyqt.py
```

1. Ensure your GLUVN hardware is connected
2. Click "Start Sensors" to begin reading from hardware
3. Move your fingers to trigger the sensors
4. Watch the lights respond to your finger movements

## Configuration Guide

### Optimal Threshold Settings
- **Flex Sensors**: Start with 200, adjust based on your sensor calibration
  - If lights don't trigger: Lower the threshold (150-180)
  - If lights trigger too easily: Raise the threshold (220-300)

### Hysteresis Settings
- **Start with 10-15** for most applications
- **Increase if you see flickering**: Higher hysteresis reduces noise
- **Decrease for faster response**: Lower hysteresis for quick triggering

### Musical Settings
- **C Major**: Good default for testing (all white keys)
- **A Minor**: Natural minor scale starting on A
- **Pentatonic**: 5-note scale, sounds good in any combination

## Troubleshooting

### Demo Versions
**Problem**: Demo doesn't start
- **Solution**: Check that all dependencies are installed
- **Check**: Python 3.7+ with tkinter/PyQt5, numpy

**Problem**: No MIDI output
- **Solution**: Install MIDI software or connect MIDI device
- **Note**: Visual feedback will still work without MIDI

### Hardware Versions
**Problem**: "Sensor reader not available" error
- **Solution**: 
  1. Check hardware connection (USB/serial)
  2. Verify `port_read.py` can access your device
  3. Check device permissions on Linux/Mac

**Problem**: Lights don't respond to finger movement
- **Solution**:
  1. Adjust threshold values (try 150-250 range)
  2. Check sensor calibration
  3. Verify flex sensors are working properly

**Problem**: Lights flicker or trigger randomly
- **Solution**:
  1. Increase hysteresis value (15-25)
  2. Check for loose sensor connections
  3. Reduce electrical noise in environment

## Technical Details

### Sensor Processing Pipeline
```
Sensor → Trigger Strategy → Mapping Strategy → MIDI Output
                ↓
           Visual Display
```

### Key Classes
- **`HysteresisTrigger`**: Processes sensor data with threshold and hysteresis
- **`BasicMapper`**: Maps finger triggers to musical notes
- **`FingerLight`**: Visual representation of finger state
- **`TenFingerGUI/Demo`**: Main application classes

### Default Note Mappings
**Left Hand** (C Major):
- Thumb: G2, Index: A2, Middle: B2, Ring: C3, Pinky: D3

**Right Hand** (C Major):
- Thumb: E3, Index: F3, Middle: G3, Ring: A3, Pinky: B3

## Advanced Usage

### Custom Configuration
You can modify the default settings by editing the configuration in the code:

```python
# Example: Custom threshold settings
self.trigger_config = {
    'thresholds': {'flex': 180},    # Lower threshold
    'hysteresis': {'flex': 15},     # Higher hysteresis
    'trigger_sensors': {'l': 'flex', 'r': 'flex'}
}
```

### Adding New Scales
To add custom scales, modify the `mapper.py` file:

```python
scales = {
    'major': [2, 2, 1, 2, 2, 2, 1],
    'minor': [2, 1, 2, 2, 1, 2, 2],
    'your_scale': [2, 1, 3, 1, 2, 2, 1]  # Your custom intervals
}
```

## Framework Comparison

### PyQt5 Versions (Recommended)
- **Pros**: Consistent with existing GLUVN tools, better performance, professional appearance
- **Cons**: Requires PyQt5 installation
- **Best for**: Production use, integration with other GLUVN applications

### tkinter Versions
- **Pros**: Built into Python, no additional dependencies
- **Cons**: Less polished appearance, different from other GLUVN tools
- **Best for**: Quick testing, systems without PyQt5

## Next Steps

1. **Test the PyQt5 demo version** to understand the interface
2. **Try different musical settings** to find your preferred sound
3. **Connect your hardware** and test with real sensors
4. **Experiment with threshold values** to optimize for your setup
5. **Explore the codebase** to understand the architecture

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review the main GLUVN documentation
3. Check the test suite in `gluvn_python/tests/`
4. Look at the implementation in `core/strategies/`

## Files in this Directory

- `ten_finger_demo_pyqt.py` - **PyQt5 demo version (recommended)**
- `ten_finger_gui_pyqt.py` - **PyQt5 hardware version (recommended)**
- `ten_finger_demo.py` - tkinter demo version
- `ten_finger_gui.py` - tkinter hardware version
- `README.md` - This documentation file

All applications use the new strategy-based architecture for trigger processing and note mapping, providing a clean foundation for future enhancements and customization. The PyQt5 versions are recommended for consistency with the existing GLUVN sensor plotting tools. 