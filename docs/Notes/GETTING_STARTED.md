# Getting Started with GLUVN Ten Finger System

## Quick Start Guide

### 1. Test the System (No Hardware Required)

Start with the demo version to understand how the system works:

```bash
cd gluvn_python
python examples/ten_finger_demo.py
```

**What you'll see:**
- A GUI with 10 finger lights (5 per hand)
- Configuration controls (threshold, hysteresis, scale, root note)
- Start/Stop/Auto Demo buttons

**How to use:**
1. Click "Start Demo" to begin
2. Click on individual finger lights to trigger them manually
3. Try "Auto Demo" to see automated sequences
4. Experiment with different musical scales and root notes

### 2. Test with Your Glove Hardware

Once you understand the interface, connect your GLUVN hardware:

```bash
cd gluvn_python
python examples/ten_finger_gui.py
```

**Setup:**
1. Connect your GLUVN sensor glove via USB/serial
2. Make sure your flex sensors are properly calibrated
3. Click "Start Sensors" to begin reading from hardware
4. Move your fingers to see the lights respond

## Understanding the System

### Visual Feedback
- **🔴 Dark Red Light**: Finger not triggered (sensor below threshold)
- **🟢 Green Light**: Finger triggered (sensor above threshold + hysteresis)
- **Note Names**: Each finger displays its corresponding MIDI note

### Key Parameters

#### Threshold (50-500)
- **What it does**: Sensor value needed to trigger a finger
- **Default**: 200 (good starting point for most flex sensors)
- **Adjust if**:
  - Lights don't trigger → Lower threshold (150-180)
  - Lights trigger too easily → Higher threshold (220-300)

#### Hysteresis (1-50)
- **What it does**: Prevents flickering from sensor noise
- **Default**: 10 (good balance of stability and responsiveness)
- **Adjust if**:
  - Lights flicker → Increase hysteresis (15-25)
  - Response feels sluggish → Decrease hysteresis (5-8)

#### Musical Settings
- **Scale**: Major (bright), Minor (darker), Pentatonic (always sounds good)
- **Root Note**: C, D, E, F, G, A, B (with sharps/flats available)

### Default Note Layout

**C Major Scale:**
```
Left Hand:          Right Hand:
Thumb:  G2 (55)     Thumb:  C4 (60)  ← Middle C
Index:  A2 (57)     Index:  D4 (62)
Middle: B2 (59)     Middle: E4 (64)
Ring:   C3 (52)     Ring:   F4 (65)
Pinky:  D3 (53)     Pinky:  G4 (67)
```

## Calibration Guide

### Step 1: Find Your Sensor Range
1. Start the demo or hardware version
2. Note the current threshold setting
3. Move your fingers and observe when lights trigger
4. Adjust threshold until triggering feels natural

### Step 2: Optimize Hysteresis
1. Set threshold to your preferred value
2. Start with hysteresis = 10
3. If lights flicker when finger is still → Increase hysteresis
4. If response feels delayed → Decrease hysteresis

### Step 3: Test Musical Settings
1. Try different scales to hear the difference
2. Use C Major for a familiar sound
3. Use Pentatonic for improvisation (always sounds musical)
4. Change root note to match your preferred key

## Troubleshooting

### Demo Version Issues
**Problem**: Demo won't start
- Check Python version (3.7+)
- Install missing packages: `pip install numpy tkinter`

**Problem**: No MIDI sound
- Install MIDI software (GarageBand, LoopMIDI, etc.)
- Connect virtual/hardware MIDI device
- Visual feedback will still work without MIDI

### Hardware Version Issues
**Problem**: "Sensor reader not available"
```bash
# Check if device is connected
ls /dev/tty*    # Look for your Arduino/USB device

# Check Python can access the device
python -c "from port_read import Reader; print('✅ Reader available')"
```

**Problem**: Lights don't respond to finger movement
1. Check sensor connections (wiggle wires)
2. Verify sensor values in raw data
3. Adjust threshold range (try 100-300)
4. Check sensor calibration

**Problem**: Lights trigger randomly or flicker
1. Increase hysteresis (15-30)
2. Check for electrical interference
3. Verify sensor mounting (not too loose/tight)
4. Check for damaged sensors

## Advanced Configuration

### Custom Thresholds per Hand
```python
# In the code, you can set different thresholds:
self.trigger_config = {
    'thresholds': {'flex': 200, 'press': 15},
    'hysteresis': {'flex': 10, 'press': 5},
    'trigger_sensors': {'l': 'flex', 'r': 'press'}  # Different sensors per hand
}
```

### Custom Musical Scales
```python
# In mapper.py, add new scales:
scales = {
    'major': [2, 2, 1, 2, 2, 2, 1],
    'minor': [2, 1, 2, 2, 1, 2, 2],
    'blues': [3, 2, 1, 1, 3, 2],        # Add blues scale
    'harmonic_minor': [2, 1, 2, 2, 1, 3, 1]  # Add harmonic minor
}
```

### Performance Optimization
- **Reduce update rate**: Change GUI update from 50ms to 100ms if needed
- **Adjust queue size**: Increase maxsize if you experience dropped data
- **Monitor CPU usage**: The demo should use <5% CPU on modern systems

## System Architecture

### Data Flow
```
Sensor → Trigger Strategy → Mapping Strategy → MIDI Output
                ↓
           Visual Display
```

### Key Components
1. **HysteresisTrigger**: Converts sensor data to discrete events
2. **BasicMapper**: Maps finger triggers to musical notes
3. **FingerLight**: Visual representation of trigger state
4. **GUI Application**: Coordinates everything with user controls

## Next Steps

### 1. Basic Usage
- Master the demo interface
- Understand threshold and hysteresis effects
- Try different musical settings

### 2. Hardware Integration
- Connect and calibrate your sensors
- Optimize settings for your specific hardware
- Test with actual music software

### 3. Customization
- Explore the codebase structure
- Try modifying musical scales
- Experiment with different trigger behaviors

### 4. Advanced Features
- Look at the strategy pattern implementation
- Consider adding new trigger strategies
- Explore integration with DAW software

## Support Resources

- **Code Documentation**: See `core/strategies/` for implementation details
- **Test Suite**: Run `pytest tests/` to verify functionality
- **Examples**: Check `examples/` directory for more applications
- **Architecture Overview**: See main README for system design

## File Reference

- `examples/ten_finger_demo.py` - Demo version (no hardware)
- `examples/ten_finger_gui.py` - Hardware version
- `test_basic_imports.py` - Verify system functionality
- `tests/test_trigger_strategies.py` - Unit tests
- `core/strategies/` - Core implementation
- `mapper.py` - Musical note mapping

**Ready to start?** Run `python test_basic_imports.py` to verify everything is working, then launch the demo! 