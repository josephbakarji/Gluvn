# 🎛️ GLUVN User-Configurable Parameters Guide

## Overview

All key parameters in the GLUVN IMU modulation system are now easily configurable through the GUI interface. No more hardcoded constants! Users can adjust everything from sensitivity to smoothing to sensor assignments.

---

## 🎵 **Trigger Parameters**

### Basic Trigger Controls
| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| **Threshold** | 50-500 | 120 | Minimum sensor value to trigger notes |
| **Hysteresis** | 1-50 | 10 | Prevents trigger bouncing near threshold |
| **Sensor Type** | flex/press | flex | Which sensor type to use for triggering |

---

## 🎛️ **Volume Control (Accelerometer)**

### Volume Parameters
| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| **Base Volume** | 0-100 | 20 | Minimum volume level (always present) |
| **Volume Smoothing** | 1-50 | 10 | Averaging window size for stable volume |
| **Volume Sensitivity** | 5000-30000 | 15000 | How sensitive volume is to hand movement |

### How It Works
- Uses accelerometer magnitude (√(x² + y² + z²)) to control volume
- Higher hand movement = higher volume
- Smoothing prevents jittery volume changes
- Base volume ensures notes are always audible

---

## 🎶 **Pitch Bend Control (IMU)**

### Pitch Parameters
| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| **Pitch Bend Range** | 1000-8192 | 8192 | Maximum pitch bend amount (MIDI units) |
| **Pitch Smoothing** | 1-20 | 5 | History window size for smooth pitch changes |
| **Pitch Mode** | modulo/linear | modulo | How IMU data maps to pitch bend |
| **Pitch Sensor** | imu1-imu5 | imu2 | Which IMU sensor controls pitch |

### Pitch Modes
- **Modulo**: Cyclical pitch bending (good for vibrato effects)
- **Linear**: Direct mapping from sensor to pitch bend

---

## 🪟 **Moving Window Control**

### Window Parameters
| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| **Min Window Size** | 1-20 | 1 | Smallest averaging window |
| **Max Window Size** | 10-100 | 50 | Largest averaging window |
| **Window Sensor** | imu1-imu5 | imu1 | Which sensor controls window size |

### How It Works
- IMU data dynamically controls averaging window sizes
- Smaller windows = more responsive, less smooth
- Larger windows = more stable, less responsive

---

## 🎭 **Choir Control (Multi-Voice)**

### Voice Controllers
| Voice | Default Sensor | Range | Description |
|-------|----------------|-------|-------------|
| **Voice 1 Volume** | imu1 | 0-127 | First voice volume control |
| **Voice 2 Volume** | imu2 | 0-127 | Second voice volume control |
| **Voice 3 Volume** | imu3 | 0-127 | Third voice volume control |
| **Master Pitch** | imu4 | -8192 to +8192 | Overall pitch bend for all voices |

---

## 💾 **Configuration Management**

### Save/Load System
- **Save Config**: Export all current settings to JSON file
- **Load Config**: Import settings from JSON file
- **Basic Preset**: Simple volume + pitch bend setup
- **Advanced Preset**: Full modulation with all features

### Configuration Files
```json
{
  "threshold": 120,
  "hysteresis": 10,
  "sensor_type": "flex",
  "volume_enabled": true,
  "base_volume": 20,
  "volume_smoothing": 10,
  "volume_sensitivity": 15000,
  "pitch_bend_enabled": true,
  "pitch_range": 8192,
  "pitch_smoothing": 5,
  "pitch_mode": "modulo",
  "pitch_sensor": "imu2"
}
```

---

## 🎚️ **Preset Configurations**

### Basic Preset
- **Use Case**: Simple performance with volume and pitch control
- **Features**: Moderate sensitivity, stable smoothing
- **Best For**: Beginners, straightforward musical expression

| Parameter | Value |
|-----------|-------|
| Volume Enabled | ✅ |
| Base Volume | 20 |
| Volume Sensitivity | 15000 |
| Pitch Bend Enabled | ✅ |
| Pitch Range | 4096 |
| Pitch Mode | modulo |

### Advanced Preset
- **Use Case**: Complex performances with all modulation features
- **Features**: High sensitivity, minimal smoothing, all controls
- **Best For**: Experienced users, complex musical arrangements

| Parameter | Value |
|-----------|-------|
| Volume Enabled | ✅ |
| Base Volume | 10 |
| Volume Sensitivity | 20000 |
| Pitch Bend Enabled | ✅ |
| Pitch Range | 8192 |
| Pitch Mode | linear |
| Window Control | ✅ |
| Choir Control | ✅ |

---

## 🎯 **Parameter Tuning Tips**

### For Stable Performance
- **Higher smoothing values** (10-20) for stable, predictable control
- **Moderate sensitivity** (15000) for consistent response
- **Modulo pitch mode** for cyclical effects

### For Responsive Performance
- **Lower smoothing values** (3-7) for immediate response
- **Higher sensitivity** (20000-30000) for subtle movements
- **Linear pitch mode** for direct control

### For Complex Arrangements
- **Enable window control** for dynamic response characteristics
- **Enable choir control** for multi-voice arrangements
- **Lower base volume** (5-15) to allow more dynamic range

---

## 🔧 **Technical Details**

### Sensor Mappings
- **imu1-imu5**: Different IMU orientation axes
- **imu3, imu4, imu5**: Typically accelerometer X, Y, Z axes
- **Each sensor** can be assigned to different modulation functions

### MIDI Output Ranges
- **Volume/Aftertouch**: 0-127 (standard MIDI)
- **Pitch Bend**: -8192 to +8192 (standard MIDI pitch bend)
- **Real-time updates**: 50ms refresh rate (20 FPS)

### Smoothing Implementation
- **Moving average**: Using circular buffer for efficiency
- **Window sizes**: Configurable from 1-50 samples
- **Memory efficient**: Bounded memory usage regardless of window size

---

## 🚀 **Quick Start Guide**

### 1. Basic Setup
1. Start with **Basic Preset**
2. Adjust **Base Volume** to comfortable level
3. Test **Volume Sensitivity** with hand movements
4. Fine-tune **Pitch Range** for desired bend amount

### 2. Advanced Setup
1. Load **Advanced Preset**
2. Assign different **sensors** to different functions
3. Adjust **smoothing** values for your playing style
4. Enable **Window/Choir Control** for complex effects

### 3. Save Your Settings
1. Configure all parameters to your liking
2. Click **Save Config** to export settings
3. Share configuration files with other users
4. Load different configs for different performance styles

---

**🎉 Result: Complete user control over all modulation parameters with no hardcoded constants!** 