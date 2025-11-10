# GLUVN Sensor-to-MIDI System

## Overview

The GLUVN system transforms sensor data from instrumented gloves into MIDI messages, enabling expressive musical performance through hand gestures. The system combines flex sensors, pressure sensors, and IMU (Inertial Measurement Unit) data to create a versatile musical interface.

## Current Architecture

### Core Components

1. **SensorProcess** (`senstonote_modern.py`)
   - Processes raw sensor data from each hand
   - Implements hysteresis-based triggering logic
   - Handles both discrete triggers and continuous modulation

2. **BaseApp** (`senstonote_modern.py`)
   - Base class for all sensor-to-MIDI applications
   - Manages sensor configuration and initialization
   - Provides basic note triggering functionality

3. **MovingWindow** (Multiple implementations)
   - Advanced application with window-based note mapping
   - Supports volume control, pitch bending, and dynamic note selection
   - **ISSUE**: Currently duplicated across multiple files with variations

4. **NoteMapper** (`mapper.py`)
   - Handles musical theory and note-to-MIDI conversion
   - Supports multiple scales and mapping strategies
   - Implements moving window system for extended note range

5. **Configuration System** (`configs/`)
   - JSON-based configuration files
   - Base and specialized configuration classes
   - Supports saving/loading of presets

### Current Applications

- **BaseApp**: Simple finger-to-note mapping
- **MovingWindow**: Dynamic note window selection
- **Harmonizer**: Chord-based harmonization
- **Jacob Choir**: Complex IMU-based directional control

## Requirements

### Functional Requirements

1. **Sensor Integration**
   - Support for flex sensors (finger bend detection)
   - Support for pressure sensors (finger press detection)
   - Support for IMU data (orientation, acceleration)
   - Configurable sensor combinations per hand

2. **Musical Mapping**
   - Multiple scale support (major, minor, pentatonic, etc.)
   - Configurable root notes and transposition
   - Dynamic note window selection
   - Chord and harmony generation
   - Custom mapping strategies

3. **MIDI Output**
   - Note on/off events with velocity
   - Continuous controllers (volume, pitch bend)
   - Aftertouch and channel pressure
   - Multiple MIDI channel support

4. **Real-time Performance**
   - Low-latency sensor processing
   - Stable triggering with hysteresis
   - Smooth continuous control
   - Minimal audio dropouts

5. **Configuration Management**
   - Preset saving and loading
   - GUI-based configuration (future)
   - Runtime parameter adjustment
   - User-friendly defaults

### Technical Requirements

1. **Modularity**
   - Pluggable sensor processing modules
   - Extensible mapping system
   - Configurable application modes
   - Clean separation of concerns

2. **Flexibility**
   - Support for new sensor types
   - Custom trigger logic
   - Advanced mapping algorithms
   - Multi-modal interactions

3. **Usability**
   - Simple configuration for common use cases
   - Advanced options for power users
   - Clear documentation and examples
   - Error handling and recovery

4. **Performance**
   - Efficient sensor data processing
   - Minimal memory allocation in real-time paths
   - Optimized MIDI output
   - Scalable to multiple hands/devices

## Current Issues

### Code Duplication
- **MovingWindow class** is duplicated across multiple files:
  - `senstonote_modern.py` (canonical version)
  - `app_harmonizer.py`
  - `app_10fig_accel.py`
  - `app_10fig_pitch.py`
  - `app_jacob_choir.py`
  - Legacy files

### Inconsistent Interfaces
- Different parameter names across implementations
- Varying initialization patterns
- Inconsistent error handling

### Limited Extensibility
- Hard-coded sensor processing logic
- Fixed mapping strategies
- Difficult to add new interaction modes

### Configuration Complexity
- Manual parameter tuning required
- No runtime configuration updates
- Limited preset management

## Planned Improvements

### 1. Unified Architecture

#### Core Classes Refactoring
```python
# Unified base classes
class SensorProcessor:
    """Handles sensor data processing with pluggable strategies"""
    
class TriggerStrategy:
    """Base class for trigger logic implementations"""
    
class MappingStrategy:
    """Base class for sensor-to-note mapping strategies"""
    
class ModulationStrategy:
    """Base class for continuous control strategies"""
```

#### Application Framework
```python
class GluvnApp:
    """Unified application framework"""
    def __init__(self, config: AppConfig):
        self.sensor_processor = SensorProcessor(config.sensor_config)
        self.trigger_strategy = config.trigger_strategy
        self.mapping_strategy = config.mapping_strategy
        self.modulation_strategy = config.modulation_strategy
```

### 2. Enhanced Configuration System

#### Hierarchical Configuration
- Base configurations for common patterns
- Specialized configurations for specific applications
- Runtime configuration updates
- Configuration validation

#### Preset Management
- Built-in presets for common use cases
- User-defined preset saving/loading
- Preset sharing and import/export
- Version control for configurations

### 3. Advanced Mapping System

#### Pluggable Mappers
```python
class BasicMapper(MappingStrategy):
    """Simple finger-to-note mapping"""
    
class WindowMapper(MappingStrategy):
    """Moving window note selection"""
    
class ChordMapper(MappingStrategy):
    """Chord-based harmonization"""
    
class DirectionalMapper(MappingStrategy):
    """IMU-based directional control"""
```

#### Smart Mappings
- Machine learning-based adaptation
- Context-aware note selection
- Predictive harmonization
- User behavior learning

### 4. Modular Sensor Processing

#### Sensor Abstraction
```python
class SensorChannel:
    """Abstract sensor data channel"""
    
class FlexSensor(SensorChannel):
    """Flex sensor implementation"""
    
class PressureSensor(SensorChannel):
    """Pressure sensor implementation"""
    
class IMUSensor(SensorChannel):
    """IMU sensor implementation"""
```

#### Processing Pipeline
- Configurable processing chains
- Real-time filtering and smoothing
- Calibration and normalization
- Multi-sensor fusion

### 5. GUI Configuration Interface

#### Configuration Editor
- Visual parameter adjustment
- Real-time preview
- Preset management
- Help and documentation integration

#### Performance Monitor
- Real-time sensor visualization
- MIDI output monitoring
- Performance metrics
- Debugging tools

## Implementation Plan

### Phase 1: Core Refactoring
1. Create unified base classes
2. Eliminate MovingWindow duplication
3. Implement strategy pattern for mappings
4. Update configuration system

### Phase 2: Enhanced Functionality
1. Implement advanced mapping strategies
2. Add modular sensor processing
3. Create comprehensive preset library
4. Improve error handling and validation

### Phase 3: GUI Development
1. Design configuration interface
2. Implement real-time monitoring
3. Add preset management UI
4. Create user documentation

### Phase 4: Advanced Features
1. Machine learning integration
2. Multi-device support
3. Network synchronization
4. Performance optimization

## File Structure

```
gluvn_python/
├── core/
│   ├── sensor_processor.py      # Unified sensor processing
│   ├── strategies/
│   │   ├── trigger_strategies.py
│   │   ├── mapping_strategies.py
│   │   └── modulation_strategies.py
│   └── app_framework.py         # Unified application framework
├── configs/
│   ├── base_config.py          # Base configuration classes
│   ├── presets/                # Built-in preset configurations
│   └── schemas/                # Configuration validation schemas
├── mappers/
│   ├── note_mapper.py          # Enhanced note mapping
│   ├── chord_mapper.py         # Chord and harmony mapping
│   └── smart_mapper.py         # ML-based adaptive mapping
├── gui/
│   ├── config_editor.py        # Configuration interface
│   ├── monitor.py              # Real-time monitoring
│   └── preset_manager.py       # Preset management
├── legacy/                     # Legacy code (deprecated)
└── examples/                   # Example applications and tutorials
```

## Usage Examples

### Basic Configuration
```python
from gluvn_python import GluvnApp, BasicConfig

config = BasicConfig(
    root_note='C',
    scale='major',
    trigger_sensors={'l': 'flex', 'r': 'flex'}
)

app = GluvnApp(config)
app.start()
```

### Advanced Configuration
```python
from gluvn_python import GluvnApp, AdvancedConfig
from gluvn_python.strategies import WindowMapper, AccelVolumeControl

config = AdvancedConfig(
    mapping_strategy=WindowMapper(num_windows=10),
    modulation_strategy=AccelVolumeControl(smoothing=0.8),
    trigger_thresholds={'flex': 180, 'press': 20}
)

app = GluvnApp(config)
app.start()
```

### Custom Mapping
```python
class CustomMapper(MappingStrategy):
    def map_sensors_to_notes(self, sensor_data):
        # Custom mapping logic
        return note_events

config = AdvancedConfig(mapping_strategy=CustomMapper())
app = GluvnApp(config)
```

## Contributing

1. Follow the modular architecture principles
2. Add comprehensive tests for new features
3. Update documentation for API changes
4. Maintain backward compatibility where possible
5. Use type hints and docstrings consistently

## Testing

- Unit tests for core components
- Integration tests for complete workflows
- Performance benchmarks
- Real-time latency measurements
- User acceptance testing

## Future Enhancements

- Multi-user collaboration
- Cloud-based preset sharing
- Mobile device integration
- VR/AR visualization
- AI-powered composition assistance 