# GLUVN System Refactoring Implementation Plan

## Phase 1: Core Architecture Refactoring

### Step 1: Create Strategy Pattern Base Classes

#### 1.1 Create `core/strategies/` directory structure
```
gluvn_python/core/
├── __init__.py
├── strategies/
│   ├── __init__.py
│   ├── base_strategies.py
│   ├── trigger_strategies.py
│   ├── mapping_strategies.py
│   └── modulation_strategies.py
```

#### 1.2 Base Strategy Classes (`core/strategies/base_strategies.py`)
```python
from abc import ABC, abstractmethod
from typing import Dict, Any, List, Optional
import numpy as np

class TriggerStrategy(ABC):
    """Base class for trigger logic implementations"""
    
    @abstractmethod
    def process_triggers(self, sensor_data: Dict[str, Any], hand: str) -> np.ndarray:
        """Process sensor data and return trigger events"""
        pass
    
    @abstractmethod
    def get_trigger_state(self, hand: str) -> np.ndarray:
        """Get current trigger state for hand"""
        pass

class MappingStrategy(ABC):
    """Base class for sensor-to-note mapping strategies"""
    
    @abstractmethod
    def map_to_notes(self, trigger_events: np.ndarray, hand: str, **kwargs) -> List[int]:
        """Map trigger events to MIDI note numbers"""
        pass
    
    @abstractmethod
    def update_mapping(self, **kwargs) -> None:
        """Update mapping parameters"""
        pass

class ModulationStrategy(ABC):
    """Base class for continuous control strategies"""
    
    @abstractmethod
    def process_modulation(self, sensor_data: Dict[str, Any], hand: str) -> Dict[str, float]:
        """Process sensor data for continuous controls"""
        pass
```

#### 1.3 Concrete Strategy Implementations

**Trigger Strategies** (`core/strategies/trigger_strategies.py`)
```python
class HysteresisTrigger(TriggerStrategy):
    """Hysteresis-based triggering with configurable thresholds"""
    
    def __init__(self, thresholds: Dict[str, float], hysteresis: Dict[str, float]):
        self.thresholds = thresholds
        self.hysteresis = hysteresis
        self.trigger_states = {'l': np.zeros(5, dtype=bool), 'r': np.zeros(5, dtype=bool)}
        self.trig_on = {'l': np.zeros(5, dtype=bool), 'r': np.zeros(5, dtype=bool)}
        self.trig_off = {'l': np.zeros(5, dtype=bool), 'r': np.zeros(5, dtype=bool)}

class MultiSensorTrigger(TriggerStrategy):
    """Trigger based on multiple sensor fusion"""
    
class IMUDirectionalTrigger(TriggerStrategy):
    """IMU-based directional triggering (Jacob Choir style)"""
```

**Mapping Strategies** (`core/strategies/mapping_strategies.py`)
```python
class BasicMapper(MappingStrategy):
    """Simple finger-to-note mapping"""
    
    def __init__(self, note_mapper):
        self.note_mapper = note_mapper
        self.note_maps = note_mapper.basic_map_2hands()

class WindowMapper(MappingStrategy):
    """Moving window note selection"""
    
    def __init__(self, note_mapper, num_lh_fingers=5, num_rh_fingers=5):
        self.note_mapper = note_mapper
        self.window_trigger, self.note_windows = note_mapper.moving_window(
            num_lhf=num_lh_fingers, num_rhf=num_rh_fingers
        )
        self.current_window = None

class ChordMapper(MappingStrategy):
    """Chord-based harmonization mapping"""

class DirectionalMapper(MappingStrategy):
    """IMU-based directional note control"""
```

**Modulation Strategies** (`core/strategies/modulation_strategies.py`)
```python
class VolumeModulation(ModulationStrategy):
    """Volume control strategies"""
    
class AccelVolumeModulation(VolumeModulation):
    """Accelerometer-based volume control"""
    
class PitchBendModulation(ModulationStrategy):
    """Pitch bend control strategies"""
    
class IMUPitchBend(PitchBendModulation):
    """IMU-based pitch bending"""
```

### Step 2: Unified Application Framework

#### 2.1 Core Application Class (`core/app_framework.py`)
```python
from typing import Optional, Dict, Any
from .strategies.base_strategies import TriggerStrategy, MappingStrategy, ModulationStrategy
from configs.base_config import BaseConfig
from mapper import NoteMapper
from port_read import Reader
from midi_writer import MidiWriter
import queue
from threading import Thread

class GluvnApp(Thread):
    """Unified application framework for all GLUVN applications"""
    
    def __init__(self, config: BaseConfig):
        super().__init__()
        self.daemon = True
        self.config = config
        
        # Initialize core components
        self.reader = Reader(sensor_config=config.sensor_config)
        self.note_mapper = NoteMapper(root_note=config.root_note, scale=config.scale)
        self.midi_writer = MidiWriter()
        self.collect_q = queue.Queue(maxsize=20)
        
        # Initialize strategies
        self.trigger_strategy = self._create_trigger_strategy()
        self.mapping_strategy = self._create_mapping_strategy()
        self.modulation_strategy = self._create_modulation_strategy()
        
        # Initialize sensor processors
        self.sensor_processors = {}
        self._initialize_sensor_processors()
    
    def _create_trigger_strategy(self) -> TriggerStrategy:
        """Factory method to create trigger strategy from config"""
        # Implementation based on config.trigger_type
        pass
    
    def _create_mapping_strategy(self) -> MappingStrategy:
        """Factory method to create mapping strategy from config"""
        # Implementation based on config.mapping_type
        pass
    
    def _create_modulation_strategy(self) -> Optional[ModulationStrategy]:
        """Factory method to create modulation strategy from config"""
        # Implementation based on config.modulation_type
        pass
    
    def run(self):
        """Main application loop"""
        self.reader.start_readers()
        
        while True:
            sensor_data = self.collect_q.get(block=True)
            hand = sensor_data.get('hand')
            
            # Process triggers
            trigger_events = self.trigger_strategy.process_triggers(sensor_data, hand)
            
            # Map to notes
            if np.any(trigger_events):
                notes = self.mapping_strategy.map_to_notes(trigger_events, hand)
                for note in notes:
                    if note is not None:
                        self.midi_writer.trig_note(note)
            
            # Process modulation
            if self.modulation_strategy:
                modulation = self.modulation_strategy.process_modulation(sensor_data, hand)
                self._apply_modulation(modulation)
    
    def _apply_modulation(self, modulation: Dict[str, float]):
        """Apply modulation controls to MIDI output"""
        if 'volume' in modulation:
            self.midi_writer.aftertouch(int(modulation['volume']))
        if 'pitch_bend' in modulation:
            self.midi_writer.pitch_bend(int(modulation['pitch_bend']))
```

### Step 3: Enhanced Configuration System

#### 3.1 Extended Base Configuration (`configs/base_config.py`)
```python
class BaseConfig:
    def __init__(self):
        # Existing configuration...
        
        # Strategy configuration
        self.trigger_type = 'hysteresis'  # 'hysteresis', 'multi_sensor', 'imu_directional'
        self.mapping_type = 'basic'       # 'basic', 'window', 'chord', 'directional'
        self.modulation_type = None       # None, 'volume', 'pitch_bend', 'combined'
        
        # Strategy-specific parameters
        self.strategy_params = {}

class AdvancedConfig(BaseConfig):
    def __init__(self):
        super().__init__()
        self.trigger_type = 'hysteresis'
        self.mapping_type = 'window'
        self.modulation_type = 'combined'
        
        self.strategy_params = {
            'window_mapper': {
                'num_lh_fingers': 5,
                'num_rh_fingers': 5,
                'instrument': 'double_flex'
            },
            'volume_modulation': {
                'controller': 'accel_mag',
                'base_volume': 20,
                'averaging_window': 10
            },
            'pitch_bend_modulation': {
                'controller': 'imu2',
                'bend_range': 8192
            }
        }
```

#### 3.2 Preset Configurations (`configs/presets/`)
Create preset files for common configurations:
- `ten_finger_basic.json`
- `moving_window_flex.json`
- `harmonizer_advanced.json`
- `jacob_choir_directional.json`

### Step 4: Eliminate MovingWindow Duplication

#### 4.1 Update `senstonote_modern.py`
```python
# Remove MovingWindow class, replace with:
from core.app_framework import GluvnApp
from core.strategies.mapping_strategies import WindowMapper
from core.strategies.modulation_strategies import AccelVolumeModulation, IMUPitchBend

# Keep BaseApp for backward compatibility
class BaseApp(GluvnApp):
    """Backward compatibility wrapper"""
    def __init__(self, **kwargs):
        # Convert old-style parameters to new config
        config = self._convert_legacy_params(**kwargs)
        super().__init__(config)

# Factory function for MovingWindow
def MovingWindow(**kwargs):
    """Factory function to create MovingWindow-style applications"""
    config = MovingWindowConfig()
    # Update config from kwargs
    return GluvnApp(config)
```

#### 4.2 Update Application Files
Replace duplicated MovingWindow classes with imports:
```python
# In app_harmonizer.py, app_10fig_accel.py, etc.
from senstonote_modern import MovingWindow

# Or use the new framework directly:
from core.app_framework import GluvnApp
from configs.moving_window_config import MovingWindowConfig

config = MovingWindowConfig()
# Customize config as needed
app = GluvnApp(config)
```

### Step 5: Enhanced Mapper System

#### 5.1 Extended NoteMapper (`mappers/note_mapper.py`)
```python
class NoteMapper:
    # Existing implementation...
    
    def get_mapping_strategy(self, strategy_type: str, **params):
        """Factory method for mapping strategies"""
        if strategy_type == 'basic':
            return BasicMapper(self)
        elif strategy_type == 'window':
            return WindowMapper(self, **params)
        elif strategy_type == 'chord':
            return ChordMapper(self, **params)
        elif strategy_type == 'directional':
            return DirectionalMapper(self, **params)
        else:
            raise ValueError(f"Unknown mapping strategy: {strategy_type}")
```

#### 5.2 Chord Mapper (`mappers/chord_mapper.py`)
```python
class ChordMapper:
    """Advanced chord and harmony mapping"""
    
    def __init__(self, note_mapper, chord_types=['major', 'minor']):
        self.note_mapper = note_mapper
        self.chord_types = chord_types
        self.current_chord = None
    
    def map_to_chord(self, root_note: int, chord_type: str) -> List[int]:
        """Map a root note to a chord"""
        # Implementation for chord generation
        pass
```

## Phase 2: Implementation Steps

### Step 1: Create Directory Structure
```bash
mkdir -p gluvn_python/core/strategies
mkdir -p gluvn_python/configs/presets
mkdir -p gluvn_python/mappers
mkdir -p gluvn_python/examples
```

### Step 2: Implement Base Classes
1. Create `core/strategies/base_strategies.py`
2. Implement concrete strategy classes
3. Create unified application framework

### Step 3: Update Configuration System
1. Extend base configuration classes
2. Create preset configurations
3. Add configuration validation

### Step 4: Refactor Existing Applications
1. Update `senstonote_modern.py`
2. Replace duplicated MovingWindow classes
3. Update application files to use new framework

### Step 5: Testing and Validation
1. Create unit tests for strategy classes
2. Test backward compatibility
3. Validate real-time performance
4. Test with actual hardware

## Migration Guide

### For Existing Applications
```python
# Old way:
from senstonote_modern import MovingWindow
app = MovingWindow(
    root_note='C',
    scale='major',
    volume_controller='accel_mag',
    pitch_bender='imu2'
)

# New way:
from core.app_framework import GluvnApp
from configs.moving_window_config import MovingWindowConfig

config = MovingWindowConfig()
config.root_note = 'C'
config.scale = 'major'
config.volume_controller = 'accel_mag'
config.pitch_bender = 'imu2'

app = GluvnApp(config)
```

### For Custom Applications
```python
# Create custom strategy
class MyCustomMapper(MappingStrategy):
    def map_to_notes(self, trigger_events, hand, **kwargs):
        # Custom mapping logic
        return notes

# Use in application
config = AdvancedConfig()
config.mapping_strategy = MyCustomMapper()
app = GluvnApp(config)
```

## Benefits of Refactoring

1. **Eliminates Code Duplication**: Single source of truth for core functionality
2. **Improves Maintainability**: Changes in one place affect all applications
3. **Enhances Extensibility**: Easy to add new strategies and behaviors
4. **Better Testing**: Isolated components can be tested independently
5. **Cleaner Architecture**: Clear separation of concerns
6. **Configuration Flexibility**: Easy to create and share presets
7. **Future-Proof**: Foundation for GUI and advanced features

## Timeline

- **Week 1-2**: Implement base strategy classes and framework
- **Week 3**: Update configuration system and create presets
- **Week 4**: Refactor existing applications and eliminate duplication
- **Week 5**: Testing, validation, and documentation
- **Week 6**: Performance optimization and bug fixes

## Success Metrics

1. All existing applications work with new framework
2. No performance regression in real-time operation
3. Reduced codebase size (eliminate ~80% of duplication)
4. Improved test coverage (>90% for core components)
5. Simplified configuration for common use cases
6. Easy extensibility for new features 