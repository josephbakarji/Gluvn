# Immediate Next Steps for GLUVN Refactoring

## Priority 1: Foundation Setup (This Week)

### 1. Create Core Directory Structure
```bash
cd gluvn_python
mkdir -p core/strategies
mkdir -p configs/presets
mkdir -p mappers
mkdir -p examples
mkdir -p tests
```

### 2. Implement Base Strategy Classes

#### Create `core/__init__.py`
```python
"""
GLUVN Core Module
Unified sensor-to-MIDI processing framework
"""

from .app_framework import GluvnApp
from .strategies import *

__version__ = "2.0.0"
```

#### Create `core/strategies/__init__.py`
```python
"""
Strategy pattern implementations for GLUVN system
"""

from .base_strategies import TriggerStrategy, MappingStrategy, ModulationStrategy
from .trigger_strategies import HysteresisTrigger, MultiSensorTrigger, IMUDirectionalTrigger
from .mapping_strategies import BasicMapper, WindowMapper, ChordMapper, DirectionalMapper
from .modulation_strategies import VolumeModulation, AccelVolumeModulation, PitchBendModulation, IMUPitchBend

__all__ = [
    'TriggerStrategy', 'MappingStrategy', 'ModulationStrategy',
    'HysteresisTrigger', 'MultiSensorTrigger', 'IMUDirectionalTrigger',
    'BasicMapper', 'WindowMapper', 'ChordMapper', 'DirectionalMapper',
    'VolumeModulation', 'AccelVolumeModulation', 'PitchBendModulation', 'IMUPitchBend'
]
```

### 3. Start with Minimal Working Implementation

#### Step 3.1: Create `core/strategies/base_strategies.py`
This is the foundation - implement the abstract base classes first.

#### Step 3.2: Create `core/strategies/trigger_strategies.py`
Start with `HysteresisTrigger` - port the existing trigger logic from `SensorProcess`.

#### Step 3.3: Create `core/strategies/mapping_strategies.py`
Start with `BasicMapper` - port the basic note mapping functionality.

#### Step 3.4: Create `core/app_framework.py`
Implement the unified application framework.

### 4. Test with Existing Application

#### Step 4.1: Create Compatibility Layer
Update `senstonote_modern.py` to use the new framework while maintaining backward compatibility.

#### Step 4.2: Test with `app_10fig_inst.py`
This is the simplest application - use it to validate the new framework works.

## Priority 2: Configuration Enhancement (Next Week)

### 1. Extend Configuration System
- Add strategy type parameters to `BaseConfig`
- Create `AdvancedConfig` class
- Implement configuration validation

### 2. Create Preset Configurations
- Port existing JSON configs to new format
- Create additional presets for common use cases
- Add preset loading/saving functionality

### 3. Update App Runner
- Modify `app_runner.py` to use new framework
- Add support for strategy selection
- Implement preset selection

## Priority 3: Eliminate Duplication (Week 3)

### 1. Replace MovingWindow Implementations
- Update each app file to use new framework
- Remove duplicated MovingWindow classes
- Maintain functionality parity

### 2. Create Migration Scripts
- Script to automatically update application files
- Validation script to ensure functionality is preserved
- Performance comparison script

## Detailed Implementation Order

### Day 1-2: Base Classes
1. `core/strategies/base_strategies.py` - Abstract base classes
2. `core/strategies/trigger_strategies.py` - HysteresisTrigger implementation
3. `core/strategies/mapping_strategies.py` - BasicMapper implementation

### Day 3-4: Application Framework
1. `core/app_framework.py` - GluvnApp implementation
2. `core/strategies/modulation_strategies.py` - Basic modulation strategies
3. Test with simple application

### Day 5-7: Integration and Testing
1. Update `senstonote_modern.py` for backward compatibility
2. Test with `app_10fig_inst.py`
3. Create unit tests for core components
4. Performance validation

## Code Templates to Start With

### Base Strategy Template
```python
# core/strategies/base_strategies.py
from abc import ABC, abstractmethod
from typing import Dict, Any, List, Optional, Union
import numpy as np

class TriggerStrategy(ABC):
    """Base class for trigger logic implementations"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.trigger_states = {}
        self.initialize()
    
    @abstractmethod
    def initialize(self):
        """Initialize strategy-specific state"""
        pass
    
    @abstractmethod
    def process_triggers(self, sensor_data: Dict[str, Any], hand: str) -> np.ndarray:
        """Process sensor data and return trigger events"""
        pass
    
    def get_trigger_state(self, hand: str) -> np.ndarray:
        """Get current trigger state for hand"""
        return self.trigger_states.get(hand, np.zeros(5, dtype=bool))
```

### Hysteresis Trigger Template
```python
# core/strategies/trigger_strategies.py
from .base_strategies import TriggerStrategy
import numpy as np

class HysteresisTrigger(TriggerStrategy):
    """Hysteresis-based triggering with configurable thresholds"""
    
    def initialize(self):
        self.thresholds = self.config.get('thresholds', {'flex': 200, 'press': 15})
        self.hysteresis = self.config.get('hysteresis', {'flex': 5, 'press': 5})
        self.trigger_sensor = self.config.get('trigger_sensor', 'flex')
        
        # Initialize state for both hands
        for hand in ['l', 'r']:
            self.trigger_states[hand] = np.zeros(5, dtype=bool)
            
        self.trig_on = {'l': np.zeros(5, dtype=bool), 'r': np.zeros(5, dtype=bool)}
        self.trig_off = {'l': np.zeros(5, dtype=bool), 'r': np.zeros(5, dtype=bool)}
    
    def process_triggers(self, sensor_data: Dict[str, Any], hand: str) -> np.ndarray:
        """Implement hysteresis-based triggering logic"""
        if self.trigger_sensor not in sensor_data:
            return np.zeros(5, dtype=int)
            
        # Port existing trigger logic from SensorProcess.trigger_logic()
        sensor_values = sensor_data[self.trigger_sensor]
        # ... implementation
        
        return trigger_events
```

## Testing Strategy

### Unit Tests
```python
# tests/test_trigger_strategies.py
import unittest
import numpy as np
from core.strategies.trigger_strategies import HysteresisTrigger

class TestHysteresisTrigger(unittest.TestCase):
    def setUp(self):
        config = {
            'thresholds': {'flex': 200},
            'hysteresis': {'flex': 5},
            'trigger_sensor': 'flex'
        }
        self.trigger = HysteresisTrigger(config)
    
    def test_basic_triggering(self):
        # Test basic trigger functionality
        sensor_data = {'flex': [250, 150, 250, 150, 250]}  # Above/below threshold
        events = self.trigger.process_triggers(sensor_data, 'r')
        # Assert expected trigger events
```

### Integration Tests
```python
# tests/test_app_framework.py
import unittest
from core.app_framework import GluvnApp
from configs.base_config import BaseConfig

class TestGluvnApp(unittest.TestCase):
    def test_basic_app_creation(self):
        config = BaseConfig()
        app = GluvnApp(config)
        self.assertIsNotNone(app.trigger_strategy)
        self.assertIsNotNone(app.mapping_strategy)
```

## Success Criteria for Phase 1

1. ✅ Core directory structure created
2. ✅ Base strategy classes implemented and tested
3. ✅ HysteresisTrigger working with existing sensor data
4. ✅ BasicMapper producing correct note mappings
5. ✅ GluvnApp framework running simple applications
6. ✅ Backward compatibility maintained for existing apps
7. ✅ Performance equivalent to current implementation
8. ✅ Unit tests passing for core components

## Risk Mitigation

### Backup Strategy
- Keep all existing code intact during refactoring
- Create new modules alongside existing ones
- Only replace when new implementation is fully validated

### Performance Monitoring
- Benchmark current system performance
- Monitor latency and throughput during refactoring
- Rollback if performance degrades significantly

### Incremental Deployment
- Start with simplest applications
- Gradually migrate more complex applications
- Maintain parallel implementations during transition

## Communication Plan

### Documentation Updates
- Update README with new architecture overview
- Create migration guide for existing users
- Document new configuration options

### Code Reviews
- Review each major component before integration
- Validate against existing functionality
- Ensure code quality and maintainability standards

This plan provides a clear, actionable roadmap for starting the refactoring process while minimizing risk and maintaining system functionality. 