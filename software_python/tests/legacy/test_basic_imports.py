#!/usr/bin/env python3
"""
Basic Import and Functionality Test

This script tests that all the core components can be imported and
basic functionality works as expected.
"""

import sys
import os
sys.path.insert(0, '.')

def test_imports():
    """Test that all required modules can be imported"""
    print("Testing imports...")
    
    try:
        from core.strategies.trigger_strategies import HysteresisTrigger
        print("✅ HysteresisTrigger imported successfully")
        
        from core.strategies.mapping_strategies import BasicMapper
        print("✅ BasicMapper imported successfully")
        
        from mapper import NoteMapper
        print("✅ NoteMapper imported successfully")
        
        return True, (HysteresisTrigger, BasicMapper, NoteMapper)
    except Exception as e:
        print(f"❌ Import error: {e}")
        import traceback
        traceback.print_exc()
        return False, None

def test_basic_functionality(classes):
    """Test basic functionality of core components"""
    print("\nTesting basic functionality...")
    
    try:
        HysteresisTrigger, BasicMapper, NoteMapper = classes
        
        # Test trigger strategy
        config = {
            'thresholds': {'flex': 200}, 
            'hysteresis': {'flex': 10}, 
            'trigger_sensors': {'l': 'flex', 'r': 'flex'}
        }
        trigger = HysteresisTrigger(config)
        print("✅ HysteresisTrigger initialized successfully")
        
        # Test mapping strategy
        note_mapper = NoteMapper(root_note='C', scale='major')
        mapper = BasicMapper(note_mapper, {})
        print("✅ BasicMapper initialized successfully")
        
        # Test note mappings
        left_notes = mapper.get_note_mapping('l')
        right_notes = mapper.get_note_mapping('r')
        
        print(f"✅ Left hand notes: {left_notes}")
        print(f"✅ Right hand notes: {right_notes}")
        
        # Test trigger processing
        import numpy as np
        sensor_data = {'flex': [100, 100, 300, 100, 100]}  # Third finger above threshold
        trigger_events = trigger.process_triggers(sensor_data, 'r')
        print(f"✅ Trigger processing: {trigger_events}")
        
        # Test note mapping
        notes = mapper.map_to_notes(trigger_events, 'r')
        print(f"✅ Note mapping: {notes}")
        
        return True
        
    except Exception as e:
        print(f"❌ Functionality error: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main test function"""
    print("GLUVN Basic Functionality Test")
    print("=" * 40)
    
    imports_ok, classes = test_imports()
    if not imports_ok:
        print("\n❌ Import tests failed. Cannot continue.")
        return False
    
    functionality_ok = test_basic_functionality(classes)
    if not functionality_ok:
        print("\n❌ Functionality tests failed.")
        return False
    
    print("\n🎉 All tests passed! The system is ready to use.")
    print("\nYou can now run:")
    print("  python examples/ten_finger_demo.py    # Demo version")
    print("  python examples/ten_finger_gui.py     # Hardware version")
    
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 