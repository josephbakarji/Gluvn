"""
Unit tests for trigger strategies

Tests the core trigger logic implementations to ensure they correctly
convert sensor data to trigger events with proper hysteresis behavior.
"""

import unittest
import numpy as np
import sys
import os

# Add the parent directory to the path so we can import from core
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from core.strategies.trigger_strategies import HysteresisTrigger, MultiSensorTrigger


class TestHysteresisTrigger(unittest.TestCase):
    """Test the HysteresisTrigger strategy"""
    
    def setUp(self):
        """Set up test configuration"""
        self.config = {
            'thresholds': {'flex': 200, 'press': 15},
            'hysteresis': {'flex': 5, 'press': 5},
            'trigger_sensors': {'l': 'flex', 'r': 'flex'}
        }
        self.trigger = HysteresisTrigger(self.config)
    
    def test_initialization(self):
        """Test that trigger strategy initializes correctly"""
        self.assertIsNotNone(self.trigger.thresholds)
        self.assertIsNotNone(self.trigger.hysteresis)
        self.assertIsNotNone(self.trigger.trigger_sensors)
        
        # Check that trigger states are initialized for both hands
        self.assertTrue('l' in self.trigger.trigger_states)
        self.assertTrue('r' in self.trigger.trigger_states)
        
        # Check that states are initially all False
        np.testing.assert_array_equal(self.trigger.trigger_states['l'], np.zeros(5, dtype=bool))
        np.testing.assert_array_equal(self.trigger.trigger_states['r'], np.zeros(5, dtype=bool))
    
    def test_basic_triggering_above_threshold(self):
        """Test basic trigger functionality when values are above threshold"""
        # Sensor values above threshold (200 + 5 = 205)
        sensor_data = {'flex': [250, 250, 250, 250, 250]}
        
        # First call should trigger all fingers
        events = self.trigger.process_triggers(sensor_data, 'r')
        expected = np.array([1, 1, 1, 1, 1])  # All fingers turn on
        np.testing.assert_array_equal(events, expected)
        
        # Check that trigger states are updated
        np.testing.assert_array_equal(self.trigger.trigger_states['r'], np.ones(5, dtype=bool))
        
        # Second call with same data should produce no events (already triggered)
        events = self.trigger.process_triggers(sensor_data, 'r')
        expected = np.array([0, 0, 0, 0, 0])  # No change
        np.testing.assert_array_equal(events, expected)
    
    def test_basic_triggering_below_threshold(self):
        """Test trigger turn-off when values go below threshold"""
        # First trigger fingers on
        sensor_data_high = {'flex': [250, 250, 250, 250, 250]}
        self.trigger.process_triggers(sensor_data_high, 'r')
        
        # Then trigger fingers off with values below threshold (200 - 5 = 195)
        sensor_data_low = {'flex': [150, 150, 150, 150, 150]}
        events = self.trigger.process_triggers(sensor_data_low, 'r')
        expected = np.array([-1, -1, -1, -1, -1])  # All fingers turn off
        np.testing.assert_array_equal(events, expected)
        
        # Check that trigger states are updated
        np.testing.assert_array_equal(self.trigger.trigger_states['r'], np.zeros(5, dtype=bool))
    
    def test_hysteresis_behavior(self):
        """Test that hysteresis prevents false triggers"""
        # Values just above threshold but within hysteresis band
        sensor_data = {'flex': [202, 202, 202, 202, 202]}  # Above 200 but below 205
        
        # Should not trigger (within hysteresis band)
        events = self.trigger.process_triggers(sensor_data, 'r')
        expected = np.array([0, 0, 0, 0, 0])  # No triggers
        np.testing.assert_array_equal(events, expected)
        
        # Values well above threshold
        sensor_data_high = {'flex': [250, 250, 250, 250, 250]}
        events = self.trigger.process_triggers(sensor_data_high, 'r')
        expected = np.array([1, 1, 1, 1, 1])  # Should trigger
        np.testing.assert_array_equal(events, expected)
    
    def test_individual_finger_triggering(self):
        """Test that individual fingers can be triggered independently"""
        # Trigger only first and third fingers
        sensor_data = {'flex': [250, 150, 250, 150, 150]}
        
        events = self.trigger.process_triggers(sensor_data, 'r')
        expected = np.array([1, 0, 1, 0, 0])  # Only fingers 0 and 2 trigger
        np.testing.assert_array_equal(events, expected)
        
        # Check trigger states
        expected_state = np.array([True, False, True, False, False])
        np.testing.assert_array_equal(self.trigger.trigger_states['r'], expected_state)
    
    def test_missing_sensor_data(self):
        """Test behavior when sensor data is missing"""
        # No flex data
        sensor_data = {'press': [10, 10, 10, 10, 10]}
        
        events = self.trigger.process_triggers(sensor_data, 'r')
        expected = np.array([0, 0, 0, 0, 0])  # No triggers
        np.testing.assert_array_equal(events, expected)
    
    def test_wrong_hand(self):
        """Test behavior with invalid hand identifier"""
        sensor_data = {'flex': [250, 250, 250, 250, 250]}
        
        events = self.trigger.process_triggers(sensor_data, 'invalid_hand')
        expected = np.array([0, 0, 0, 0, 0])  # No triggers
        np.testing.assert_array_equal(events, expected)
    
    def test_threshold_update(self):
        """Test dynamic threshold updating"""
        # Set new threshold
        self.trigger.set_threshold('flex', 300)
        
        # Values that would trigger with old threshold (200) but not new (300)
        sensor_data = {'flex': [250, 250, 250, 250, 250]}
        
        events = self.trigger.process_triggers(sensor_data, 'r')
        expected = np.array([0, 0, 0, 0, 0])  # No triggers with higher threshold
        np.testing.assert_array_equal(events, expected)
        
        # Values above new threshold
        sensor_data_high = {'flex': [350, 350, 350, 350, 350]}
        events = self.trigger.process_triggers(sensor_data_high, 'r')
        expected = np.array([1, 1, 1, 1, 1])  # Should trigger
        np.testing.assert_array_equal(events, expected)
    
    def test_different_hands_independent(self):
        """Test that left and right hands operate independently"""
        sensor_data = {'flex': [250, 250, 250, 250, 250]}
        
        # Trigger right hand
        events_r = self.trigger.process_triggers(sensor_data, 'r')
        expected = np.array([1, 1, 1, 1, 1])
        np.testing.assert_array_equal(events_r, expected)
        
        # Left hand should still be untriggered
        np.testing.assert_array_equal(self.trigger.trigger_states['l'], np.zeros(5, dtype=bool))
        
        # Trigger left hand
        events_l = self.trigger.process_triggers(sensor_data, 'l')
        expected = np.array([1, 1, 1, 1, 1])
        np.testing.assert_array_equal(events_l, expected)
        
        # Both hands should now be triggered
        np.testing.assert_array_equal(self.trigger.trigger_states['r'], np.ones(5, dtype=bool))
        np.testing.assert_array_equal(self.trigger.trigger_states['l'], np.ones(5, dtype=bool))


class TestMultiSensorTrigger(unittest.TestCase):
    """Test the MultiSensorTrigger strategy"""
    
    def setUp(self):
        """Set up test configuration for multi-sensor triggering"""
        self.config = {
            'sensor_configs': {
                'r': [
                    {'type': 'flex', 'weight': 0.7},
                    {'type': 'press', 'weight': 0.3}
                ],
                'l': [
                    {'type': 'flex', 'weight': 1.0}
                ]
            },
            'fusion_method': 'weighted_sum',
            'thresholds': {'combined': 200},
            'hysteresis': {'combined': 5}
        }
        self.trigger = MultiSensorTrigger(self.config)
    
    def test_weighted_sum_fusion(self):
        """Test weighted sum fusion of multiple sensors"""
        # Sensor data with both flex and pressure
        sensor_data = {
            'flex': [300, 300, 300, 300, 300],  # High flex values
            'press': [100, 100, 100, 100, 100]  # Moderate pressure values
        }
        
        # Expected fused value: 300 * 0.7 + 100 * 0.3 = 210 + 30 = 240
        # This should be above threshold (200 + 5 = 205)
        events = self.trigger.process_triggers(sensor_data, 'r')
        expected = np.array([1, 1, 1, 1, 1])  # Should trigger
        np.testing.assert_array_equal(events, expected)
    
    def test_single_sensor_fallback(self):
        """Test behavior when only one sensor type is available"""
        # Only flex data available
        sensor_data = {'flex': [250, 250, 250, 250, 250]}
        
        # Left hand is configured for single sensor, should work
        events = self.trigger.process_triggers(sensor_data, 'l')
        expected = np.array([1, 1, 1, 1, 1])  # Should trigger
        np.testing.assert_array_equal(events, expected)


if __name__ == '__main__':
    unittest.main() 