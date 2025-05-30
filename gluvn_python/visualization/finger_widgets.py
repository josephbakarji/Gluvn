"""
Finger Sensor Visualization Widgets

Reusable PyQt5 widgets for displaying finger sensor data and trigger states.
These widgets can be used in different GUI applications for debugging,
monitoring, and user feedback.

Author: Joseph Bakarji
"""

import sys
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                            QProgressBar, QGroupBox, QSizePolicy)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QColor, QPaintEvent, QPainter, QBrush
import numpy as np


class FingerSensorWidget(QWidget):
    """
    Enhanced finger widget showing both sensor value and trigger state
    
    Features:
    - Real-time sensor value display
    - Visual progress bar with threshold indicator
    - Trigger state light (on/off)
    - Configurable thresholds and sensor types
    """
    
    def __init__(self, finger_name, note_name, sensor_type="flex"):
        super().__init__()
        self.finger_name = finger_name
        self.note_name = note_name
        self.sensor_type = sensor_type
        self.is_triggered = False
        self.sensor_value = 0
        self.threshold = 120
        
        self.setFixedSize(120, 200)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup the widget UI components"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)
        
        # Finger name label
        self.finger_label = QLabel(self.finger_name)
        self.finger_label.setAlignment(Qt.AlignCenter)
        self.finger_label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(self.finger_label)
        
        # Sensor value display
        self.value_label = QLabel("0")
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.value_label.setStyleSheet("color: blue; background-color: lightgray; padding: 2px;")
        layout.addWidget(self.value_label)
        
        # Sensor value progress bar (vertical)
        self.sensor_bar = QProgressBar()
        self.sensor_bar.setOrientation(Qt.Vertical)
        self.sensor_bar.setRange(0, 1023)  # Typical Arduino analog range
        self.sensor_bar.setValue(0)
        self.sensor_bar.setFixedHeight(80)
        self._update_progress_bar_style()
        layout.addWidget(self.sensor_bar)
        
        # Threshold indicator
        self.threshold_label = QLabel(f"Thresh: {self.threshold}")
        self.threshold_label.setAlignment(Qt.AlignCenter)
        self.threshold_label.setFont(QFont("Arial", 8))
        self.threshold_label.setStyleSheet("color: red;")
        layout.addWidget(self.threshold_label)
        
        # Trigger light (circle)
        self.light_widget = QWidget()
        self.light_widget.setFixedSize(40, 40)
        layout.addWidget(self.light_widget, 0, Qt.AlignCenter)
        
        # Note label
        self.note_label = QLabel(self.note_name)
        self.note_label.setAlignment(Qt.AlignCenter)
        self.note_label.setFont(QFont("Arial", 9))
        layout.addWidget(self.note_label)
    
    def _update_progress_bar_style(self):
        """Update progress bar styling based on current state"""
        if self.sensor_value >= self.threshold:
            color = "#e74c3c"  # Red when above threshold
        else:
            color = "#3498db"  # Blue when below threshold
        
        self.sensor_bar.setStyleSheet(f"""
            QProgressBar {{
                border: 2px solid grey;
                border-radius: 3px;
                text-align: center;
                background-color: #f0f0f0;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 2px;
            }}
        """)
    
    def paintEvent(self, event):
        """Custom paint event to draw the trigger light and threshold line"""
        super().paintEvent(event)
        
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw trigger light circle
        light_rect = self.light_widget.geometry()
        center_x = light_rect.x() + light_rect.width() // 2
        center_y = light_rect.y() + light_rect.height() // 2
        radius = 15
        
        if self.is_triggered:
            color = QColor(0, 255, 0)  # Green when triggered
        else:
            color = QColor(139, 0, 0)  # Dark red when not triggered
        
        painter.setBrush(QBrush(color))
        painter.setPen(QColor(0, 0, 0))
        painter.drawEllipse(center_x - radius, center_y - radius, radius * 2, radius * 2)
        
        # Draw threshold line on progress bar
        bar_rect = self.sensor_bar.geometry()
        threshold_y = bar_rect.y() + bar_rect.height() - int((self.threshold / 1023.0) * bar_rect.height())
        painter.setPen(QColor(255, 0, 0, 180))  # Semi-transparent red
        painter.drawLine(bar_rect.x(), threshold_y, bar_rect.x() + bar_rect.width(), threshold_y)
    
    def update_sensor_value(self, value):
        """Update the sensor value display"""
        self.sensor_value = int(value)
        self.value_label.setText(str(self.sensor_value))
        self.sensor_bar.setValue(self.sensor_value)
        self._update_progress_bar_style()
        self.update()  # Trigger repaint for threshold line
    
    def set_triggered(self, triggered):
        """Set the trigger state"""
        if triggered != self.is_triggered:
            self.is_triggered = triggered
            self.update()  # Trigger repaint
    
    def set_threshold(self, threshold):
        """Update the threshold value"""
        self.threshold = threshold
        self.threshold_label.setText(f"Thresh: {threshold}")
        self.update()  # Trigger repaint for threshold line
    
    def update_note(self, note_name):
        """Update the note name"""
        self.note_name = note_name
        self.note_label.setText(note_name)


class TenFingerDisplay(QWidget):
    """
    Complete display widget for both hands (10 fingers)
    
    This widget provides a standardized layout for displaying all finger
    sensors in a two-hand configuration with customizable note mappings.
    """
    
    def __init__(self, note_maps=None, sensor_type="flex"):
        super().__init__()
        self.note_maps = note_maps or {'l': [], 'r': []}
        self.sensor_type = sensor_type
        self.finger_widgets = {'l': [], 'r': []}
        
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup the complete hands display"""
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(20)
        
        # Left hand
        left_group = QGroupBox("Left Hand")
        main_layout.addWidget(left_group)
        left_layout = QHBoxLayout(left_group)
        
        finger_names_left = ["Pinky", "Ring", "Middle", "Index", "Thumb"]
        for i, finger_name in enumerate(finger_names_left):
            note_name = "---"
            if 'l' in self.note_maps and i < len(self.note_maps['l']):
                # Note mapping conversion would go here
                note_name = str(self.note_maps['l'][i])
            
            widget = FingerSensorWidget(finger_name, note_name, self.sensor_type)
            self.finger_widgets['l'].append(widget)
            left_layout.addWidget(widget)
        
        # Right hand
        right_group = QGroupBox("Right Hand")
        main_layout.addWidget(right_group)
        right_layout = QHBoxLayout(right_group)
        
        finger_names_right = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        for i, finger_name in enumerate(finger_names_right):
            note_name = "---"
            if 'r' in self.note_maps and i < len(self.note_maps['r']):
                # Note mapping conversion would go here
                note_name = str(self.note_maps['r'][i])
            
            widget = FingerSensorWidget(finger_name, note_name, self.sensor_type)
            self.finger_widgets['r'].append(widget)
            right_layout.addWidget(widget)
    
    def update_sensor_value(self, hand, finger_idx, value):
        """Update sensor value for specific finger"""
        if hand in self.finger_widgets and finger_idx < len(self.finger_widgets[hand]):
            self.finger_widgets[hand][finger_idx].update_sensor_value(value)
    
    def set_triggered(self, hand, finger_idx, triggered):
        """Set trigger state for specific finger"""
        if hand in self.finger_widgets and finger_idx < len(self.finger_widgets[hand]):
            self.finger_widgets[hand][finger_idx].set_triggered(triggered)
    
    def set_threshold(self, threshold):
        """Update threshold for all finger widgets"""
        for hand in ['l', 'r']:
            for widget in self.finger_widgets[hand]:
                widget.set_threshold(threshold)
    
    def update_note_maps(self, note_maps, note_mapper=None):
        """Update note mappings for all fingers"""
        self.note_maps = note_maps
        
        for hand in ['l', 'r']:
            if hand in note_maps:
                for i, widget in enumerate(self.finger_widgets[hand]):
                    if i < len(note_maps[hand]):
                        note_value = note_maps[hand][i]
                        if note_mapper and hasattr(note_mapper, 'midi2note'):
                            note_name = note_mapper.midi2note.get(note_value, str(note_value))
                        else:
                            note_name = str(note_value)
                        widget.update_note(note_name) 