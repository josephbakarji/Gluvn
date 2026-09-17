"""
Piano Keyboard Widget for GLUVN

Displays a piano keyboard covering the range C1-C6 with visual feedback
for triggered notes. This widget integrates with the GLUVN system to
show which notes are currently being played.

Based on concepts from the daw-tools library and PyShine piano examples.

Author: Joseph Bakarji
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen, QFont
import numpy as np


class PianoKey(QWidget):
    """Individual piano key widget"""
    
    # Signal emitted when key is pressed/released  
    key_pressed = pyqtSignal(int, bool)  # midi_note, is_pressed
    
    def __init__(self, midi_note, is_black_key=False, parent=None):
        super().__init__(parent)
        self.midi_note = midi_note
        self.is_black_key = is_black_key
        self.is_triggered = False
        self.is_hovered = False
        
        # Set dimensions
        if is_black_key:
            self.setFixedSize(20, 80)
        else:
            self.setFixedSize(30, 120)
        
        # Set up styling
        self._update_style()
    
    def _update_style(self):
        """Update the visual styling based on current state"""
        if self.is_black_key:
            if self.is_triggered:
                bg_color = "#ff6b6b"  # Red when triggered
            elif self.is_hovered:
                bg_color = "#444444"  # Dark gray when hovered
            else:
                bg_color = "#000000"  # Black normally
        else:
            if self.is_triggered:
                bg_color = "#ff6b6b"  # Red when triggered
            elif self.is_hovered:
                bg_color = "#f0f0f0"  # Light gray when hovered
            else:
                bg_color = "#ffffff"  # White normally
        
        border_color = "#333333"
        
        self.setStyleSheet(f"""
            PianoKey {{
                background-color: {bg_color};
                border: 1px solid {border_color};
                border-radius: 3px;
            }}
        """)
    
    def set_triggered(self, triggered):
        """Set the triggered state of the key"""
        if triggered != self.is_triggered:
            self.is_triggered = triggered
            self._update_style()
    
    def enterEvent(self, event):
        """Handle mouse enter event"""
        self.is_hovered = True
        self._update_style()
        super().enterEvent(event)
    
    def leaveEvent(self, event):
        """Handle mouse leave event"""
        self.is_hovered = False
        self._update_style()
        super().leaveEvent(event)
    
    def mousePressEvent(self, event):
        """Handle mouse press event"""
        if event.button() == Qt.LeftButton:
            self.key_pressed.emit(self.midi_note, True)
        super().mousePressEvent(event)
    
    def mouseReleaseEvent(self, event):
        """Handle mouse release event"""
        if event.button() == Qt.LeftButton:
            self.key_pressed.emit(self.midi_note, False)
        super().mouseReleaseEvent(event)


class PianoKeyboard(QWidget):
    """Piano keyboard widget covering C1-C6 range"""
    
    # Signal emitted when a key is pressed/released
    key_pressed = pyqtSignal(int, bool)  # midi_note, is_pressed
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # MIDI note range: C1 (24) to C6 (84) = 61 keys
        self.start_note = 24  # C1
        self.end_note = 84    # C6
        
        # Dictionary to store key widgets
        self.keys = {}
        
        # Set up the UI
        self._setup_ui()
    
    def _setup_ui(self):
        """Set up the keyboard layout"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)
        
        # Title
        title = QLabel("Piano Keyboard (C1 - C6)")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(title)
        
        # Keyboard container
        keyboard_container = QWidget()
        keyboard_container.setFixedHeight(140)
        layout.addWidget(keyboard_container)
        
        # Create the keyboard
        self._create_keyboard(keyboard_container)
        
        # Add octave labels
        octave_layout = QHBoxLayout()
        octave_layout.setContentsMargins(15, 0, 15, 0)
        
        for octave in range(1, 7):  # C1 to C6
            octave_label = QLabel(f"C{octave}")
            octave_label.setAlignment(Qt.AlignCenter)
            octave_label.setFont(QFont("Arial", 8))
            octave_layout.addWidget(octave_label)
        
        layout.addLayout(octave_layout)
    
    def _create_keyboard(self, container):
        """Create the piano keyboard with proper key layout"""
        # White keys layout
        white_key_positions = []
        black_key_positions = []
        
        # Pattern for one octave (starting from C)
        # White keys: C, D, E, F, G, A, B (7 keys)
        # Black keys: C#, D#, F#, G#, A# (5 keys, positioned between white keys)
        
        white_key_count = 0
        
        for midi_note in range(self.start_note, self.end_note + 1):
            note_in_octave = midi_note % 12
            
            if self._is_white_key(note_in_octave):
                # White key
                key = PianoKey(midi_note, is_black_key=False, parent=container)
                key.key_pressed.connect(self.key_pressed.emit)
                
                # Position white key
                x_pos = white_key_count * 30
                key.move(x_pos, 20)
                
                self.keys[midi_note] = key
                white_key_count += 1
            else:
                # Black key
                key = PianoKey(midi_note, is_black_key=True, parent=container)
                key.key_pressed.connect(self.key_pressed.emit)
                
                # Position black key between white keys
                x_pos = self._get_black_key_position(midi_note, white_key_count)
                key.move(x_pos, 20)
                
                # Raise black keys above white keys
                key.raise_()
                
                self.keys[midi_note] = key
        
        # Set container size based on number of white keys
        container.setFixedWidth(white_key_count * 30 + 10)
    
    def _is_white_key(self, note_in_octave):
        """Check if a note is a white key"""
        # White keys: C(0), D(2), E(4), F(5), G(7), A(9), B(11)
        return note_in_octave in [0, 2, 4, 5, 7, 9, 11]
    
    def _get_black_key_position(self, midi_note, current_white_count):
        """Calculate the x position for a black key"""
        note_in_octave = midi_note % 12
        
        # Black key positions relative to white keys:
        # C# = between C and D
        # D# = between D and E  
        # F# = between F and G
        # G# = between G and A
        # A# = between A and B
        
        if note_in_octave == 1:  # C#
            return (current_white_count - 1) * 30 + 20
        elif note_in_octave == 3:  # D#
            return (current_white_count - 1) * 30 + 20
        elif note_in_octave == 6:  # F#
            return (current_white_count - 1) * 30 + 20
        elif note_in_octave == 8:  # G#
            return (current_white_count - 1) * 30 + 20
        elif note_in_octave == 10:  # A#
            return (current_white_count - 1) * 30 + 20
        
        return 0
    
    def set_note_triggered(self, midi_note, triggered):
        """Set the triggered state for a specific MIDI note"""
        if midi_note in self.keys:
            self.keys[midi_note].set_triggered(triggered)
    
    def clear_all_triggers(self):
        """Clear all triggered states"""
        for key in self.keys.values():
            key.set_triggered(False)
    
    def get_note_name(self, midi_note):
        """Get the note name for a MIDI note number"""
        note_names = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
        octave = (midi_note // 12) - 1
        note = note_names[midi_note % 12]
        return f"{note}{octave}"


class CompactPianoKeyboard(QWidget):
    """Compact version of piano keyboard for tight layouts"""
    
    key_pressed = pyqtSignal(int, bool)
    
    def __init__(self, start_octave=3, num_octaves=2, parent=None):
        super().__init__(parent)
        
        self.start_octave = start_octave
        self.num_octaves = num_octaves
        # MIDI note calculation: C1 = 24, so C[octave] = 12 + octave*12
        self.start_note = 12 + start_octave * 12  # C of start_octave
        self.end_note = self.start_note + (num_octaves * 12) - 1
        
        self.keys = {}
        self._setup_compact_ui()
    
    def _setup_compact_ui(self):
        """Set up compact keyboard layout"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)
        
        # Compact title
        title = QLabel(f"Keyboard (C{self.start_octave}-C{self.start_octave + self.num_octaves})")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(title)
        
        # Keyboard container
        keyboard_container = QWidget()
        keyboard_container.setFixedHeight(80)
        layout.addWidget(keyboard_container)
        
        self._create_compact_keyboard(keyboard_container)
    
    def _create_compact_keyboard(self, container):
        """Create compact keyboard with smaller keys"""
        white_key_count = 0
        
        for midi_note in range(self.start_note, self.end_note + 1):
            note_in_octave = midi_note % 12
            
            if self._is_white_key(note_in_octave):
                # Smaller white key
                key = PianoKey(midi_note, is_black_key=False, parent=container)
                key.setFixedSize(18, 60)  # Smaller than normal
                key.key_pressed.connect(self.key_pressed.emit)
                
                x_pos = white_key_count * 18
                key.move(x_pos, 10)
                
                self.keys[midi_note] = key
                white_key_count += 1
            else:
                # Smaller black key
                key = PianoKey(midi_note, is_black_key=True, parent=container)
                key.setFixedSize(12, 40)  # Smaller than normal
                key.key_pressed.connect(self.key_pressed.emit)
                
                x_pos = self._get_compact_black_key_position(midi_note, white_key_count)
                key.move(x_pos, 10)
                key.raise_()
                
                self.keys[midi_note] = key
        
        container.setFixedWidth(white_key_count * 18 + 5)
    
    def _is_white_key(self, note_in_octave):
        """Check if a note is a white key"""
        return note_in_octave in [0, 2, 4, 5, 7, 9, 11]
    
    def _get_compact_black_key_position(self, midi_note, current_white_count):
        """Calculate position for compact black key"""
        note_in_octave = midi_note % 12
        
        if note_in_octave in [1, 3, 6, 8, 10]:  # Black keys
            return (current_white_count - 1) * 18 + 12
        
        return 0
    
    def set_note_triggered(self, midi_note, triggered):
        """Set triggered state for a note"""
        if midi_note in self.keys:
            self.keys[midi_note].set_triggered(triggered)
    
    def clear_all_triggers(self):
        """Clear all triggered states"""
        for key in self.keys.values():
            key.set_triggered(False)
    
    def get_note_name(self, midi_note):
        """Get the note name for a MIDI note number"""
        note_names = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
        octave = (midi_note // 12) - 1
        note = note_names[midi_note % 12]
        return f"{note}{octave}" 