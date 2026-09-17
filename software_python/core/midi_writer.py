"""
GLUVN MIDI Writer Module

Converts sensor events into MIDI messages and sends them to Windows via
winmm.dll natively (ctypes), bypassing Mido/RtMidi -- for Python 3.14
environments without C++ build tools. Targets a loopMIDI virtual port by
name (IACDriver in core/__init__.py).

Supported: note on/off, pitch bend, aftertouch (channel pressure),
control change, polyphonic aftertouch.

Usage:
    midi_writer = MidiWriter()
    midi_writer.trig_note(60, 80)  # middle C, velocity 80
    midi_writer.pitch_bend(1000)
    midi_writer.aftertouch(100)

Author: Joseph Bakarji
Created: Mar 31, 2016
Last Updated: June 2026
"""

import ctypes
import threading
import queue
from core.__init__ import IACDriver

# Windows Multimedia API constants
MIDI_MAPPER = -1
MMSYSERR_NOERROR = 0
CALLBACK_NULL = 0


class MidiWriter:
    def __init__(self):
        self.winmm = ctypes.WinDLL('winmm.dll')
        self.h_midi_out = ctypes.c_void_p()

        num_devs = self.winmm.midiOutGetNumDevs()
        target_port_name = IACDriver
        port_index = MIDI_MAPPER

        caps = ctypes.create_string_buffer(84)
        matched = False

        for i in range(num_devs):
            if self.winmm.midiOutGetDevCapsA(i, caps, len(caps)) == MMSYSERR_NOERROR:
                # Name starts at 8-byte offset in the MIDIOUTCAPS struct
                name = caps[8:].split(b'\x00')[0].decode('utf-8', errors='ignore')
                if target_port_name in name:
                    port_index = i
                    matched = True
                    print(f"Hardware bind successful: {name} (Index {i})")
                    break

        if not matched:
            raise ConnectionError(
                f"Virtual port '{target_port_name}' not detected by WinMM. "
                "Ensure loopMIDI is active."
            )

        result = self.winmm.midiOutOpen(
            ctypes.byref(self.h_midi_out),
            port_index,
            CALLBACK_NULL,
            0,
            CALLBACK_NULL
        )

        if result != MMSYSERR_NOERROR:
            raise ConnectionError(f"WinMM port allocation failed. Code: {result}")

    def _send_short_msg(self, status, data1, data2):
        """Bitpack a standard 3-byte MIDI message into a 32-bit DWORD for the Windows API."""
        msg = status | (data1 << 8) | (data2 << 16)
        result = self.winmm.midiOutShortMsg(self.h_midi_out, msg)
        if result != MMSYSERR_NOERROR:
            print(f"Warning: MIDI message send failed. Code: {result}")

    def trig_note(self, notemidi, vel):
        self._send_short_msg(0x90, int(notemidi), int(vel))   # Note On, Ch 1

    def pitch_bend(self, pitch_change):
        # Raw offset -> 14-bit unsigned pitch bend (0-16383)
        normalized = int(pitch_change + 8192)
        normalized = max(0, min(16383, normalized))
        lsb = normalized & 0x7F
        msb = (normalized >> 7) & 0x7F
        self._send_short_msg(0xE0, lsb, msb)   # Pitch Bend, Ch 1

    def aftertouch(self, val, channel=1):
        self._send_short_msg(0xD0 + (channel - 1), int(val), 0)   # Channel Pressure

    def control_change(self, val, control, channel=1):
        self._send_short_msg(0xB0 + (channel - 1), int(control), int(val))   # Control Change

    def polytouch(self, notemidi, val, channel=1):
        self._send_short_msg(0xA0 + (channel - 1), int(notemidi), int(val))   # Polyphonic Aftertouch

    def stop(self):
        self.winmm.midiOutReset(self.h_midi_out)

    def __del__(self):
        if hasattr(self, 'h_midi_out') and self.h_midi_out:
            self.winmm.midiOutClose(self.h_midi_out)

    def trig_notes_array_playing(self, switch, playing_notes, notearr, vel=80):
        for i in range(len(switch)):
            if(switch[i] == 1):
                self.trig_note(notearr[i], vel)
                playing_notes[i] = notearr[i]
            if(switch[i] == -1):
                self.trig_note(playing_notes[i], 0)
                playing_notes[i] = None
        return playing_notes

    def turn_off_all_playing(self, playing_notes):
        for i in range(len(playing_notes)):
            if playing_notes[i] is not None:
                self.trig_note(playing_notes[i], 0)
                playing_notes[i] = None
        return playing_notes

    def trig_note_array(self, switch, notearr, vel=80):
        for i in range(len(switch)):
            if(switch[i] == 1):
                self.trig_note(notearr[i], vel)
            if(switch[i] == -1):
                self.trig_note(notearr[i], 0)


class MidiWriterThread(threading.Thread):
    def __init__(self, midi_writer):
        super().__init__()
        self.midi_writer = midi_writer
        self.msg_queue = queue.Queue()
        self.running = True

    def run(self):
        while self.running:
            try:
                msg = self.msg_queue.get(timeout=0.1)
                self._send_midi(msg)
            except queue.Empty:
                continue

    def _send_midi(self, msg):
        method, args, kwargs = msg
        if hasattr(self.midi_writer, method):
            getattr(self.midi_writer, method)(*args, **kwargs)

    def send(self, method, *args, **kwargs):
        self.msg_queue.put((method, args, kwargs))

    def stop(self):
        self.running = False
