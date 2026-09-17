"""Shared test paths for legacy fixtures."""

from pathlib import Path

# Global Variables
portL = '/dev/cu.usbmodem101' # serial port name
portR = '/dev/cu.usbmodem1101' # serial port name

baud = 115200	# baud rate set from Arduino
IACDriver = 'IAC Driver todaw' # Virtual Midi Driver
keyboard_portname = 'Digital Piano'

mainDir = str(Path(__file__).resolve().parents[2])

figDir = str(Path(mainDir) / 'figures')

simDir = str(Path(mainDir) / 'data' / 'sim')
learnDir = str(Path(mainDir) / 'data' / 'keyboard_learning')
testDir = str(Path(mainDir) / 'data' / 'test')
settingsDir = str(Path(mainDir) / 'data' / 'settings')
EXPDIR = str(Path(mainDir) / 'data' / 'experiments')
