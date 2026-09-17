"""Shared paths and device settings for the core package."""

from pathlib import Path

portR = 'COM3'   # Right hand glove
portL = 'COM4'   # Left hand glove

baud = 500000 #aligned with firmware baud rate
IACDriver         = 'loopMIDI Port'
keyboard_portname = 'loopMIDI Port'

mainDir = str(Path(__file__).resolve().parents[1])
figDir = str(Path(mainDir) / 'figures')
learnDir = str(Path(mainDir) / 'data' / 'keyboard_learning')
EXPDIR = str(Path(mainDir) / 'data' / 'experiments')

# BLE config (Nordic UART Service — must match Arduino BLEDevice::init() name)
BLE_NAME_L = "Gluvn_L"
BLE_NAME_R = "Gluvn_R"

NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # PC → M5 (write)
NUS_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # M5 → PC (notify)