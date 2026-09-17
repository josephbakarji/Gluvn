"""
GLUVN Port Reader Module — BLE + Serial support
"""

from __future__ import division
from __init__ import BLE_NAME_L, BLE_NAME_R, NUS_TX_CHAR_UUID, NUS_RX_CHAR_UUID, baud, simDir, EXPDIR, learnDir
from data_analysis import ReadWrite
import serial
import time
import numpy as np
import asyncio
import threading
import queue
import os
from threading import Thread
from struct import *
from bleak import BleakClient, BleakScanner


class Reader:
    def __init__(self,
                 directory=EXPDIR,
                 save_file='test000',
                 parse_file=None,
                 message_format=None,
                 length_checksum=24,
                 sensor_config={'l': {'flex': True, 'press': True, 'imu': True}},
                 save=False,
                 use_ble=True):       # NEW: BLE mode toggle

        self.directory        = directory
        self.baud             = baud
        self.hands            = list(sensor_config.keys())
        self.sensor_config    = sensor_config
        self.save             = save
        self.save_file        = save_file
        self.message_format   = message_format or '>sBHHHHHHBBBBBBBBBB'
        self.length_checksum  = length_checksum
        self.parse_qsize      = 0 if self.save else 30
        self.parse_file       = None if parse_file is None else os.path.join(directory, parse_file)
        self.use_ble          = use_ble
        self.threads          = self.make_reader_threads()

    def make_reader_threads(self):
        import serial
        time0 = time.time()
        threads = {hand: {} for hand in self.hands}
        failed = []
        for hand in self.hands:
            if self.parse_file is None:
                if self.use_ble:
                    ble_name = BLE_NAME_R if hand == 'r' else BLE_NAME_L
                    threads[hand]['serial'] = ReadBLE(ble_name)
                else:
                    from __init__ import portL, portR
                    port = portR if hand == 'r' else portL
                    threads[hand]['port'] = port
                    try:
                        threads[hand]['serial'] = ReadSerial(port, self.baud)
                    except serial.SerialException as e:
                        print(f"Warning: could not open {port} for {hand} hand: {e}")
                        failed.append(hand)
                        continue
                threads[hand]['parser'] = ParseSerial(
                    threads[hand]['serial'].getQ(), time0,
                    format=self.message_format,
                    length_checksum=self.length_checksum,
                    **self.sensor_config[hand],
                    qsize=self.parse_qsize
                )
            else:
                threads[hand]['parser'] = ParseFile(self.directory, self.parse_file, hand)
        for hand in failed:
            del threads[hand]
            self.hands.remove(hand)
        if not self.hands:
            raise Exception("No hands connected — check COM ports or BLE devices")
        return threads

    def start_readers(self):
        for hand in self.hands:
            if self.parse_file is None:
                self.threads[hand]['serial'].start()
            self.threads[hand]['parser'].start()

    def stop_readers(self):
        for hand in self.hands:
            if self.parse_file is None:
                try:
                    self.threads[hand]['serial'].stop()
                except Exception:
                    pass

    def send_command(self, hand, cmd):
        """Send a command string to the glove over BLE or serial."""
        try:
            self.threads[hand]['serial'].send(f'{cmd}\n')
        except Exception as e:
            print(f"Command send error: {e}")


class ReadBLE(Thread):
    """
    Replaces ReadSerial for BLE mode.
    Runs an asyncio event loop in a background thread.
    Incoming notify data → sensq (same interface as ReadSerial).
    Outgoing commands → write to RX characteristic.
    """
    def __init__(self, device_name, qsize=48):
        Thread.__init__(self)
        self.daemon          = True
        self.device_name     = device_name
        self.sensq           = queue.Queue(maxsize=qsize)
        self._stop_flag      = threading.Event()
        self._connected_event = threading.Event()
        self._loop           = asyncio.new_event_loop()
        self._client         = None
        self._write_q        = asyncio.Queue()  # Commands to send to M5

    def run(self):
        self._loop.run_until_complete(self._ble_loop())

    def stop(self):
        self._stop_flag.set()

    def getQ(self):
        return self.sensq

    def wait_connected(self, timeout=20.0):
        """Block until BLE is connected (or timeout). Returns True if connected."""
        return self._connected_event.wait(timeout)

    def send(self, text):
        """Thread-safe: queue a command string to be written to M5 RX char."""
        asyncio.run_coroutine_threadsafe(
            self._write_q.put(text), self._loop
        )

    def _notification_handler(self, sender, data: bytearray):
        """Called by bleak on each BLE notify — push raw bytes to queue."""
        self.sensq.put(bytes(data))

    async def _ble_loop(self):
        print(f"Scanning for BLE device: {self.device_name}...")
        while not self._stop_flag.is_set():
            try:
                # Scan for device by name
                device = await BleakScanner.find_device_by_name(
                    self.device_name, timeout=10.0
                )
                if device is None:
                    print(f"Device '{self.device_name}' not found, retrying...")
                    await asyncio.sleep(2)
                    continue

                print(f"Connecting to {self.device_name}...")
                async with BleakClient(device) as client:
                    self._client = client
                    self._connected_event.set()
                    print(f"Connected to {self.device_name}")

                    # Note: on Windows (WinRT), MTU is negotiated automatically.
                    # Explicit request_mtu() can cause disconnects on some BLE stacks.
                    # await client.request_mtu(64)

                    # Subscribe to TX notifications (sensor data from M5)
                    await client.start_notify(NUS_TX_CHAR_UUID, self._notification_handler)

                    # Keep connection alive, process outgoing commands
                    while not self._stop_flag.is_set() and client.is_connected:
                        try:
                            cmd = await asyncio.wait_for(self._write_q.get(), timeout=0.1)
                            await client.write_gatt_char(
                                NUS_RX_CHAR_UUID,
                                cmd.encode() if isinstance(cmd, str) else cmd,
                                response=False
                            )
                        except asyncio.TimeoutError:
                            pass

                    self._connected_event.clear()

            except Exception as e:
                print(f"BLE error: {e} — reconnecting in 2s...")
                self._connected_event.clear()
                await asyncio.sleep(2)

        self._client = None


class ReadSerial(Thread):
    """Kept for wired fallback."""
    def __init__(self, port, baud, qsize=48):
        Thread.__init__(self)
        self.daemon      = True
        self.port        = port
        self.baud        = baud
        self.sensq       = queue.Queue(maxsize=qsize)
        self.serial_port = serial.Serial(self.port, self.baud, timeout=2)

    def run(self):
        print('Port name: ' + self.serial_port.name)
        while True:
            self.read()

    def read(self):
        try:
            temp = self.serial_port.read_until()
            if temp:
                self.sensq.put(temp)
        except Exception as e:
            print(f"Serial read error: {e}")
            time.sleep(0.1)

    def stop(self):
        try: self.serial_port.close()
        except: pass

    def send(self, text):
        try: self.serial_port.write(text.encode())
        except Exception as e: print(f"Serial send error: {e}")

    def getQ(self):
        return self.sensq


class ParseSerial(Thread):
    def __init__(self, sensq, time0, format='>sBHHHHHHBBBBBBBBBB',
                 length_checksum=24, add_timestamp=False,
                 flex=False, press=True, imu=False, qsize=30):
        Thread.__init__(self)
        self.daemon           = True
        self.sensq            = sensq
        self.time0            = time0
        self.format           = format
        self.length_checksum  = length_checksum
        self.dataq            = queue.Queue(maxsize=qsize)
        self.read_configs     = {'flex': flex, 'press': press, 'imu': imu}
        self.add_timestamp    = add_timestamp
        # Defining slices for various data types
        # Index 0 = hand, 1 = seq, 2-8 = imu, 8-13 = flex, 13-18 = press
        self.slices = {
            'imu': slice(2, 8),
            'flex': slice(8, 13),
            'press': slice(13, 18),
        }
        self._buf = b''  # Reassembly buffer for BLE fragmented packets

    def run(self):
        print('parsing starting ...')
        while True:
            self.parse()

    def parse(self):
        chunk = self.sensq.get(block=True)
        # BLE may deliver partial packets — accumulate until \n
        self._buf += chunk
        while b'\n' in self._buf:
            line, self._buf = self._buf.split(b'\n', 1)
            s = line
            if (s[:1] in (b'r', b'l')) and len(s) == self.length_checksum:
                unp = unpack(self.format, s)
                data_to_put = {}
                data_to_put['seq'] = unp[1]
                for key, active in self.read_configs.items():
                    if active:
                        data_to_put[key] = unp[self.slices[key]]
                if self.add_timestamp:
                    data_to_put['time'] = time.time() - self.time0
                self.dataq.put(data_to_put)

    def getQ(self):
        return self.dataq


class printSens(Thread):
    def __init__(self, sensorq, info=''):
        Thread.__init__(self)
        self.sensorq = sensorq
        self.info    = info
        self.daemon  = True

    def run(self):
        while True:
            print(self.info, self.sensorq.get(block=True))


class ParseFile(Thread):
    def __init__(self, directory=EXPDIR, filename='test', hand='R'):
        Thread.__init__(self)
        self.daemon    = True
        self.directory = directory
        self.filename  = filename
        self.hand      = hand
        self.dataq     = queue.Queue(maxsize=0)

    def run(self):
        print('file parsing starting ...')
        reader = ReadWrite(directory=self.directory, filename=self.filename)
        timeSens, pressData, flexData, imuData = reader.readSensors(hand=self.hand)
        time0 = time.time()
        for idx, next_time in enumerate(timeSens):
            while time.time() - time0 < next_time:
                pass
            self.dataq.put((time.time(), imuData[idx], flexData[idx], pressData[idx]))
        print('done')

    def getQ(self):
        return self.dataq