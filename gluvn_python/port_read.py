from __future__ import division
from __init__ import portL, portR, baud, simDir, EXPDIR, learnDir
from data_analysis import ReadWrite
import serial
import time
import numpy as np
import random
from threading import Thread
import binascii
from struct import *
from collections import namedtuple
import matplotlib.pyplot as plt
import math
import queue
import mido
from collections import deque
import csv, sys, os

## TODO:
# - Fix ParseFile
# - can either print or save because serial queue data is only read once

class Reader:
    def __init__(self, 
                 directory=EXPDIR, 
                 save_file='test000',
                 parse_file=None,
                 sensor_config={'r': {'flex': True, 'press': True, 'imu': True}},
                 save=False):

        self.directory = directory
        self.baud = baud # Add to inputs or remove from here
        self.hands = list(sensor_config.keys()) 
        self.sensor_config = sensor_config
        self.save = save
        self.save_file = save_file
        self.parse_qsize = 0 if self.save else 30
        self.parse_file = None if parse_file is None else os.path.join(directory, parse_file)
        self.threads = self.make_reader_threads()
        self.printers = self.make_printer_threads()
    
    def make_reader_threads(self):
        time0 = time.time() # Common starting time to sync multiple threads
        threads = {hand:{} for hand in self.hands}
        for hand in self.hands:
            if self.parse_file is None:
                threads[hand]['port'] = portR if hand == 'r' else portL
                threads[hand]['serial'] = ReadSerial(threads[hand]['port'], self.baud)
                threads[hand]['parser'] = ParseSerial(threads[hand]['serial'].getQ(), time0, 
                                                      **self.sensor_config[hand], qsize=self.parse_qsize)
            else:
                threads[hand]['parser'] = ParseFile(self.directory, self.parse_file, hand)
        return threads
    
    def start_readers(self):
        for hand in self.hands:
            if self.parse_file is None:
                self.threads[hand]['serial'].start()
            self.threads[hand]['parser'].start()

    def stop_readers(self):
        for hand in self.hands:
            if self.parse_file is None:
                self.threads[hand]['serial'].join(timeout=1)
            self.threads[hand]['parser'].join(timeout=1)

    def make_printer_threads(self):
        printers = {}
        for hand in self.hands:
            printers[hand] = printSens(self.threads[hand]['parser'].getQ())
        return printers

    def start_printers(self):
        # Check if reading threads are already running
        for hand in self.hands:
            self.printers[hand].start()

    def stop_printers(self):
        for hand in self.hands:
            self.printers[hand].join(timeout=1)    
    
    def runAndPrintSensors(self):
        self.start_readers()
        self.start_printers()
        key = input('press any key to finish \n')
        print('Shutting down...') 

        # Fix save option to account for data structure, and put all data in one file.
        if self.save:
            self.save_sensors()
            print('data saved')
        
        self.stop_printers()
        self.stop_readers()

    # Careful: not to be confused with startReaders()
    def run_sensors(self):
        self.start_readers()
        key = input('press any key to finish \n')
        print('Shutting down...')
        self.stop_readers()

    def save_sensors(self):
        datawriter = ReadWrite(self.directory, self.save_file)
        datawriter.makeDir(saveoption='addnew')
        for hand in self.hands:
            datawriter.saveSensorsFromDict(self.threads[hand]['parser'].getQ(), hand=hand)


class ReadSerial(Thread):
    def __init__(self, port, baud, qsize=48):
        Thread.__init__(self)
        self.daemon = True
        self.port = port
        self.baud = baud
        self.sensq = queue.Queue(maxsize=qsize)
        self.serial_port = serial.Serial(self.port, self.baud, timeout=None)

    def run(self):
        print('Port name: ' + self.serial_port.name)
        while True:
            self.read()

    def read(self):
        temp = self.serial_port.read_until()
        self.sensq.put(temp)
        temp = 0

    def getQ(self):
        return self.sensq

    def getQBlock(self):
        return self.sensq.get(block=True)


class ParseSerial(Thread):
    def __init__(self, sensq, time0, format='>sHHHHHHBBBBBBBBBB', 
                 length_checksum=23, add_timestamp=False, 
                 flex=False, press=True, imu=False, qsize=30):
        Thread.__init__(self)
        self.daemon = True
        self.sensq = sensq
        self.time0 = time0
        self.format = format
        self.length_checksum = length_checksum
        self.dataq = queue.Queue(maxsize=qsize)
        self.read_configs = {'flex': flex, 'press': press, 'imu': imu}
        self.add_timestamp = add_timestamp
        
        # Defining slices for various data types
        self.slices = {
            'flex': slice(7, 12),
            'press': slice(12, 17),
            'imu': slice(1, 7),
        }
        
    def run(self):
        print('parsing starting ... ')
        while True:
            self.parse()
            
    def parse(self):
        qread = self.sensq.get(block=True)
        s = qread.split(b'\n')[0]
        if (s[:1] in (b'r', b'l')) and len(s) == self.length_checksum:
            unp = unpack(self.format, s)
            data_to_put = {}
            
            for key, active in self.read_configs.items():
                if active:
                    data_to_put[key] = unp[self.slices[key]]
                    
            if self.add_timestamp: # to avoid putting empty data into the queue
                data_to_put['time'] = time.time() - self.time0
            
            self.dataq.put(data_to_put)
                
    def getQ(self):
        return self.dataq


class ReadKeyboard(Thread):
    def __init__(self, portname, time0):
        Thread.__init__(self)
        self.portname = portname
        self.dataq = queue.Queue(maxsize=0)
        self.daemon = True
        self.time0 = time0

    def run(self):
        inport = mido.open_input(self.portname)
        print('Piano Keyboard Port Name: ', self.portname)
        while True:
            msg = inport.receive(block=True)
            self.dataq.put([time.time() - self.time0, msg])

    def getQ(self):
        return self.dataq


class printSens(Thread):
    def __init__(self, sensorq, info=''):
        Thread.__init__(self)
        self.sensorq = sensorq
        self.info = info 
        self.daemon = True

    def run(self):
        while True: 
            sensvalue = self.sensorq.get(block=True) # No need to get, try only printing last element
            print(self.info, sensvalue)



# class ParseFile(Thread):
#     def __init__(self, directory, filename, flex=False, press=True, imu=False, qsize=30):
#         Thread.__init__(self)
#         self.directory = directory
#         self.filename = filename
#         self.daemon = True
#         self.dataq = queue.Queue(maxsize=qsize)
#         self.read_configs = {'flex': flex, 'press': press, 'imu': imu}
        
#         # Defining slices for various data types
#         self.slices = {
#             'flex': slice(7, 12),
#             'press': slice(12, 17),
#             'imu': slice(1, 7),
#         }
        
#     def run(self):
#         print('file parsing starting ... ')
#         while True:
#             self.parse()
            
#     def parse(self):
#         reader = ReadWrite(directory=self.directory, filename=self.filename)
#         # timeSens, pressData, flexData, imuData = reader.readSensors(hand=self.hand)
#         qread = self.sensq.get(block=True)
#         s = qread.split(b'\n')[0]
#         if (s[:1] in (b'r', b'l')) and len(s) == self.length_checksum:
#             unp = unpack(self.format, s)
#             data_to_put = {}
            
#             for key, active in self.read_configs.items():
#                 if active:
#                     data_to_put[key] = unp[self.slices[key]]
                    
#             if self.add_timestamp: # to avoid putting empty data into the queue
#                 data_to_put['time'] = time.time() - self.time0
            
#             self.dataq.put(data_to_put)
                
#     def getQ(self):
#         return self.dataq



class ParseFile(Thread):
    def __init__(self, directory=EXPDIR, filename='test', hand='R'):
        Thread.__init__(self)
        self.daemon = True
        self.directory = directory
        self.filename = filename
        self.hand = hand
        self.time0 = 0.0
        self.flexq = queue.Queue(maxsize=20)
        self.pressq = queue.Queue(maxsize=48)
        self.imuq = queue.Queue(maxsize=14)
        self.dataq = queue.Queue(maxsize=0)
        self.readFlex = False
        self.readPress = False
        self.readIMU = False
        self.readFullData = False

    def run(self):
        print('file parsing starting ... ')
        reader = ReadWrite(directory=self.directory, filename=self.filename)
        timeSens, pressData, flexData, imuData = reader.readSensors(hand=self.hand)
        time0=time.time()

        for idx, next_time in enumerate(timeSens):
            while time.time()-time0 < next_time: # Looks terribly inefficient
                pass

            if self.readFlex:
                self.flexq.put(flexData[idx])
            if self.readPress:
                self.pressq.put(pressData[idx])
            if self.readIMU:
                self.imuq.put(imuData[idx])
            if self.readFullData:
                self.dataq.put( (time.time() - self.time0,) + imuData + flexData + pressData )

        print('done')

    def getQ(self):
        return self.dataq


if __name__ == "__main__":
    save = True 
    sensor_config = {'l': {'flex': True, 'press': True, 'imu': True},
                     'r': {'flex': False, 'press': True, 'imu': True}}
    reader = Reader(sensor_config=sensor_config, 
                    save=save,
                    save_file='test00')
    
    reader.runSensors()
    reader.saveSensors()