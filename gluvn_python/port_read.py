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


class ReadSerial(Thread):
    def __init__(self, port, baud):
        Thread.__init__(self)
        self.daemon = True
        self.port = port
        self.baud = baud
        self.sensq = queue.Queue(maxsize=48)
        self.serial_port = serial.Serial(self.port, self.baud, timeout=None)

    def run(self):
        print('Port name: ' + self.serial_port.name)
        while True:
            self.read()

    def read(self):
        temp = self.serial_port.read_until()
        self.sensq.put(temp)
        temp = 0

    def getSensorQ(self):
        return self.sensq.get(block=True)

class ParseSerial(Thread):
    def __init__(self, sensq, time0):
        Thread.__init__(self)
        self.daemon = True
        self.sensq = sensq
        self.time0 = time0
        self.flexq = queue.Queue(maxsize=20)
        self.pressq = queue.Queue(maxsize=48)
        self.imuq = queue.Queue(maxsize=14)
        self.dataq = queue.Queue(maxsize=0)
        self.readFlex = False
        self.readPress = False
        self.readIMU = False
        self.readFullData = False

    def run(self):
        print('parsing starting ... ')
        while True:
            self.parse()

    def parse(self):
        qread = self.sensq.get(block=True)
        s = qread.split(b'\n')[0]
        if s[:1] == b'w' and len(s) == 23:
            unp = unpack('>sHHHHHHBBBBBBBBBB', s)
            if self.readFlex:
                self.flexq.put(unp[7:12])
            if self.readPress:
                self.pressq.put(unp[12:17])
            if self.readIMU:
                self.imuq.put(unp[1:7])
            if self.readFullData:
                self.dataq.put( (time.time() - self.time0,) + unp )

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
        print('time, yaw, pitch, roll, gx, gy, gz, f1, f2, f3, f4, f5, p1, p2, p3, p4, p5 \n')
        while True: 
            sensvalue = self.sensorq.get(block=True) # No need to get, try only printing last element
            print(self.info, sensvalue)


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
        sys.exit() #doenest work...

    def getQ(self):
        return self.dataq

