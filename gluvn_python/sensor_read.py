'''
Created on Mar 20, 2016

@author: joseph bakarji
'''

from __future__ import division
import serial, time
import numpy as np
import random
from threading import Thread
import binascii
from struct import *
from collections import namedtuple
import matplotlib.pyplot as plt
import math
import queue
from collections import deque
import csv
#import pyaudio
#import midiout

Data = namedtuple('Data', 'cs ax ay az gx gy gz f1 f2 f3 f4 f5 p1 p2 p3 p4 p5')
n1 = Data( 'a', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


# use threading.enumerate() for to see the threads and debug the lag between the two stacks
# Fixes needed: synchronize threads with locks.


class read_port(Thread):
    def __init__(self, port, baud, sensq):
        Thread.__init__(self)
        self.port = port
        self.baud = baud
        self.daemon = True
        self.sensq = sensq
 
    def run(self):
        serial_port = serial.Serial(self.port, self.baud, timeout = 0)
        print('Port name: ' + serial_port.name)
        while (True):
            read_sens(serial_port,self.sensq)

class parse_serial(Thread):
    def __init__(self, sensq, dataq, imuq, flexq, pressq):
        Thread.__init__(self)
        self.daemon = True
        self.sensq = sensq
        self.dataq = dataq
        self.imuq = imuq
        self.flexq = flexq
        self.pressq = pressq
        
    def run(self):
        print('parsing starting ... ')
        while True:
            parse(self.sensq, self.dataq, self.imuq, self.flexq, self.pressq)


# functions

def read_sens(ser,sensq):
    bytesToRead = ser.inWaiting()
    if(bytesToRead != 0):
        temp = ser.read(bytesToRead)
        sensq.put(temp)    # Global queue        
        temp = 0


def parse(sensq, dataq, imuq, flexq, pressq):
    if(sensq.empty() == False):
        qread = sensq.get()
        chunk = qread.split(b'\n')
        for s in chunk:
            if(s[:1] == b'w' and len(s) == 23):
                unp = unpack('>shhhhhhBBBBBBBBBB', s)
                mydata = n1._make(unp)
                dataq.put(mydata)
                imuq.append(mydata[1:7])
                flexq.append(mydata[7:12])
                pressq.append(mydata[12:17])


##########################  python GLOVE EMULATOR ############################
class sim_glove(Thread):
    def __init__(self, filename, imuq, flexq, pressq):
        Thread.__init__(self)
        self.daemon = True
        self.imuq = imuq
        self.flexq = flexq
        self.pressq = pressq
        self.filename = filename
    
    def run(self):
        
        # Read file
        with open(self.filename, 'r') as f:
            reader = csv.reader(f)
            read_list = list(reader)
        
        # change string to numbers
        header = read_list.pop(0)
        for readvec in read_list:
            readvec[0] = float(readvec[0])
            for i in range(1,len(readvec)):
                readvec[i] = int(readvec[i])        
        tlist = [vec[0] for vec in read_list]
        
        # append data to queues according to time stamp
        t0 = time.time()
        for i in range(len(tlist)):
            while(time.time() - t0 < tlist[i]): # wastes time waiting? need locks
                pass
            self.imuq.append(read_list[i][1:7])
            self.flexq.append(read_list[i][7:12])
            self.pressq.append(read_list[i][12:17])

# Write data to file                
def write2file(filename, q):
    t0 = time.time()
    file = open(filename, 'w')
    file.write('time,yaw,pitch,roll,gx,gy,gz,f1,f2,f3,f4,f5,p1,p2,p3,p4,p5' + '\n')
    while True:
        if(q.empty() == False):
            datalist = list(q.get())
            file.write(str(time.time() - t0) + ', ' + str(datalist[1:])[1:-1] + '\n')

################################################################################
################################################################################


if __name__ == '__main__':
    

    # Variables
    port = '/dev/tty.usbmodem1411'
    baud = 57600
    
    
    sensq = queue.Queue()
    dataq = queue.Queue()
    pressq = deque(maxlen = 10)
    flexq = deque(maxlen = 10)
    imuq = deque(maxlen = 10)
    
    
    # Start reading threads
    ParseThread = parse_serial(sensq, dataq, imuq, flexq, pressq)
    SensorThread = read_port(port, baud, sensq)
    ParseThread.start()
    SensorThread.start()


    write2file('data/sim/rnd_glove_ypr_press.csv', dataq)

            
            
            