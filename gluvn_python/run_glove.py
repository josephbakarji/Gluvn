from __future__ import division
from __init__ import portL, portR, baud, testDir, simDir, EXPDIR, learnDir, keyboard_portname
from port_read import ReadSerial, ParseSerial, printSens, ReadKeyboard, ParseFile
from data_analysis import ReadWrite, Analyze, Analyze2Hands
from senstonote import WeighTrig_ai, WeighTrig
import senstonote
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
from collections import deque
import csv, sys, os

class RunGlove:
    def __init__(self, directory=testDir, filename='test0'):
        self.directory = directory
        self.filename = filename
        self.time0 = time.time()

    def runKeyboard(self):
        self.keyboardThread = ReadKeyboard(keyboard_portname, self.time0)
        self.keyboardThread.start()

    def plotRun(self):
        dataplotting = Analyze(self.directory, self.filename)
        dataplotting.plotKeyAndGlove('basic')
        dataplotting.plot_sampling_frequency()

    def saveRun(self, sensThread, keyboardThread):
        datawriter = ReadWrite(self.directory, self.filename)
        datawriter.saveData(sensThread.getQ(), key_dataq = keyboardThread.getQ())
        self.directory, self.filename = datawriter.getSaveLocation()

    def setSaveLocation(self, directory, filename):
        self.directory = directory
        self.filename = filename

    def getSaveLocation(self):
        return self.directory, self.filename

    def recordKeyboardAndGlove(self, port=portR):
        sensThread = ReadSerial(portR, baud)
        parseThread = ParseSerial(sensThread.sensq, self.time0)
        parseThread.readFullData = True

        self.runKeyboard()
        sensThread.start()
        parseThread.start()

        key = input('press any key to finish \n')
        print('End of run')

        self.saveRun(parseThread, self.keyboardThread)
        self.plotRun()


    def printAllSens(self, hands='right', printit=True, save=False, plot=False, fromfile=False): # can function be split in 2? (right-left hands)
        righthand = False 
        lefthand = False
        if hands == 'right':
            righthand = True
        elif hands == 'left':
            lefthand = True 
        elif hands == 'both':
            righthand = True
            lefthand = True 
        else:
            print('Incorrect Option: set variable "hands" to right, left or both')

        if righthand:
            sensThreadR = ReadSerial(portR, baud)
            parseThreadR = ParseSerial(sensThreadR.sensq, self.time0)
            parseThreadR.readFullData = True
            sensThreadR.start()
            parseThreadR.start()
            
            if printit:
                printThreadR = printSens(parseThreadR.dataq, info='R') 
                printThreadR.start()

        if lefthand:
            sensThreadL = ReadSerial(portL, baud)
            parseThreadL = ParseSerial(sensThreadL.sensq, self.time0)
            parseThreadL.readFullData = True
            sensThreadL.start()
            parseThreadL.start()
            
            if printit:
                printThreadL = printSens(parseThreadL.dataq, info='L') 
                printThreadL.start()

        key = input('press any key to finish \n')
        print('End of run')
       
       # Better to separate into other functions?
        if save:
            if not printit:
                datawriter = ReadWrite(self.directory, self.filename)
                datawriter.makeDir(saveoption='addnew')
                if lefthand:
                    datawriter.saveData(parseThreadL.dataq, hand='L', saveoption='overwrite') #redundant saveoption
                if righthand:
                    datawriter.saveData(parseThreadR.dataq, hand='R', saveoption='overwrite')
                new_filename = datawriter.get_filename()
            else:
                print('Not Saved! You can either print or save - Fix sensq.get(block) in printSens to do both simultaneously')
        
        if plot:
            plotter = Analyze2Hands(self.directory, new_filename)
            plotter.plotSensors(savename='all_sensors')

            
#####################################
#####################################

    def simpleTrigger(self, notes, pressTrigThresh):
        sensThread = ReadSerial(portR, baud)
        parseThread = ParseSerial(sensThread.sensq, self.time0)
        parseThread.readPress = True
        PressTrigP = WeighTrig(parseThread.pressq, pressTrigThresh, notes) 

        sensThread.start()
        parseThread.start()
        PressTrigP.start()

        key = input('press any key to finish \n')
        print('End of run')



    def aiTrigger(self):
        sensThread = ReadSerial(portR, baud)
        parseThread = ParseSerial(sensThread.sensq, self.time0)
        parseThread.readPress = True
        parseThread.readFlex = True
        pre_flexsize = 12 
        pressTrigThresh = 15 
        dshmidt = 5
        PressTrigP = WeighTrig_ai(parseThread.pressq, parseThread.flexq, pre_flexsize, pressTrigThresh, dshmidt) 

        sensThread.start()
        parseThread.start()
        PressTrigP.start()


        key = input('press any key to finish \n')
        print('End of run')



    def twoHandInstrument(self, fromfile=False):
        # Uses Pressure in right and flex in left for full keyboard access.
        
        if fromfile:
            parseThreadR = ParseFile(directory=self.directory, filename=self.filename, hand='R')
            parseThreadL = ParseFile(directory=self.directory, filename=self.filename, hand='L')
            parseThreadR.readPress = True
            parseThreadL.readFlex = True
            parseThreadR.start()
            parseThreadL.start()

        else:
            sensThreadR = ReadSerial(portR, baud)
            sensThreadL = ReadSerial(portL, baud)
            parseThreadR = ParseSerial(sensThreadR.sensq, self.time0)
            parseThreadL = ParseSerial(sensThreadL.sensq, self.time0)
            parseThreadR.readPress = True
            parseThreadL.readFlex = True

            sensThreadR.start()
            parseThreadR.start()
            sensThreadL.start()
            parseThreadL.start()

        # Fix that part
        collectq = queue.Queue(maxsize=20)
        pressTrigThresh = 15 
        dshmidt = 5
        basenote = 'C3'
        key = 'C'
        

        pressTrigPR = senstonote.WeighTrig2h(parseThreadR.pressq, collectq, 'R', pressTrigThresh, dshmidt) 
        #pressTrigPL = senstonote.WeighTrig2h(sensThreadL.pressq, collectq, 'L', 40, 10)  
        pressTrigPL = senstonote.WeighTrig2h(parseThreadL.flexq, collectq, 'L', 100, 20)

        collectSend = senstonote.CombHandsSendMidi(collectq, basenote, key)

        pressTrigPR.start()
        pressTrigPL.start()
        collectSend.start()

        key = input('press any key to finish \n')
        print('End of run')


