from __init__ import keyboard_portname, EXPDIR, testDir, settingsDir, figDir, portL, portR, baud
from run_glove import RunGlove
from data_analysis import Analyze, Stats, Analyze2Hands
from learning import Learn
import numpy as np
import matplotlib.pyplot as plt
import sys
import copy


class play2hands:
    def __init__(self, directory=testDir, filename='test0'):
        self.directory = directory 
        self.filename = filename
        self.run = RunGlove(directory=self.directory, filename=self.filename)
        self.printit = False
        self.save = True
        self.plotit = True
        
    def plot(self, save2file='all_sensors', opt='default'):
        plotter = Analyze2Hands(directory=self.directory, filename=self.filename)
        plotter.plotSensors(savename=save2file, option=opt)

    def record(self):
        self.run.printAllSens(hands='both', printit=self.printit, save=self.save, plot=self.plotit)

    def simulate(self):
        self.run.twoHandInstrument(fromfile=True)

    def play(self):
        self.run.twoHandInstrument(fromfile=False)

if __name__ == '__main__':
    filename = 'imu_test_2'
    plot_opt = 'imu'
    p = play2hands(directory=EXPDIR, filename=filename)
    p.plot(save2file=filename, opt=plot_opt)
    #p.simulate()
    #p.record()



