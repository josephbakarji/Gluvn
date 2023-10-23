import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import itertools
import csv, sys, os
from __init__ import learnDir, figDir, EXPDIR
from utils import merge_queues
import numpy as np
import copy
import pdb

class ReadWrite:
    def __init__(self, directory, filename):
        self.directory = directory
        self.filename = filename
        self.direct = self.getFullPath()


    def get_filename(self):
        return self.filename

    def getFullPath(self):
        return self.directory + self.filename+'/'

    def saveData(self, sens_dataq, hand='R', key_dataq=None, saveoption='addnew'): # Added optional key_dataq without ensuring consistency
        self.makeDir(saveoption=saveoption)
        self.saveSensors(sens_dataq, hand=hand)
        if key_dataq != None:
            self.saveKeyboard(key_dataq)

    def makeDir(self, saveoption='addnew'):
        if (saveoption == 'addnew'):
            directory_list = next(os.walk(self.directory))
            i = 0
            fm = self.filename + '_' + str(i)
            while os.path.isdir(self.directory+fm):
                i += 1
                fm = self.filename + '_' + str(i)
            self.filename = fm

            self.direct = self.getFullPath()
            print('Creating Directory: '+ self.direct)
            os.mkdir(self.direct)

        elif(saveoption == 'manual'):
            if(os.path.isdir(self.direct)):
                prompt = input("Overwriting " + self.direct + "  \nPress Enter to Proceed \nPress n to exit \nOr write new filename: ")
                if prompt == 'n':
                    os.exit()
                elif len(prompt) != 0:
                    self.filename = prompt
                    self.direct = self.getFullPath()
                    os.mkdir(self.direct)
                else:
                    pass
            else:
                print('Writing to '+ self.direct)
                os.mkdir(self.direct)

        # elif(saveoption == 'overwrite'):
        #     print("Overwriting or appending " + self.direct) # Overwrite or append..
        else:
            print('Invalid Saving Option')

#################################
#################################

    # Not used for new data-structure
    def saveSensors(self, dataq, hand=''):
        ff = open(self.direct + self.filename + '_sensors' + hand + '.csv', 'w')
        print(self.filename)
        ff.write('time, yaw, pitch, roll, gx, gy, gz, f1, f2, f3, f4, f5, p1, p2, p3, p4, p5' + '\n')
        while(not dataq.empty()):
            datalist = list(dataq.get(block=True))
            ff.write('%.6f' % datalist[0] + ', ' + str(datalist[2:])[1:-1] + '\n')
        ff.close()


    def saveSensorsFromDict(self, dataq, hand):
        ff = open(self.direct + self.filename + '_sensors_'+hand+'.csv', 'w')
        print('saving to: ', self.filename)
        imu_labels = ['yaw', 'pitch', 'roll', 'gx', 'gy', 'gz']
        flex_labels = ['f1', 'f2', 'f3', 'f4', 'f5']
        press_labels = ['p1', 'p2', 'p3', 'p4', 'p5']

        sorted_labels = ['timestamp', 'imu', 'flex', 'press']
        included_sensors = [label for label in sorted_labels if label in list(dataq.get(block=True).keys())] 
        labels = []
        labels += ['time'] if 'timestamp' in included_sensors else []
        labels += imu_labels if 'imu' in included_sensors else []
        labels += flex_labels if 'flex' in included_sensors else []
        labels += press_labels if 'press' in included_sensors else []

        ff.write(', '.join(labels) + '\n')
        while(not dataq.empty()):
            elem = dataq.get(block=True)
            sensors_data = []
            for sensor in included_sensors:
                sensors_data += elem[sensor]
            sensor_string = ', '.join(['{:.5f}'.format(i) if type(i) == float else str(i) for i in sensors_data])
            ff.write(sensor_string + '\n')
        ff.close()
        


    def saveKeyboard(self, dataq):
        ff = open( self.direct + self.filename + '_keyboard.csv', 'w')
        ff.write('time, note, velocity' + '\n')
        while(not dataq.empty()):
            datalist = dataq.get(block=True)
            ff.write('%.6f' % datalist[0] + ', ' + str(datalist[1].note) + ', ' + str(datalist[1].velocity)+ '\n')
        ff.close()


    def readData(self): # Change name: e.g. readKeyAndRH
        if(not os.path.isdir(self.direct)):
            print("Directory "+ self.direct +"does not exist - Existing Program!")
            sys.exit()

        timeSens, pressData, flexData, imuData = self.readSensors()
        timeKeyboard, notes, velocity = self.readKeyboard()
        return timeSens, pressData, flexData, imuData, timeKeyboard, notes, velocity

    def read2hands(self):
        if(not os.path.isdir(self.direct)):
            print("Directory "+ self.direct +" does not exist - Existing Program!")
            sys.exit()

        timeSensR, pressDataR, flexDataR, imuDataR = self.readSensors(hand='R')
        timeSensL, pressDataL, flexDataL, imuDataL = self.readSensors(hand='L')
        return timeSensR, pressDataR, flexDataR, imuDataR, timeSensL, pressDataL, flexDataL, imuDataL


    def readSensors(self, hand='', Filter=True):
        file2read = self.direct + self.filename + '_sensors'+ hand +'.csv'
        with open(file2read, 'r') as (g):
            reader = csv.reader(g)
            read_list = list(reader)
            header = read_list.pop(0)
            for readvec in read_list:
                readvec[0] = float(readvec[0])
                for i in range(1, len(readvec)):
                    readvec[i] = int(readvec[i])

        timeSens = [vec[0] for vec in read_list]
        imuData = [vec[1:7] for vec in read_list]
        flexData = [vec[7:12] for vec in read_list]
        pressData = [vec[12:17] for vec in read_list]

        if Filter: 
            return timeSens, self.naiveFilter(pressData), self.naiveFilter(flexData), imuData # Filter as an option
        else:
            return timeSens, pressData, flexData, imuData  


    def readKeyboard(self):
        file2read = self.direct + self.filename + '_keyboard.csv'
        if(not os.path.isfile(file2read)):
            print('No keyboard file')
            return []
        with open(file2read, 'r') as (g):
            reader = csv.reader(g)
            read_list = list(reader)
            header = read_list.pop(0)
            for readvec in read_list:
                readvec[0] = float(readvec[0])
                for i in range(1, len(readvec)):
                    readvec[i] = int(readvec[i])

        timeKeyboard = [vec[0] for vec in read_list]
        notes = [vec[1] for vec in read_list]
        velocity = [vec[2] for vec in read_list]
        return timeKeyboard, notes, velocity

    def getSaveLocation(self):
        return self.directory, self.filename

    def naiveFilter(self, pressData):
        # Solve the spiking problem of sensors sometimes jumping to 255
        for i in range(len(pressData)):
            for j in range(len(pressData[i])):
                if abs(pressData[i][j] - pressData[i-1][j]) > 200:
                    pressData[i][j] = pressData[i-1][j]
        return pressData




class Analyze2Hands(ReadWrite):
    def __init__(self, directory=learnDir, filename='test0'):
        self.directory = directory
        self.filename = filename
        super().__init__(self.directory, self.filename)
        self.direct = super().getFullPath()

        self.timeSensR = []
        self.pressDataR = []
        self.flexDataR = []
        self.imuDataR = []
        self.timeSensL = []
        self.pressDataL = []
        self.flexDataL = []
        self.imuDataL = []

    def readAllSens(self):
        print("Reading from " + self.direct)
        if(not os.path.isdir(self.direct)):
            raise Exception("file does not exist")

        self.timeSensR, self.pressDataR, self.flexDataR, self.imuDataR = super().readSensors(hand='R')
        self.timeSensL, self.pressDataL, self.flexDataL, self.imuDataL = super().readSensors(hand='L')

    def invertList(self, data):
        return list(map(list, zip(*data)))

    def getPressData(self, hand='R'):
        if hand == 'R': 
            pdata = self.pressDataR
        elif hand == 'L':
            pdata = self.pressDataL
        else:
            raise exception('incorrect hand')

        if(pdata == []):
            print('Pressure sensor data is empty')
            return []
        return [[p[i] for p in pdata] for i in range(len(pdata[0]))]

    def getFlexData(self, hand='R'):
        if hand == 'R': 
            fdata = self.flexDataR
        elif hand == 'L':
            fdata = self.flexDataL
        else:
            raise Exception('Incorrect hand')

        if(fdata == []):
            print('Pressure sensor data is empty')
            return []
        return [[f[i] for f in fdata] for i in range(len(fdata[0]))]

    def getImuData(self, hand='R'):
        if hand == 'R': 
            imudata = self.imuDataR
        elif hand == 'L':
            imudata = self.imuDataL
        else:
            raise Exception('Incorrect hand')

        if(imudata == []):
            print('Pressure sensor data is empty')
            return []

        return [[f[i] for f in imudata] for i in range(len(imudata[0]))]


    def addSens2Plot(self, ax, time, sensdata, pick, linestyle='-', color='k', linewidth=1):
        for i in range(len(pick)):
            if(pick[i]):
                ax.plot(time, sensdata[i], linestyle = linestyle, linewidth = 2)


    def addSlider2Plot(self, ax, plt):
        # https://stackoverflow.com/questions/31001713/plotting-the-data-with-scrollable-x-time-horizontal-axis-on-linux
        axcolor = 'lightgoldenrodyellow'
        axpos = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor=axcolor) 
        spos = Slider(axpos, 'Pos', self.timeSens[0], self.timeSens[-1])
        def update(val): 
            pos = spos.val 
            ax.axis([pos, pos+5, 0, 255]) 
            fig.canvas.draw_idle() 
        spos.on_changed(update) 

    def plotSensors(self, savedir=figDir, savename='', option='default'):
            
        # Read sensors
        self.readAllSens()
        fsR = self.getFlexData(hand='R')
        psR = self.getPressData(hand='R')
        imusR = self.getImuData(hand='R')
        timeL = self.timeSensL

        fsL = self.getFlexData(hand='L')
        psL = self.getPressData(hand='L')
        imusL = self.getImuData(hand='L')
        timeR = self.timeSensR

        # Plotting
        fig = plt.figure()
        ax1 = plt.subplot(221)
        ax2 = plt.subplot(222)
        ax3 = plt.subplot(223)
        ax4 = plt.subplot(224)
        ax = [ax1, ax2, ax3, ax4]
        
        if option == 'default':

            ylim = [255, 65500, 255, 65500]
            dt = 5
            plt.subplots_adjust(bottom=0.25) 

            self.addSens2Plot(ax1, timeR, psR, [1, 1, 1, 1, 1], '-')
            self.addSens2Plot(ax2, timeR, imusR, [0, 0, 0, 1, 1, 1], '-')
            self.addSens2Plot(ax3, timeL, fsL, [1, 1, 1, 1, 1], '-')
            self.addSens2Plot(ax4, timeL, imusL, [1, 1, 1, 0, 0, 0], '-')

            for idx, a in enumerate(ax):
                a.axis([0, dt, 0, ylim[idx]])

            ax1.legend(['p1', 'p2', 'p3', 'p4', 'p5'], fontsize=13)
            ax2.legend(['Gyro x', 'Gyro y', 'Gyro z'], fontsize=13)
            ax3.legend(['f1', 'f2', 'f3', 'f4', 'f5'], fontsize=13)
            ax4.legend(['pitch', 'roll', 'yaw'], fontsize=13)

            ax1.set_title('Pressure Right Hand')
            ax2.set_title('Gyro Right Hand')
            ax3.set_title('Flex Left Hand')
            ax4.set_title('Accelerometer Left Hand')

        elif option == 'imu':

            ylim = [65500, 65500, 65500, 65500]
            dt = 5
            plt.subplots_adjust(bottom=0.25) 

            self.addSens2Plot(ax1, timeR, imusR, [1, 1, 1, 0, 0, 0], '-')
            self.addSens2Plot(ax2, timeR, imusR, [0, 0, 0, 1, 1, 1], '-')
            self.addSens2Plot(ax3, timeL, imusL, [1, 1, 1, 0, 0, 0], '-')
            self.addSens2Plot(ax4, timeL, imusL, [0, 0, 0, 1, 1, 1], '-')

            for idx, a in enumerate(ax):
                a.axis([0, dt, 0, ylim[idx]])

            ax1.legend(['Gyro x', 'Gyro y', 'Gyro z'], fontsize=13)
            ax2.legend(['pitch', 'roll', 'yaw'], fontsize=13)
            ax3.legend(['Gyro x', 'Gyro y', 'Gyro z'], fontsize=13)
            ax4.legend(['pitch', 'roll', 'yaw'], fontsize=13)

            ax1.set_title('Gyro Right Hand')
            ax2.set_title('Accel Right Hand')
            ax3.set_title('Gyro Left Hand')
            ax4.set_title('Accel Left Hand')

        elif option == 'imumag':

            ylim = [65500, 65500, 65500, 65500]
            dt = 5
            plt.subplots_adjust(bottom=0.25) 

            self.addSens2Plot(ax1, timeR, imusR, [1, 1, 1, 0, 0, 0], '-')
            self.addSens2Plot(ax2, timeR, imusR, [0, 0, 0, 1, 1, 1], '-')
            self.addSens2Plot(ax3, timeL, imusL, [1, 1, 1, 0, 0, 0], '-')
            self.addSens2Plot(ax4, timeL, imusL, [0, 0, 0, 1, 1, 1], '-')

            for idx, a in enumerate(ax):
                a.axis([0, dt, 0, ylim[idx]])

            ax1.legend(['Gyro x', 'Gyro y', 'Gyro z'], fontsize=13)
            ax2.legend(['pitch', 'roll', 'yaw'], fontsize=13)
            ax3.legend(['Gyro x', 'Gyro y', 'Gyro z'], fontsize=13)
            ax4.legend(['pitch', 'roll', 'yaw'], fontsize=13)

            ax1.set_title('Gyro Right Hand')
            ax2.set_title('Accel Right Hand')
            ax3.set_title('Gyro Left Hand')
            ax4.set_title('Accel Left Hand')

        else:
            raise Exception('Wrong Plotting Option')

        axcolor = 'lightgoldenrodyellow'
        axpos = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor=axcolor) 
        spos = Slider(axpos, 'Time slider', self.timeSensR[0], self.timeSensR[-1])
        def update(val): 
            pos = spos.val 
            for idx, a in enumerate(ax):
                a.axis([pos, pos+dt, 0, ylim[idx]]) 
            fig.canvas.draw_idle() 
        spos.on_changed(update) 

        # Show plot and save it
        plt.show()
        if savename == '':
            prompt = input("Press Enter to write to default.pdf, or write name of file: ")
            if(len(prompt)>0):
                fig.savefig(savedir + prompt +'.pdf')
            else:
                fig.savefig(savedir + 'default.pdf')
        else:
            fig.savefig(savedir + savename + '.pdf')
    

    def plot_sampling_frequency(self):
        # To get sampling frequency of sensor data readings

        fig, ax = plt.subplots()
        plt.subplots_adjust(bottom=0.25) 
        self.readFullData()
        difftime = [self.timeSens[i+1] - self.timeSens[i] for i in range(len(self.timeSens) - 1)]
        avgdifftime = sum(difftime)/len(difftime)
        ax.plot(self.timeSens[:-1], difftime, '-', linewidth = 2)
        plt.xlabel('Time')
        plt.ylabel('Sampling dt (seconds)')
        plt.title('Sampling Period with avg = ' + str(avgdifftime))
        plt.show()


class Analyze(ReadWrite):
    def __init__(self, directory=learnDir, filename='test0'):
        self.directory = directory
        self.filename = filename
        super().__init__(self.directory, self.filename)
        self.direct = super().getFullPath()
        self.timeSens = []
        self.pressData = []
        self.flexData = []
        self.imuData = []
        self.timeKeyboard = []
        self.notes = []
        self.velocity = []

    def readFullData(self): # readKeyAndRH
        print("Reading from " + self.direct)
        if(not os.path.isdir(self.direct)):
            raise Exception("file does not exist")

        self.timeSens, self.pressData, self.flexData, self.imuData, self.timeKeyboard, self.notes, self.velocity = super().readData()

    def invertList(self, data):
        return list(map(list, zip(*data)))


    def getPressData(self):
        if(self.pressData == []):
            print('Pressure sensor data is empty')
            return []
        return [[p[i] for p in self.pressData] for i in range(len(self.pressData[0]))]

    def getFlexData(self):
        if(self.flexData == []):
            print('Flex sensor data is empty')
            return []
        return [[f[i] for f in self.flexData] for i in range(len(self.flexData[0]))]

    def getImuData(self):
        if(self.imuData == []):
            print('Flex sensor data is empty')
            return []
        return [[f[i] for f in self.imuData] for i in range(len(self.imuData[0]))]

    def getKeyTimeData(self):
        if(self.timeKeyboard == []):
            print('Keyboard time data is empty')
            return []
        return self.timeKeyboard 

    def getNotes(self, timeKeyboard, notes, velocity):
        # transforms notes from (time, note, velocity), to (note, velocity, start_time, end_time)
        if(notes == []):
            print('There are no notes')
            return []
        TimedNotes = []
        for Sidx, note in enumerate(notes):
            if(velocity[Sidx] != 0):
                start_time = timeKeyboard[Sidx]
                for Eidx in range(Sidx+1,len(notes)):
                    if(velocity[Eidx] == 0 and notes[Eidx] == note):
                        end_time = timeKeyboard[Eidx]
                        break
                TimedNotes.append( (note, velocity[Sidx], start_time, end_time) )

        return TimedNotes


    def addNotes2Plot(self, ax, TimedNotes, opacity):
        for tn in TimedNotes:
            note, velocity, start_time, end_time = tn
            ax.fill_between([start_time, end_time], [velocity*2 for _ in range(2)], alpha = opacity )

    def addSens2Plot(self, ax, sensdata, pick, linestyle='-', color='k', linewidth=1):
        for i in range(len(pick)):
            if(pick[i]):
                ax.plot(self.timeSens, sensdata[i], linestyle = linestyle, linewidth = 3)


    def addSlider2Plot(self, ax, plt):
        # https://stackoverflow.com/questions/31001713/plotting-the-data-with-scrollable-x-time-horizontal-axis-on-linux
        axcolor = 'lightgoldenrodyellow'
        axpos = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor=axcolor) 
        spos = Slider(axpos, 'Pos', self.timeSens[0], self.timeSens[-1])
        def update(val): 
            pos = spos.val 
            ax.axis([pos, pos+5, 0, 255]) 
            fig.canvas.draw_idle() 
        spos.on_changed(update) 

    def plotKeyAndGlove(self, mode, savedir=figDir, savename=''):
        if mode == 'basic':

            fig, ax = plt.subplots()
            plt.subplots_adjust(bottom=0.25) 

            self.readFullData()
            fs = self.getFlexData()
            ps = self.getPressData()
            tn = self.getNotes(self.timeKeyboard, self.notes, self.velocity)

            self.addNotes2Plot(ax, tn, 0.3)
            self.addSens2Plot(ax, ps, [1, 1, 1, 1, 1], '-')
            self.addSens2Plot(ax, fs, [1, 0, 0, 0, 0], '--')

            plt.axis([10,15, 0, 255])
            ax.legend(['Thumb', 'Index', 'Major', 'Ring', 'Pinky', 'Thumb Flex'], fontsize=13)
            plt.xlabel('Time (seconds)', fontsize=18)
            plt.ylabel('Sensor Readings', fontsize=18)
            plt.title('Keyboard and Sensors')

            axcolor = 'lightgoldenrodyellow'
            axpos = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor=axcolor) 
            spos = Slider(axpos, 'Time slider', self.timeSens[0], self.timeSens[-1])
            def update(val): 
                pos = spos.val 
                ax.axis([pos, pos+5, 0, 255]) 
                fig.canvas.draw_idle() 
            spos.on_changed(update) 
            
            plt.show()
            if savename == '':
                prompt = input("Press Enter to write to default.pdf, or write name of file: ")
                if(len(prompt)>0):
                    fig.savefig(savedir + prompt +'.pdf')
                else:
                    fig.savefig(savedir + 'default.pdf')
            else:
                fig.savefig(savedir + savename + '.pdf')

        else:
            raise Exception('mode does not exist')
            

    def plot_sampling_frequency(self):
        # To get sampling frequency of sensor data readings

        fig, ax = plt.subplots()
        plt.subplots_adjust(bottom=0.25) 

        self.readFullData()
        difftime = [self.timeSens[i+1] - self.timeSens[i] for i in range(len(self.timeSens) - 1)]
        avgdifftime = sum(difftime)/len(difftime)
        ax.plot(self.timeSens[:-1], difftime, '-', linewidth = 2)
        plt.xlabel('Time')
        plt.ylabel('Sampling dt (seconds)')
        plt.title('Sampling Period with avg = ' + str(avgdifftime))
        plt.show()






class Stats:
	def __init__(self, includefile='', directory=learnDir):
		self.directory = directory
		self.includefile = includefile
		self.filelist = []



	def get_features_wtu(self, idxminus=10, idxplus=0):
		# Returns features (Fdiff, Ndiff, thumb_flex), taking into account thumb flexing
		# Merge with get_thumb_flex (very similar)
		####
		trigevent = self.collect_trigger_events(idxminus =idxminus, idxplus=idxplus)

		# Classified with flex_events[0] as not thumb under, and flex_events[1] as thumb under
		flex_events = {0: {0: [], 1: [], 2: [], 3: [], 4: []}, 1: {0: [], 1: [], 2: [], 3: [], 4: []}}
		flex_times ={0: {0: [], 1: [], 2: [], 3: [], 4: []}, 1: {0: [], 1: [], 2: [], 3: [], 4: []}}
		Fpair_full = []
		Ndiff_full = []
		flex_array_full = []

		# C-scale aware: transform midi notes to indexed C-scale [60, 62, 64, 65...] -> [20, 21, 22, 23,...]
		Cnotes = []
		for key, val in csv.reader(open('data/tables/note2num.csv')):
			if(len(key)==2):
				Cnotes.append(val)
		Cnotes_val2idx = dict()
		for idx, val in enumerate(sorted(Cnotes)):
			Cnotes_val2idx[int(val)] = idx 


		# Separate trigger events per finger
		for fileidx in range(len(trigevent['filename'])):
			Fpair = []
			Ndiff = []
			flex_array = []

			trigonevent = copy.deepcopy(trigevent['on'][fileidx])
			prev_event = trigonevent[0]
			for idx, event in enumerate(trigonevent[1:]):
				flex = event['flex'][0, :] # Only take flex data of thumb! Other fingers might be useful
				time = event['senstime'] - event['trigtime'] 

				# Checks if note and finger are incremented together (cross-over) as labels (perfect labeling not garanteed)
				noteC = Cnotes_val2idx[event['note']] if event['note'] in Cnotes_val2idx.keys() else Cnotes_val2idx[event['note']+1]
				prev_noteC = Cnotes_val2idx[prev_event['note']] if prev_event['note'] in Cnotes_val2idx.keys() else Cnotes_val2idx[prev_event['note']+1]
				Note_diff = noteC - prev_noteC
				Finger_pair = (prev_event['pressidx'], event['pressidx'])
				parallel_inc = Note_diff * (Finger_pair[1] - Finger_pair[0])

				# Binary label flex by storing in 0 and 1 arrays
				Fidx = event['pressidx']
				if(parallel_inc >= 0):
					flex_events[0][Fidx].append(flex)
					flex_times[0][Fidx].append(time)
				else:
					flex_events[1][Fidx].append(flex)
					flex_times[1][Fidx].append(time)

				Ndiff.append(Note_diff)
				Fpair.append(Finger_pair)
				flex_array.append(flex)
				prev_event = event

			Fpair_full.append( Fpair )
			Ndiff_full.append( Ndiff )
			flex_array_full.append( flex_array )


		# Make lists of arrays (length 5 one array per finger) 
		flex_array0 = []
		flex_array1 = []
		time_array0 = []
		time_array1 = []
		for key, flist in flex_events[0].items():
			flex_array0.append( np.asarray(flist) )
			time_array0.append(np.asarray(flex_times[0][key]))
		for key, flist in flex_events[1].items():
			flex_array1.append( np.asarray(flist) )
			time_array1.append(np.asarray(flex_times[1][key]))

		flex_full = []
		for flex_array in flex_array_full:
			flex_full.append( np.asarray(flex_array))

		return Fpair_full, Ndiff_full, flex_array0, flex_array1, flex_full


	def get_thumb_flex(self, idxminus=10, idxplus=0):
		trigevent = self.collect_trigger_events(idxminus =idxminus, idxplus=idxplus)
		# Fix subsequent functions and return trigevent instead of these.
		trigonevent = list(itertools.chain.from_iterable(trigevent['on']))	

		# Classified with flex_events[0] as not thumb under, and flex_events[1] as thumb under
		flex_events = {0: {0: [], 1: [], 2: [], 3: [], 4: []}, 1: {0: [], 1: [], 2: [], 3: [], 4: []}}
		flex_times ={0: {0: [], 1: [], 2: [], 3: [], 4: []}, 1: {0: [], 1: [], 2: [], 3: [], 4: []}}

		# Separate trigger events per finger
		prev_event = trigonevent[0]
		for idx, event in enumerate(trigonevent[1:]):
			prev_event = trigonevent[idx-1]
			flex = event['flex'][0, :] # Only take flex data of thumb! Other fingers might be useful
			time = event['senstime'] - event['trigtime'] 

			parallel_inc = ( event['note'] - prev_event['note'] ) * (event['pressidx'] - prev_event['pressidx'])
			if(parallel_inc >= 0):
				flex_events[0][event['pressidx']].append(flex)
				flex_times[0][event['pressidx']].append(time)
			else:
				flex_events[1][event['pressidx']].append(flex)
				flex_times[1][event['pressidx']].append(time)



		# Make lists of arrays (length 5 one array per finger) 
		flex_array0 = []
		flex_array1 = []
		time_array0 = []
		time_array1 = []
		for key, flist in flex_events[0].items():
			flex_array0.append( np.asarray(flist) )
			time_array0.append(np.asarray(flex_times[0][key]))
		for key, flist in flex_events[1].items():
			flex_array1.append( np.asarray(flist) )
			time_array1.append(np.asarray(flex_times[1][key]))

		# self.plot_thumbunder_flex(time_array0, flex_array0, time_array1, flex_array1)
		# plt.show()	

		return flex_array0, flex_array1, time_array0, time_array1


	def get_press_distribution(self, idxminus=10, idxplus=0):
		trigevent = self.collect_trigger_events(idxminus =idxminus, idxplus=idxplus)
		# Fix subsequent functions and return trigevent instead of these.
		trigonevent = list(itertools.chain.from_iterable(trigevent['on']))	
		trigoffevent = list(itertools.chain.from_iterable(trigevent['off']))	

		#finger_idx = {'thumb': 0, 'index': 1, 'major': 2, 'ring': 3, 'pinky': 4}
		press_events = {0: [], 1: [], 2: [], 3: [], 4: []}
		press_times = {0: [], 1: [], 2: [], 3: [], 4: []}
		release_events = {0: [], 1: [], 2: [], 3: [], 4: []}
		release_times = {0: [], 1: [], 2: [], 3: [], 4: []}
		vel = {0: [], 1: [], 2: [], 3: [], 4: []}

		# Separate trigger events per finger
		for event in trigonevent:
			p = event['press'][event['pressidx'], :]
			time = event['senstime'] - event['trigtime'] 
			press_events[event['pressidx']].append(p)
			press_times[event['pressidx']].append(time)
			vel[event['pressidx']].append(event['velocity'])

		for event in trigoffevent:	
			p = event['press'][event['pressidx'], :]
			time = event['senstime'] - event['trigtime'] # time relative to trigger time 
			release_events[event['pressidx']].append(p)
			release_times[event['pressidx']].append(time)

		# Make lists (of length 5) of arrays and  
		press_parray = []
		press_tarray = []
		release_parray = []
		release_tarray = []
		vel_array = []
		for key, plist in press_events.items():
			press_parray.append( np.asarray(plist) )
			press_tarray.append(np.asarray(press_times[key]))
			vel_array.append(np.asarray(vel[key]))

		#print(vel_array)

		for key, plist in release_events.items():
			release_parray.append( np.asarray(plist) )
			release_tarray.append(np.asarray(press_times[key]))

		self.plot_velocity_correlations(press_parray, press_tarray, vel_array)

		#self.plot_all_pressures(press_parray, press_tarray)
		#self.plot_all_pressures(release_parray, release_tarray)
		plt.show()	

	def collect_trigger_events(self, idxminus=10, idxplus=3):
		# data = [(timeSens, pressData, flexData, imuData, timeKeyboard, notes, velocity),...]
		
		data = self.collect_data()
		analyze = Analyze()
		trigevent = {'on': [], 'off': [], 'filename': []}
		for runidx, run in enumerate(data):
			timeSens, pressData, flexData, imuData, timeKeyboard, notes, velocity = run
			tnotes = analyze.getNotes(timeKeyboard, notes, velocity)

			press = np.asarray(list(map(list, zip(*pressData))))
			flex = np.asarray(list(map(list, zip(*flexData))))
			imu = np.asarray(list(map(list, zip(*imuData))))
			senstime = np.asarray(timeSens)

			trigonevent = []
			trigoffevent = []
			for nidx, note in enumerate(tnotes):
				idx_start = (np.abs(senstime - note[2])).argmin()
				idx_end = (np.abs(senstime - note[3])).argmin()
				start_range = np.array(range(idx_start-idxminus, idx_start+idxplus))
				end_range = np.array(range(idx_end-idxminus, idx_end+idxplus))

				# Find Index of pressure in use
				a = np.sum(press[ : , idx_start : idx_end], axis = 1)
				press_idx = a.argmax()

				# Add trigger events to trigon and trigoff events lists of dictionaries
				#print(press[:, start_range])
				trigonevent.append( {'press': press[ : , start_range], 
					'pressidx': press_idx,
					'flex':  flex[ : , start_range], 
					'imu':  imu[ : , start_range],
					'senstime': senstime[start_range], 
					'velocity': note[1],
					'note': note[0],
					'trigtime': note[2]} )

				trigoffevent.append( {'press': press[ : , end_range], 
					'pressidx': press_idx,
					'flex':  flex[ : , end_range], 
					'imu':  imu[ : , end_range], 
					'senstime': senstime[end_range],
					'note': note[0],
					'trigtime': note[2]} )

			trigevent['on'].append(trigonevent)
			trigevent['off'].append(trigoffevent)
			trigevent['filename'].append(self.filelist[runidx])


		return trigevent
		#return totaltrigon, totaltrigoff 

	def write_finger_to_note(self, overwrite=True):

		# C-scale aware: transform midi notes to indexed C-scale [60, 62, 64, 65...] -> [20, 21, 22, 23,...]
		Cnotes = []
		for key, val in csv.reader(open('data/tables/note2num.csv')):
			if(len(key)==2):
				Cnotes.append(val)
		Cnotes_val2idx = dict()
		for idx, val in enumerate(sorted(Cnotes)):
			Cnotes_val2idx[int(val)] = idx 

		# Start saving
		trigevent = self.collect_trigger_events()
		for runidx, trigonevent in enumerate(trigevent['on']):

			# Make file with note-to-finger list 
			reader = ReadWrite(self.directory, self.filelist[runidx])
			filename = reader.getFullPath() + self.filelist[runidx] + '_noteandfinger.csv'
			print(filename)
			if(overwrite or os.path.isfile(filename)):
				ff = open(filename, 'w')
				ff.write('note, finger' + '\n')
				for event in trigonevent:	
					ff.write(str(event['note']) + ', ' + str(event['pressidx']) + '\n')
				ff.close()




			# Make incremental vectors
			Cn = []
			for event in trigonevent:
				note = event['note']
				#print('note: {}'.format(note))
				if note not in Cnotes_val2idx.keys(): # A note outside the C scale was played (by mistake)
					note += 1
				Cn.append( Cnotes_val2idx[note] ) 

			Cndiff = []
			Fdiff = []
			Ftuple = [] # list of consecutive fingers 
			for i in range(1, len(trigonevent)):
				cdif = Cn[i] - Cn[i-1]
				Cndiff.append( cdif )
				Fdiff.append(trigonevent[i]['pressidx'] - trigonevent[i-1]['pressidx'] )
				Ftuple.append( (trigonevent[i-1]['pressidx'], trigonevent[i]['pressidx']  )  )


			# Save to file
			# Make file with noteincrement-to-fingerincrement
			filename = reader.getFullPath() + self.filelist[runidx] + '_noteandfinger_inc.csv'
			if(overwrite or os.path.isfile(filename)):
				ff = open(filename, 'w')
				ff.write('finger_increment, note_increment' + '\n')
				for i in range(len(Fdiff)):
					ff.write(str(Fdiff[i]) + ', ' + str(Cndiff[i]) + '\n')
				ff.close()

			# Make file with (previous_finger, present_finger, note_increment) 
			filename = reader.getFullPath() + self.filelist[runidx] + '_noteandfinger_noteinc.csv'
			if(overwrite or os.path.isfile(filename)):
				ff = open(filename, 'w')
				ff.write('previous_finger, present_finger, note_increment' + '\n')
				for i in range(len(Fdiff)):
					ff.write(str(Ftuple[i][0]) + ', ' + str(Ftuple[i][1]) + ', ' + str(Cndiff[i]) + '\n')
				ff.close()

			# Make file with (previous_finger, present_finger, note_increment, pre_trigger_thumb_flex_sensor) 
			filename = reader.getFullPath() + self.filelist[runidx] + '_noteandfinger_andflex_noteinc.csv'
			if(overwrite or os.path.isfile(filename)):
				ff = open(filename, 'w')
				ff.write('previous_finger, present_finger, note_increment, thumb_flex' + '\n')
				for i in range(len(Fdiff)):
					thumbflex = trigonevent[i]['flex'][0, :]
					ff.write(str(Ftuple[i][0]) + ', ' + str(Ftuple[i][1]) + ', ' + str(Cndiff[i]) + ', '+ ', '.join( str(t) for t in thumbflex )+ '\n')
				ff.close()

	def collect_data(self):
		if(not os.path.isdir(self.directory)):
			raise Exception("wrong data directory")
		
		self.filelist = self.get_filelist()
		full_data = []
		for filename in self.filelist:
			reader = ReadWrite(self.directory, filename)
			full_data.append( reader.readData() )

		return full_data

	def get_filelist(self):
		if self.includefile == '':
			return next(os.walk(self.directory))[1]
		else:
			# Read the include-exclude file determining which files to read
			# If first line == 'exlude', exclude subsequent lines, if == 'include', include subsequent lines
			with open(self.includefile, 'r') as F:
				incex_option = (F.readline()).split('\n')[0]
				incex_filelist = []
				g = 'initial'
				while 1:
					g = F.readline()
					if(g == ''):
						break
					incex_filelist.append(g.split('\n')[0])

			# exclude or include files from list
			filelist = next(os.walk(self.directory))[1]
			incidx = []
			for idx, f in enumerate(filelist):
				for incex_file in incex_filelist:
					prefix = '_'.join(f.split('_')[:-1])
					if incex_file == prefix: 
						print(prefix)
						incidx.append(idx)
			if(incex_option == 'include'):
				chosenfiles = [f for i, f in enumerate(filelist) if i in set(incidx)]
			elif(incex_option == 'exclude'):
				chosenfiles = [f for i, f in enumerate(filelist) if i not in set(incidx)]
			else:
				raise Exception("wrong option")

			print("Files chosen:")
			print(chosenfiles)

		return chosenfiles 

	def plot_thumbunder_flex(self, time_array0, flex_array0, time_array1, flex_array1):
		for i in range(len(time_array0)):
			plt.figure(i)
			plt.subplot(121)
			for j in range(time_array0[i].shape[0]):
				plt.plot(time_array0[i][j, :], flex_array0[i][j, :])
			plt.xlabel('relative time')
			plt.ylabel('flex')
			plt.title('NO thumb under with present finger '+ str(i))

			plt.subplot(122)
			for j in range(time_array1[i].shape[0]):
				plt.plot(time_array1[i][j, :], flex_array1[i][j, :])
			plt.xlabel('relative time')
			plt.ylabel('flex')
			plt.title('Thumb under with present finger '+ str(i))

		plt.figure(5)
		plt.subplot(121)
		for i in range(len(time_array0)):
			for j in range(time_array0[i].shape[0]):
				plt.plot(time_array0[i][j, :], flex_array0[i][j, :])
		plt.xlabel('relative time')
		plt.ylabel('flex')
		plt.title('NO thumb under for all fingers') 

		plt.subplot(122)
		for i in range(len(time_array1)):
			for j in range(time_array1[i].shape[0]):
				plt.plot(time_array1[i][j, :], flex_array1[i][j, :])
		plt.xlabel('relative time')
		plt.ylabel('flex')
		plt.title('Thumb under for all fingers')


	def plot_velocity_correlations(self, parray, tarray, velarray):

		mean_der_list = []
		mean_int_list = []
		for i in range(len(parray)):
			num_trig = parray[i].shape[0]
			mean_der = np.zeros((num_trig, 2))
			mean_int = np.zeros((num_trig, 2))
			for j in range(num_trig):
				mean_der[j, 0] = np.mean( np.divide( np.diff(parray[i][j, :]), np.diff(tarray[i][j, :]) ))
				mean_int[j, 0] = np.mean( np.multiply(parray[i][j, :-1], np.diff(tarray[i][j, :])) )
				mean_der[j, 1] = velarray[i][j]
				mean_int[j, 1] = velarray[i][j]
				#print(parray[i][j, :], velarray[i][j])
			mean_der_list.append(mean_der)
			mean_int_list.append(mean_int)

		plt.figure(1)
		sp0 = 231
		for i in range(len(parray)):
			plt.subplot(sp0+i)
			plt.scatter(mean_der_list[i][:, 1], mean_der_list[i][:, 0])
			plt.xlabel('velocity')
			plt.ylabel('mean derivative')

		plt.figure(2)
		sp0 = 231
		for i in range(len(parray)):
			plt.subplot(sp0+i)
			plt.scatter(mean_int_list[i][:, 1], mean_int_list[i][:, 0])
			plt.xlabel('velocity')
			plt.ylabel('mean integral')


	def plot_all_pressures(self, parray, tarray):
		plt.figure(3)
		sp0 = 231
		print(parray)
		for i in range(len(parray)):
			plt.subplot(sp0+i)
			for j in range(parray[i].shape[0]):
				plt.plot(tarray[i][j, :], parray[i][j, :])
			plt.xlabel('relative time')
			plt.ylabel('pressure')


