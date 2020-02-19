from data_analysis import Analyze
from __init__ import keyboard_portname, EXPDIR, simDir, learnDir, settingsDir, figDir, portL, portR, baud
from run_glove import RunGlove
from data_analysis import Analyze, Stats, Analyze2Hands
from learning import Learn
import numpy as np
import matplotlib.pyplot as plt
import sys
import copy



def plot_allsensors():
    plotter = Analyze2Hands(directory='', filename='test')
    plotter.plotSensors(savename='all_sensors')

def printsensors():
    gluvn = RunGlove(directory=EXPDIR, directory)
    gluvn.printAllSens(hands='both', printit=False, save=True, plot=True)

def record():
    f = 'two_scales'
    gluvn = RunGlove(directory=learnDir, filename=f)
    gluvn.recordKeyboardAndGlove()

def plot_press():
    # Plot pressure sensors around trigger on and trigger off events (to learn velocity and trigger)
    stat = Stats(includefile=settingsDir + 'includeSingleNotes.txt') 
    stat.get_press_distribution(5, 0)

def plot_NoteAndSens():
    # Plot time series of note (volume) and sensors of specific file 
    # TODO: save it to the folder

    # files = next(os.walk(directory))[1]
    # for f in files:	
    f = 'two_scales_1'
    A = Analyze(filename=f)
    A.plotKeyAndGlove('basic', savename=f)

def write_note2finger_files():
    # write csv files with (Finger_diff, Note_diff) columns
    stat = Stats(includefile='') 
    stat.write_finger_to_note()

def learn_and_test():
    # Make training/test sets and make predictions based on transition probability (Tmat is the matrix predicting Note_diff based on Finger_diff)
    # mode = 'bothinc' uses (finger_previous - finger_present) as features
    # mode = 'noteinc' uses (finger_previous, finger_present) as features
    # mode = 'thumbflex' uses (finger_previous, finger_present, thumb_flex) as features

    L = Learn(includefile=settingsDir + 'predIntervals.txt', 
            mode='bothinc', 
            trainsize=0.7)

    print('NO THUMB UNDER')
    data = L.learn_transition_prob()
    L.predict_transition_prob(data)
    L.plot_transition_matrix(data)

    print('with thumb under')
    data = L.learn_transition_prob_withThumbUnder()
    L.predict_transition_prob(data)
    L.plot_transition_matrix(data)

    plt.show()
    # print(data.Tmat)
    # for key in sorted(data.Tprob.keys()):
    # 	print(key, data.Tprob[key])

    # Avarage Accuracy
    # accuracy = []
    # for _ in range(40):
    # 	accuracy.append( L.predict_transition_prob(data) )
    # print('Mean Mean Accuracy = {}'.format(sum(accuracy)/len(accuracy)))

    # Plot transition Matrix

def plot_transition():

    L = Learn(includefile=settingsDir + 'predIntervals.txt', 
            mode='bothinc', 
            trainsize=0.7)

    data = L.learn_transition_prob()
    L.predict_transition_prob(data)
    L.plot_transition_matrix(data)
    plt.show()

def plot_thumbunder_flex():
    stat = Stats(includefile=settingsDir + 'thumbunderRuns.txt') 
    flex_array0, flex_array1, time_array0, time_array1 = stat.get_thumb_flex(15, 1)
    stat.plot_thumbunder_flex(time_array0, flex_array0, time_array1, flex_array1)
    plt.show()	

def thumbunder_Logistic():
    L = Learn(includefile=settingsDir + 'excludeSingleNotes.txt', trainsize=0.8)

    a = []
    pretrigflex = range(3, 30)
    # for n in pretrigflex:
    # 	print(n)
    # 	a.append(accuracy)
    # fig = plt.figure(10)
    # for i in range(len(a[0])):
    # 	v = [ac[i] for ac in a]
    # 	plt.plot(pretrigflex, v)
    # plt.legend(['Thumb', 'Index', 'Major', 'Ring'])
    # plt.show()

    # filelist = stat.get_filelist() 
    Fpair_full, Ndiff_full, flex_array0, flex_array1, flex_full = L.stat.get_features_wtu(idxminus=10, idxplus=0)
    tu_predictor, accuracy, CM = L.learn_thumbunder(flex_array0, flex_array1) # Returns thumb_under predictors of length[1]

    objects = ('Thumb', 'Index', 'Major', 'Ring')
    y_pos = np.arange(len(objects))

    plt.bar(y_pos, accuracy, align='center', alpha=0.9)
    plt.xticks(y_pos, objects, fontsize=13)
    plt.ylabel('Accuracy', fontsize=18)
    plt.title('Thumb Under Prediction Accuracy', fontsize=18)
    plt.xlabel('Post-TU Finger', fontsize=15)
    plt.savefig(figDir + 'TU_accuracy.pdf')


    fig = plt.figure()
    for tidx in CM.keys():
        plt.subplot(221+tidx)
        ax = plt.gca()
        print(CM[tidx])
        c = ax.pcolormesh(range(3), range(3), CM[tidx], cmap='Blues')#, vmin=0.0, vmax=1.0)
        fig.colorbar(c, ax=ax)
        plt.xlabel('Actual Thumb-under', fontsize=10)
        plt.ylabel('Predicted Thumb-under', fontsize=10)

    plt.savefig(figDir + 'TUcm.pdf')



    fig = plt.figure()
    ax = plt.gca()
    print(CM[1])
    CMcopy = copy.deepcopy(CM[0])
    CM[0][0,:] = CM[0][0,:]/np.sum(CM[0][0,:])
    CM[0][1,:] = CM[0][1,:]/np.sum(CM[0][1,:])
    c = ax.pcolor(CM[0], cmap='Blues')#, vmin=0.0, vmax=1.0)
    fig.colorbar(c, ax=ax)
    plt.xlabel('Actual Thumb-under', fontsize=10)
    plt.ylabel('Predicted Thumb-under', fontsize=10)

    for i in range(2):
        for j in range(2):
            plt.text(j, i, int(CMcopy[i, j]), color='k', fontsize=15)

    labelx = [item.get_text() for item in ax.get_xticklabels()]
    labely = [item.get_text() for item in ax.get_yticklabels()]
    for i in range(len(labelx)):
        labelx[i] = ''
        labely[i] = ''
    labelx[1] = '0'
    labely[2] = '0'
    labelx[3] = '1'
    labely[6] = '1'
    ax.set_xticklabels(labelx)
    ax.set_yticklabels(labely)
    plt.show()

def main():
    fn = 'test0_7'
    R = RunGlove(directory=EXPDIR, filename=fn)
    R.twoHandInstrument(fromfile=True)
    #R.aiTrigger()

if __name__ == '__main__':


    if len(sys.argv) > 1:
        if sys.argv[1] == 'record':	
            record()
        elif sys.argv[1] == 'learn':
            learn_and_test()
        elif sys.argv[1] == 'makefiles':
            write_note2finger_files()
        elif sys.argv[1] == 'flexlearn':
            thumbunder_Logistic()
        elif sys.argv[1] == 'showdata':
            plot_NoteAndSens()
        elif sys.argv[1] =='plottransition':
            plot_transition()
        elif sys.argv[1] == 'play':
            runglove()
        elif sys.argv[1] == 'thumbunder':
            thumbunder_Logistic()
        elif sys.argv[1] == 'printsensors':
            printsensors()
        else:
            print("something went wrong in selecting app")
    else:
        main()
        #gluvn.simpleTrigger(['C3', 'D3', 'E3', 'F3', 'G3'], 15)
