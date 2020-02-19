'''
Created on May 21, 2016

@author: josephbakarji
'''

from MusicFunction import *
import csv



def makeallnotes(notes, octrange):
    allnotes = []
    for i in octrange:
        for j in notes:
            a = j + str(i)
            allnotes.append(a)
    return allnotes
    
def noterange(start, end, allnotes):
    return allnotes[allnotes.index(start) : allnotes.index(end)+1]
                
ns = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
nb = ['C', 'Db', 'D', 'Eb', 'E', 'F', 'Gb', 'G', 'Ab', 'A', 'Bb', 'B']

orange = range(0,6)
keyboards = makeallnotes(ns,orange)
#noteslices = noterange('C1','F#5',keyboards)

keyboardb = makeallnotes(nb,orange)
#notesliceb = noterange('C0','Gb5',keyboardb)


tuplist = []
tuplist_num2note = []
for i in range(len(keyboards)):
    tuplist.append((keyboards[i], notename_to_midinum(keyboards[i])))
    tuplist.append((keyboardb[i], notename_to_midinum(keyboardb[i])))

    tuplist_num2note.append((notename_to_midinum(keyboards[i]), keyboards[i]))

tupdict = dict(tuplist)
tupdict_num2note = dict(tuplist_num2note)

w2 = csv.writer(open("../data/tables/num2note.csv", "w"))
for key, val in tupdict_num2note.items():
    w2.writerow([key, val])

w = csv.writer(open("../data/tables/note2num.csv", "w"))
for key, val in tupdict.items():
    w.writerow([key, val])




dictest = {}
for key, val in csv.reader(open("../data/tables/note2num.csv")):
    print(key)
    dictest[key] = val

print(dictest)