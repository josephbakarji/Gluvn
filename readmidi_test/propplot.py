import csv, sys, os
import matplotlib.pyplot as plt

from mido import MidiFile



runname = 'run1'
hand = 'R'
filename = 'data/'+runname+hand

mid = MidiFile(filename+'.mid')
f2read = filename+'.csv'

with open(f2read, 'r') as (g):
    reader = csv.reader(g)
    read_list = list(reader)
header = read_list.pop(0)
for readvec in read_list:
    readvec[0] = float(readvec[0])
    for i in range(1, len(readvec)):
        readvec[i] = int(readvec[i])

tlist = [ vec[0] for vec in read_list ]
p5 = [vec[-1]*(int(vec[-1]!=255)) for vec in read_list]
p4 = [vec[-2]*(int(vec[-2]!=255)) for vec in read_list]
p3 = [vec[-3]*(int(vec[-3]!=255)) for vec in read_list]
p2 = [vec[-4]*(int(vec[-4]!=255)) for vec in read_list]
p1 = [vec[-5]*(int(vec[-5]!=255)) for vec in read_list]
f1 = [vec[-10]*0.7 for vec in read_list]

fig = plt.figure(1)
axes = plt.gca()
axes.plot(tlist, p1, tlist, p2, tlist, p3, tlist, p4, tlist, p5, linewidth = 3)
axes.plot(tlist, f1, '--', linewidth = 3)
axes.set_xlim([10,15])
axes.set_ylim([0,255])
axes.legend(['Pressure 1 (Thumb)', 'Pressure 2', 'Pressure 3', 'Pressure 4', 'Pressure 5', 'Flex 1'])
plt.xlabel('Time (ndt)')
plt.ylabel('Sensor Reading')
plt.show()
fig.savefig('test.pdf')
