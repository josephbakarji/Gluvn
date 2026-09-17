'''
Created on Jun 16, 2016

@author: josephbakarji
'''

import csv
import time  
from collections import deque

pressq = deque(maxlen = 50)
flexq = deque(maxlen = 50)
imuq = deque(maxlen = 50)

with open('../data/sim/rnd_glove_ypr_press.csv', 'r') as f:
    reader = csv.reader(f)
    read_list = list(reader)
    
header = read_list.pop(0)
for readvec in read_list:
    readvec[0] = float(readvec[0])
    for i in range(1,len(readvec)):
        readvec[i] = int(readvec[i])
    
tlist = [vec[0] for vec in read_list]

t0 = time.time()
for i in range(len(tlist)):
    while(time.time() - t0 < tlist[i]):
        pass
    imuq.append(read_list[i][1:7])
    flexq.append(read_list[i][7:12])
    pressq.append(read_list[i][12:17])
    print(pressq[-1])
