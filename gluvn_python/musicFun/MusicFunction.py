'''
Created on May 21, 2016

@author: josephbakarji
'''



def accid(notename):
    ac = len(notename[1:-1]) # first letter is a note and last a number
    if '#' in notename:
        return ac
    elif 'b' in notename:
        return -ac
    else:
        return 0
    
# replace this bulky function by a dictionary.
def notename_to_midinum(notename):
    notes = "C . D . E F . G . A . B".split()
    name = notename[0:1].upper()  # upper() makes string upper case
    number = notes.index(name)
    ac = accid(notename)
    localmidinum =  (number + ac)%12
    globalmidinum = 12 * (int(notename[-1]) + 2) + localmidinum
    return globalmidinum