# ! /usr/bin/env python
# _*_ coding: UTF-8 _*_


# import modules
import time, sys, signal
import numpy as np
sys.path.append('/home/amigos/NASCORX-master/base/')
import Cryo

def Loatt(ch=0):
    '''
    DESCRIPTION
    ================

    ARGUMENT
    ================
        1. : 
            Type: 

    RETURN
    ================
    Nothing.
    '''
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    att = Cryo.mixer()
    while 1:
        print('INPUT LO att. 0 -- 100 [mA]')
        ret = raw_input()
        current = float(ret)
        att.set_loatt(att=current, ch=ch)

if __name__ == '__main__':
    Loatt()

# written by K.Urushihara
