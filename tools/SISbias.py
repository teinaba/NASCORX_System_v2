# ! /usr/bin/env python
# _*_ coding: UTF-8 _*_


# import modules
import time, sys, signal
import numpy as np
sys.path.append('/home/amigos/NASCORX-master/base/')
import Cryo

def bias(ch=0):
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
    box = Cryo.mixer()
    while 1:
        print('INPUT SIS Voltage: 0 -- 30 [mV]')
        ret = raw_input()
        voltage = float(ret)
        box.set_sisv(Vmix=voltage, ch=ch)
        time.sleep(0.5)
        ret = box.monitor_sis()
        if ch == 0:
            Vmon = ret[0]*1e+1
            Imon = ret[1]*1e+3
        else:
            Vmon = ret[2]*1e+1
            Imon = ret[3]*1e+3
        print('Vmon: '+str(Vmon)+' mV')
        print('Imon: '+str(Imon)+' uA')

bias(ch=0)

# written by K.Urushihara
