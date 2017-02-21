# ! /usr/bin/env python
# _*_ coding: UTF-8 _*_


# import modules
import time, sys, datetime, os, signal
import matplotlib.pyplot as plt
import numpy as np
sys.path.append('/home/amigos/NASCORX_System-master/base/')
import Cryo
sys.path.append('/home/amigos/NASCORX_System-master/device/')
import CPZ340816

class Boxchecker(object):
    def __init__(self):
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.box = Cryo.mixer()
        self.DA = CPZ340816.cpz340816()

    def Vrem_Vout_ratio_tune(self, ch, Vrem=5.0):
        print('Now start tuning of the Vrem to Vout ratio (ch = '+str(ch)+')')
        print('Connect "Remote Controller" to the D/A Board.')
        print('Connect "Vmon" to the voltmeter.')
        print('Ready?: [yes]/no')
        sf = raw_input()
        if sf != 'no':
            quit()
        else:
            voltage = float(Vrem)
            DA.set_voltage(voltage=voltage, ch=ch)
            DA.set_output(onoff=1)
        while 1:
            Vnow = DA.query_voltage()
            Vrem = round(float(Vnow[ch]), 3)
            print('........................'
            print('Now D/A output --> '+str(Vrem)+' V')
            print('Target voltage --> '+str(Vrem*0.3)+' V')
            print(' Finish Tuning: input "bien"')
            print(' Change the D/A Output: input 0 -- 10 [V]')
            ef = raw_input()
            if ef == 'bien':
                break
            elif 0.0<=ef<=10.0:
                voltage = float(ef)
                DA.set_voltage(voltage=voltage, ch=ch)
            else:
                pass
        DA.set_output(onoff=0)
        print('Finish Tuning!')
    return

# written by K.Urushihara
