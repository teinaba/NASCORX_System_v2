# ! /usr/bin/env python
# _*_ coding: UTF-8 _*_


# import modules
import time, sys, datetime, os, csv
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import numpy as np
sys.path.append('/home/amigos/NASCORX-master/base/')
import IF
sys.path.append('/home/amigos/NASCORX-master/device/')
import ML2437A

def linearity(average=5, pngpath='/home/amigos/data/linearity/'):
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
    t = datetime.datetime.now()
    ut = t.strftime('%Y%m%d%H%M%S')
    filename = 'Linearity'+ut+'.png'

    pm = ML2437A.ml2437a(IP='192.168.100.113', GPIB=13)
    driver = IF.attenuation()
    
    P_list = np.array([])
    Perr_list = np.array([])
    for i in range(12):
        driver.set_att(X1=i, Y1=11, X2=11, Y2=11)
        dP_list = np.array([])
        time.sleep(0.5)
        for j in range(average):
            ret = pm.measure()
            dP_list = np.append(dP_list, ret)    
            time.sleep(0.1)
        dP_mean = np.mean(dP_list, axis=0)
        dP_std = np.std(dP_list, axis=0)
        P_list = np.append(P_list, dP_mean)
        Perr_list = np.append(Perr_list, dP_std)
        print('Att '+str(i)+': '+str(dP_mean))

    att = np.arange(0, 12, 1)
    plt.errorbar(att, P_list, yerr=Perr_list, fmt='.', ecolor='red', color='red', label='Power Meter')
    P_ideal = np.array([])
    for i in range(12):
        P_ideal = np.append(P_ideal, P_list[0]-i)
    plt.plot(att, P_list, color='green', label='interpolation')
    plt.title('Linearity '+t.strftime('%Y/%m/%d/ %H:%M:%S'))
    plt.xlim(0, 12)
    plt.xlabel('Attenuation [dB]')
    plt.ylabel('Power Level [dBm]')
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.savefig(pngpath+filename)
    plt.show()
 

linearity()

# written by K.Urushihara
