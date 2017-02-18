#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/base/')
import numpy
import matplotlib.pyplot as plt
import Cryo

m = Cryo.mixer()


def debug_sis():
    Vs = 30
    m.set_sisv(Vmix=Vs, ch=1)
    time.sleep(0.1)
    ret = m.monitor_sis()
    for i in ret:
        print(i)
    ohm = (ret[2]*1e-2)/(ret[3]*1e-3)
    print('R = '+str(ohm))
    ret = m.query_sisv()
    print(ret)
    raw_input()

def debug_LOatt():
    m.set_loatt(att=50, ch=0)
    time.sleep(0.5)
    ret = m.query_loatt()
    print(ret)

def debug_sis2():
    ave = 3
    v = numpy.arange(0, 30, 5.0)
    Vlist = numpy.empty((0,len(v)), float)
    Ilist = numpy.empty((0,len(v)), float)
    for j in range(ave):
        xlist = numpy.array([])
        ylist = numpy.array([])
        for i in v:
            m.set_sisv(Vmix=i, ch=1)
            time.sleep(0.1)
            ret = m.monitor_sis()
            xlist = numpy.append(xlist, ret[2]*1e+1)
            ylist = numpy.append(ylist, ret[3]*1e+3)
        Vlist = numpy.append(Vlist, [xlist], axis=0)
        Ilist = numpy.append(Ilist, [ylist], axis=0)
    Vmon = numpy.mean(Vlist, axis=0)
    Imon = numpy.mean(Ilist, axis=0)
    xerr = numpy.std(Vlist, axis=0)
    yerr = numpy.std(Ilist, axis=0)
    print(Vmon)
    print(Imon)
    plt.errorbar(Vmon, Imon, xerr=xerr, yerr=yerr, fmt='.', ecolor='red', color='red', label='A/D value')
    plt.plot([v[0], v[-1]], [v[0], v[-1]/51.*1e+3], color='green', label='51 ohm')
    plt.xlim(v[0], v[-1])
    plt.ylim(v[0], v[-1]/51.*1e+3)
    plt.title('I - V (R=51ohm)')
    plt.xlabel('Mixer Voltage [mV]')
    plt.ylabel('Mixer Current [uA]')
    plt.grid(True)
    plt.legend(loc='upper left')
    plt.text(2.0, 10, 'average: '+str(ave))
    plt.savefig('/home/amigos/urushihara/IVremote_test2.png')
    plt.show()

def debug_sis3():
    V_list = numpy.arange(0, 30, 5.0)

    Vmon_list = numpy.array([])
    Imon_list = numpy.array([])
    for i in V_list:
        m.set_sisv(Vmix=i, ch=1)
        time.sleep(0.1)
        ret = m.monitor_sis()
        Vmon_list = numpy.append(Vmon_list, ret[2]*1e+3)
        Imon_list = numpy.append(Imon_list, ret[3]*1e+3)
    
    plt.subplot(1, 2, 1)
    plt.plot(V_list, Vmon_list, marker='o', color='red', label='remote')
    plt.plot([V_list[0], V_list[-1]], [V_list[0], V_list[-1]*1e+2], color='green', label='ideal')
    plt.xlim(V_list[0], V_list[-1])
    plt.ylim(V_list[0], V_list[-1]*1e+2)
    plt.title('Vmix - Vmon (R=51ohm)')
    plt.xlabel('Mixer Voltage [mV]')
    plt.ylabel('Monitor Voltage [mV]')
    plt.grid(True)
    plt.legend(loc='upper left')

    plt.subplot(1, 2, 2)
    plt.plot(V_list, Imon_list, marker='o', color='red', label='remote')
    plt.plot([V_list[0], V_list[-1]], [V_list[0], V_list[-1]/51.*1e+3], color='green', label='ideal')
    plt.xlim(V_list[0], V_list[-1])
    plt.ylim(V_list[0], V_list[-1]/51.*1e+3)
    plt.title('Vmix - Imon (R=51ohm)')
    plt.xlabel('Mixer Voltage [mV]')
    plt.ylabel('Monitor Current [mV]')
    plt.grid(True)
    plt.legend(loc='upper left')

    plt.savefig('/home/amigos/urushihara/IVremote_test3.png')
    plt.show()



debug_sis3()
#debug_LOatt()
