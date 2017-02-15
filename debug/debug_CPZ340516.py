#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/device/')
import CPZ340516

DA = CPZ340516.cpz340516(dev=1)

def debug_range():
    mode = ['DA_0_1mA', 'DA_0_1mA', 2, 'DA_0_100mA']
    ch = [3, 15, 0, None]
    for i in range(len(mode)):
        print('****************')
        print('TEST CASE: '+str(i+1))
        print(' mode: '+str(mode[i]))
        print(' CH: '+str(ch[i]))
        DA.set_Irange(mode=mode[i], ch=ch[i])
        ret = DA.query_Irange()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

def debug_current():
    clist = [0.004, 1.0, 0.0005]
    for j in range(len(clist)):
        for i in range(9):
            print('****************')
            print('TEST CASE: ')
            print(' mode: ch1=DA_0_1mA')
            print(' current: '+str(clist[j]))
            print(' CH: '+str(i))
            DA.set_Irange(mode='DA_0_100mA')
            DA.set_Irange(mode='DA_0_1mA', ch=1)
            DA.set_current(current=clist[j], ch=i)
            ret = DA.query_current()
            print('RETURN...')
            print(ret)
            print('TYPE...')
            print(type(ret))

def debug_output():
    onoff = [0, 1, 0, 1, 0]
    for i in range(len(onoff)):
        print('****************')
        print('TEST CASE: onoff='+str(onoff[i]))
        print(' mode: DA_0_100mA')
        print(' current: 2mA')
        DA.set_Irange(mode='DA_0_100mA')
        DA.set_current(current=0.002)
        DA.set_output(onoff=onoff[i])
        ret = DA.query_output()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

debug_range()
debug_current()
debug_output()
