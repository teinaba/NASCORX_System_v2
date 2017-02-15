#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/device/')
import CPZ340816

DA = CPZ340816.cpz340816(dev=2)

def debug_voltage():
    voltage = [-1, 2.555555, 11, 0]
    ch = [None, 15, 0, None]
    for i in range(len(voltage)):
        print('****************')
        print('TEST CASE: '+str(i+1))
        print(' VOLTAGE: '+str(voltage[i]))
        print(' CH: '+str(ch[i]))
        DA.set_voltage(voltage=voltage[i], ch=ch[i])
        ret = DA.query_voltage()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

def debug_output():
    onoff = [0, 1, 'ON', 'OFF', 100, 0]
    for i in range(len(onoff)):
        print('****************')
        print('TEST CASE: '+str(i+1))
        print(' OUTPUT: '+str(onoff[i]))
        DA.set_output(onoff=onoff[i])
        ret = DA.query_output()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))


debug_voltage()
debug_output()

