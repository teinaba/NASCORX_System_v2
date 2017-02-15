#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/device/')
import MG3692C

SG = MG3692C.mg3692c(IP='192.168.100.113', GPIB=14)

def debug_freq():
    freq = [2.0, 5.5, 10, 18.5, 20]
    for i in range(5):
        print('****************')
        print('TEST CASE: freq='+str(freq[i]))
        SG.set_freq(freq[i])
        time.sleep(0.1)
        ret = SG.query_freq()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

def debug_power():
    power = [-20.0, -10, 0, 10.0, 30]
    for i in range(5):
        print('****************')
        print('TEST CASE: power='+str(power[i]))
        SG.set_power(power[i])
        time.sleep(0.1)
        ret = SG.query_power()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

def debug_output():
    onoff = [0, 1, -1, 1.0, 0]
    for i in range(5):
        print('****************')
        print('TEST CASE: RF='+str(onoff[i]))
        SG.set_output(onoff[i])
        time.sleep(0.1)
        ret = SG.query_output()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))


debug_freq()
debug_power()
debug_output()


