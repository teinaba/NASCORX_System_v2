#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/device/')
import L218

L = L218.l218(IP='192.168.100.106', GPIB=6)

def debug_measure():
    for i in range(10):
        print('****************')
        print('TEST CASE: ch='+str(i+1))
        ret = L.measure(ch=i+1)
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

def debug_loop(seq=1):
    for i in range(seq):
        print('****************')
        print('TEST SEQUENCE: '+str(i+1))
        ret = L.measure(ch=1)
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))
        time.sleep(0.1)

debug_measure()
#debug_loop(seq=10000)
