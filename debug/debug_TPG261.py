#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/device/')
import TPG261

P = TPG261.tpg261(IP='192.168.100.112', port=9600)

def debug_pressure():
    for i in range(3):
        print('****************')
        print('TEST CASE: '+str(i+1))
        ret = P.query_pressure()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

def debug_unit():
    unit = ['torr', 'bar', 'pascal', 'torr']
    for i in range(len(unit)):
        print('****************')
        print('TEST CASE: unit='+unit[i])
        P.set_unit(unit=unit[i])
        ret = P.query_unit()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

def debug_baurate():
    for i in range(3):
        print('****************')
        print('TEST CASE: '+str(i+1))
        P.set_baurate(rate=9600)
        ret = P.query_baurate()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

def debug_gauge():
    for i in range(3):
        print('****************')
        print('TEST CASE: '+str(i+1))
        ret = P.query_gauge()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

debug_pressure()
debug_unit()
debug_baurate()
debug_gauge()
