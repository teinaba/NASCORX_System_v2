#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
sys.path.append('/home/amigos/NASCORX-master/device/')
import TR72W

T = TR72W.tr72w(IP='192.168.100.105')

def debug_temp():
    print('****************')
    print('TEST FUNCTION: temp()')
    ret = T.temp()
    print('RETURN...')    
    print(ret)
    print('TYPE...')
    print(type(ret))

def debug_hum():
    print('****************')
    print('TEST FUNCTION: hum()')
    ret = T.hum()
    print('RETURN...')    
    print(ret)
    print('TYPE...')
    print(type(ret))

debug_temp()
debug_hum()

