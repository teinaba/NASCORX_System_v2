#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/device/')
import FSW_0020

SG = FSW_0020.fsw_0020(IP='192.168.100.215', port=10001)

def debug_all():
    freq = [1.0, 0.8, 0.6, 0.5, 0.6]
    power = [-5.0, -6.0, 13.0, -8.0, -10.0]
    for i in range(5):
        print('****************')
        print('TEST CASE: '+str(i+1))
        print(' freq = '+str(freq[i]))
        print(' power = '+str(power[i]))
        SG.set_freq(freq[i])
        SG.set_power(power[i])
        time.sleep(0.1)
        ret = SG.query_freq()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))
        ret = SG.query_power()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))
        SG.set_output(onoff=1)
        ret = SG.query_output()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))
        SG.set_output(onoff=0)

def debug_ref():
    print('****************')
    print('TEST CASE: INT Ref.')
    SG.set_ref(source='INT')
    ret1 = SG.query_ref()
    print('RETURN...')
    print(ret1)
    print('TYPE...')
    print(type(ret1))
    print('****************')
    print('TEST CASE: EXT Ref.')
    SG.set_ref(source='EXT')
    ret2 = SG.query_ref()
    print('RETURN...')
    print(ret2)
    print('TYPE...')
    print(type(ret2))
    print('****************')
    print('TEST CASE: INT Ref.')
    SG.set_ref()
    ret3 = SG.query_ref()
    print('RETURN...')
    print(ret3)
    print('TYPE...')
    print(type(ret3))

def debug_refout():
    print('****************')
    print('TEST CASE: Ref. OFF')
    SG.set_refout(onoff=0)
    ret1 = SG.query_refout()
    print('RETURN...')
    print(ret1)
    print('TYPE...')
    print(type(ret1))
    print('****************')
    print('TEST CASE: Ref. ON')
    SG.set_refout(onoff=1)
    ret2 = SG.query_refout()
    print('RETURN...')
    print(ret2)
    print('TYPE...')
    print(type(ret2))
    print('****************')
    print('TEST CASE: Ref. OFF')
    SG.set_refout()
    ret3 = SG.query_refout()
    print('RETURN...')
    print(ret3)
    print('TYPE...')
    print(type(ret3))

def debug_temp():
    print('****************')
    print('TEST: query_temp()')
    ret = SG.query_temp()
    print('RETURN...')
    print(ret)
    print('TYPE...')
    print(type(ret))


debug_ref()
debug_refout()
debug_temp()
debug_all()


