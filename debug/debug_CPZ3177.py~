#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/device/')
import CPZ3177

AD = CPZ3177.cpz3177(dev=1)

def debug_mode():
    mode = ['single', 'diff', 'hoge']
    for i in range(len(mode)):
        print('****************')
        print('TEST CASE: mode='+mode[i])
        AD.set_mode(mode=mode[i])
        ret = AD.query_mode()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

def debug_input():
    for i in range(3):
        print('****************')
        print('TEST CASE: '+str(i+1))
        ret = AD.query_input()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

def debug_range():
    for i in range(3):
        print('****************')
        print('TEST CASE: '+str(i+1))
        ret = AD.query_input_range()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

debug_mode()
debug_input()
debug_range()

