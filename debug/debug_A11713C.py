#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/device/')
import A11713C

driver = A11713C.a11713c(IP='192.168.100.114')

def debug_model():
    print('****************')
    print('TEST CASE: 1X:AG8494g 1Y:AG8494g 2X:NA 2Y:NA')
    driver.set_model(model='AG8494g', ch='1X')
    driver.set_model(model='AG8494g', ch='1Y')
    driver.set_model(model='NA', ch='2X')
    driver.set_model(model='NA', ch='2Y')
    ret = driver.query_model()
    print('RETURN...')
    print(ret)
    print('TYPE...')
    print(type(ret))
    print('****************')
    print('TEST CASE: 1X:AG8494g 1Y:AG8494g hoge:NA 2Y:hoge')
    driver.set_model(model='AG8494g', ch='1X')
    driver.set_model(model='AG8494g', ch='1Y')
    driver.set_model(model='NA', ch='hoge')
    driver.set_model(model='hoge', ch='2Y')
    ret = driver.query_model()
    print('RETURN...')
    print(ret)
    print('TYPE...')
    print(type(ret))

def debug_level():
    for i in range(14):
        print('****************')
        print('TEST CASE: level='+str(i-1))
        driver.set_level(level=i-1, ch='1X')
        driver.set_level(level=i-1, ch='1Y')
        ret = driver.query_level()
        print('RETURN...')
        print(ret)
        print('TYPE...')
        print(type(ret))

def debug_voltage():
    print('****************')
    print('TEST CASE: bank1:24V bank2:24V')
    driver.set_voltage(voltage='24V', bank=1)
    driver.set_voltage(voltage='24V', bank=1)
    ret = driver.query_voltage()
    print('RETURN...')
    print(ret)
    print('TYPE...')
    print(type(ret))
    print('****************')
    print('TEST CASE: bank1:hoge hoge:24V')
    driver.set_voltage(voltage='hoge', bank=1)
    driver.set_voltage(voltage='24V', bank='hoge')
    ret = driver.query_voltage()
    print('RETURN...')
    print(ret)
    print('TYPE...')
    print(type(ret))


debug_model()
debug_level()
debug_voltage()

