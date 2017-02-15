#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/device/')
import ML2437A

pm = ML2437A.ml2437a(IP='192.168.100.113', GPIB=13)


def functiontest():
    for i in [1, 2, 3]:
        ret = pm.measure(resolution=i)
        print(ret)

    ret = pm.query_average_onoff()
    print(ret)
    pm.set_average_count(count=30)
    ret = pm.query_average_count()
    print(ret)
    pm.set_average_onoff(onoff=0)
    ret = pm.query_average_onoff()
    print(ret)
    pm.set_average_onoff(onoff=1)
    ret = pm.query_average_onoff()
    print(ret)
    
functiontest()
