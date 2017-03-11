#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX_System-master/manage/')
import Ymap
m = Ymap.ymap()

#Setting Parameter
LOrange=[90.0, 120.0] #GHz
LOres=5.0 #GHz
Vrange=[7.0, 8.4] #mV
Vres=0.1 #mV
Irange=[10.0, 30.0] #%
Ires=5.0 #%
logpath='/home/amigos/data/SIS/'

def measure():
    m.makemap(LOrange, LOres, Vrange, Vres, Irange, Ires, logpath)

measure()
