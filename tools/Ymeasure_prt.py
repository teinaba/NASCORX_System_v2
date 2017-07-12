#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX_System-master/manage/')
import Ymap
m = Ymap.ymap()

#Setting Parameter
LOrange=[105, 105.1] #GHz 90.0 120.0
LOres=5.0 #GHz 5.0
Vrange=[7.0, 8.4] #mV 7.0 8.4
Vres=0.05 #mV 0.1
Irange=[10.0, 30.0] #% 10.0 30.0
Ires=1.0 #% 1.0
logpath='/home/amigos/data/SIS/'

def measure():
    m.makemap(LOrange, LOres, Vrange, Vres, Irange, Ires, logpath)

measure()
