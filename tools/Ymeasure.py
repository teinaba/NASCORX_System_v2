#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX_System-master/manage/')
import Ymap

m = Ymap.ymap()

def measure():
    m.makemap(LOrange=[115.0, 116.0], LOres=5.0, Vrange=[7.0, 8.4], Vres=0.1, Irange=[10.0, 30.0], Ires=10.0)

measure()


