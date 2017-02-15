#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/manage/')
import Ymap

m = Ymap.ymap()

def debug():
    m.makemap(LOrange=[85, 120], LOres=15, Vrange=[6, 8], Vres=1, Irange=[50, 90], Ires=20)

debug()


