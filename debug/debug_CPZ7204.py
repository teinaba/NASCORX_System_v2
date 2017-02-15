#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/device/')
import CPZ7204

mtr = CPZ7204.cpz7204(dev=1)

#mtr.set_home()

for i in range(10):
    mtr.rot_angle(speed=3000, angle=90)
    mtr.rot_angle(speed=3000, angle=-90)

#mtr.go_home()

