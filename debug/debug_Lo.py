#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
sys.path.append('/home/amigos/NASCORX-master/base/')
import Lo

#l = Lo.firstlo()
#print(l.__dict__)

#s = Lo.secondlo()
#print(s.__dict__)

t = Lo.secondlo(device='FSW00202', IP_table='/home/amigos/NASCORX-master/base/IP_table_230.txt')
print(t.__dict__)

def debug_1st():
    l.start_osci(115.0, 1.5)
    ret1 = l.query_status()
    print(ret1)
    l.change_freq(125.0)
    l.change_power(-30.0)
    ret2 = l.query_status()
    print(ret2)
    l.end_osci()
    ret3 = l.query_status()
    print(ret3)

def debug_2nd1():
    s.start_osci(5.0, 1.5)
    ret1 = s.query_status()
    print(ret1)
    s.change_freq(2.0)
    s.change_power(-50.0)
    ret2 = s.query_status()
    print(ret2)
    s.end_osci()
    ret3 = s.query_status()
    print(ret3)

def debug_2nd2():
    t.start_osci(5.0, 1.5)
    time.sleep(0.1)
    ret1 = t.query_status()
    print(ret1)
    t.change_freq(2.0)
    t.change_power(-50.0)
    time.sleep(0.1)
    ret2 = t.query_status()
    print(ret2)
    t.end_osci()
    time.sleep(0.1)
    ret3 = t.query_status()
    print(ret3)


#debug_1st()
#debug_2nd1()
debug_2nd2()

