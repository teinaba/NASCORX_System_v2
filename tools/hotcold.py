#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import sys, time, signal
sys.path.append('/home/amigos/NASCORX-master/device/')
import CPZ7204


def chopper():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    load = CPZ7204.cpz7204()
    load.set_home() # define HOT position
    print('Current position is defined as the HOT position.')
    cnt = 1
    while 1:
        if cnt == 1:
            print('Enter --> COLD')
            ret = raw_input()    
            load.rot_angle(speed=3000, angle=-90) # COLD
            cnt = 0
        else:
            print('Enter --> HOT')
            ret = raw_input()    
            load.go_home(speed=3000) # HOT
            cnt = 1
    return

def fineturn():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    load = CPZ7204.cpz7204()
    while 1:
        print('Input Angle')
        ret = raw_input()    
        angle = float(ret)
        load.rot_angle(speed=1000, angle=angle)
    return

if __name__ == '__main__':
    chopper()

# written by K.Urushihara

