#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import time, sys
sys.path.append('/home/amigos/pyinterface-master/')
import pyinterface

class cpz7204(object):
    '''
    DESCRIPTION
    ================
    This class cntrols the CPZ-7204.
    ////CPZ-7204 Specification////
    Function: Pulse Motion Controller
    Axises: 2
    Maximum Pulse Rate: 66.67 kpps

    ARGUMENTS
    ================
    1. dev: device number
        Type: int
        Default: 1
    '''

    def __init__(self, dev=1):
        self.dev = dev
        self.driver = pyinterface.gpg7204.gpg7204(ndev=self.dev)

    def query_position(self):
        """        
        DESCRIPTION
        ================
        This function queries the pulse counts.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. cnt: pulse counts
            Type: int
        """
        cnt = self.driver.get_position()
        return cnt

    def set_home(self):
        """        
        DESCRIPTION
        ================
        This function sets the home position.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        Nothing.
        """
        self.driver.set_org()
        return

    def go_home(self, speed=1000):
        """        
        DESCRIPTION
        ================
        This function moves to the home position.
        
        ARGUMENTS
        ================
        1. speed: rotate speed
            Type: int
            Default: 1000
        
        RETURNS
        ================
        Nothing.
        """
        init_pos = self.query_position()
        spd = abs(int(speed))
        cnt = int(init_pos*-1)
        self.driver.move(speed=spd, count=cnt)
        while 1:
            time.sleep(0.5)
            crrt_pos = self.query_position()
            if crrt_pos == 0:
                 break
            else:
                continue         
        return

    def rot_angle(self, speed=1000, angle=90):
        """        
        DESCRIPTION
        ================
        This function rotates the moter.
        
        ARGUMENTS
        ================
        1. speed: rotate speed
            Type: int
            Default: 1000
        
        1. angle: rotate angle [deg]
            Type: float
            Default: 90

        RETURNS
        ================
        Nothing.
        """
        init_pos = self.query_position()
        spd = abs(int(speed))
        cnt = int(float(angle)*10)
        tgt_pos = init_pos + cnt
        self.driver.move(speed=spd, count=cnt)
        while 1:
            time.sleep(0.5)
            crrt_pos = self.query_position()
            if crrt_pos == tgt_pos:
                 break
            else:
                continue
        return


#written by K.Urushihara
