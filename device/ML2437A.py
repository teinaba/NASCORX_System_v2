#! /usr/bin/env python
# _*_ coding: UTF-8 _*_

import time, sys
import pymeasure

class ml2437a(object):
    '''
    DESCRIPTION
    ================
    This class cntrols the ML2437A.

    ARGUMENTS
    ================
    1. dev: device number
        Type: int
        Default: 1
    '''

    def __init__(self, IP='192.168.100.1', GPIB=1):
        self.IP = IP
        self.GPIB = GPIB

        
    def measure(self, ch=1, resolution=3):
        '''        
        DESCRIPTION
        ================
        This function queries the input power level.
        
        ARGUMENTS
        ================
        1. ch: the sensor channel number.
            Number: 1-2
            Type: int
            Default: 1
        2. resolution: the sensor order of the resolution.
            Number: 1-3
            Type: int
            Default: 3
            
        RETURNS
        ================
        1. power: the power value [dBm]
            Type: float
        '''
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        self.com.send('CHUNIT %d, DBM' %(ch))
        self.com.send('CHRES %d, %d' %(ch, resolution))
        self.com.send('o %d' %(ch))
        time.sleep(0.1)
        ret = self.com.readline()
        self.com.close()
        power = float(ret)
        return power

    def set_average_onoff(self, onoff, sensor='A'):
        '''        
        DESCRIPTION
        ================
        This function switches the averaging mode.
        
        ARGUMENTS
        ================
        1. onoff: averaging mode
            Number: 0 or 1
            Type: int
            Default: Nothing.
            
        2. sensor: averaging sensor.
            Number: 'A' or 'B'
            Type: string
            Default: 'A'
            
        RETURNS
        ================
        Nothing.
        '''
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        if onoff == 1:
            self.com.send('AVG %s, RPT, 60' %(sensor))
        else:
            self.com.send('AVG %s, OFF, 60' %(sensor))
        self.com.close()
        return
        
    def query_average_onoff(self):
        '''        
        DESCRIPTION
        ================
        This function queries the averaging mode.
        
        ARGUMENTS
        ================
        Nothing.
            
        RETURNS
        ================
        1. onoff: averaging mode
            Number: 0 or 1
            Type: int
        '''
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        self.com.send('STATUS')
        ret = self.com.readline()
        if ret[17] == '0':
            ret = 0
        else:
            ret = 1
        self.com.close()
        return ret
    
    def set_average_count(self, count, sensor='A'):
        '''        
        DESCRIPTION
        ================
        This function sets the averaging counts.
        
        ARGUMENTS
        ================
        1. count: averaging counts
            Type: int
            Default: Nothing.
            
        2. sensor: averaging sensor.
            Number: 'A' or 'B'
            Type: string
            Default: 'A'
            
        RETURNS
        ================
        Nothing.
        '''
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        self.com.send('AVG %s, RPT, %d' %(sensor, count))
        self.com.close()
        return
        
    def query_average_count(self):
        '''        
        DESCRIPTION
        ================
        This function queries the averaging counts.
        
        ARGUMENTS
        ================
        Nothing.
            
        RETURNS
        ================
        1. count: averaging counts
            Type: int
        '''
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        self.com.send('STATUS')
        ret = self.com.readline()
        count = int(ret[19:23])
        self.com.close()
        return count

#written by K.Urushihara
# 2017/08/29 T.Inaba: delete sys.path for pymeasure