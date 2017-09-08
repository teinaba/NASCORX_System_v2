#! /usr/bin/env python
# _*_ coding: UTF-8 _*_

#import modules
import time, sys
import pymeasure

class mg3692c(object):
    '''
    DESCRIPTION
    ================
    This class cntrols the MG3692C.

    ARGUMENTS
    ================
    1. IP: IP address of the Prologix
        Type: string
        Default: '192.168.100.1'
    2. GPIB: GPIB number of the MG3692C
        Type: int
        Default: 1
    '''

    def __init__(self, IP='192.168.100.1', GPIB=1):
        self.IP = IP
        self.GPIB = GPIB
        self.com = pymeasure.gpib_prologix(IP, GPIB)
        
    def set_freq(self, freq, unit='GHz'):
        """        
        DESCRIPTION
        ================
        This function sets the CW frequency.
        
        ARGUMENTS
        ================
        1. freq: CW frequency
            Number: 2.0-20.0 [GHz]
            Type: float
            Default: nothing
        2. unit: unit of the CW frequency 
            Number: 'GHz', 'MHZ', 'KHZ', 'HZ'
            Type: string
            Default: 'GHz'

        RETURNS
        ================
        Nothing.
        """
        self.com.open()
        self.com.send('FREQ:CW %.10f %s'%(freq, unit))
        self.com.close()
        return
    
    def query_freq(self):
        """        
        DESCRIPTION
        ================
        This function queries the CW frequency.
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. freq: current CW frequency
            Type: float [GHz]
        """
        self.com.open()
        self.com.send('FREQ:CW?')
        ret = self.com.readline()
        self.com.close()
        freq = float(ret)/1e+9
        return freq
        
    def set_power(self, power=-20.0):
        """        
        DESCRIPTION
        ================
        This function sets the CW power level.
        
        ARGUMENTS
        ================
        1. power: CW power level
            Number: -20.0-30.0 [dBm]
            Type: float
            Default: -20.0 [dBm]

        RETURNS
        ================
        Nothing.
        """
        if -20.0<=power<=30.0:
            self.com.open()
            self.com.send('POW %f dBm'%(power))
            self.com.close()
        else:
            print('!!!!ERROR!!!!')
            print('You set invalid power level.')
            print('-20.0 <= power <= +30.0.')
        return
    
    def query_power(self):
        """        
        DESCRIPTION
        ================
        This function queries the CW power level.
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. power: current CW power level
            Type: float [dBm]
        """
        self.com.open()
        self.com.send('POW?')
        ret = self.com.readline()
        self.com.close()
        power = float(ret)
        return power
    
    def set_output(self, onoff=0):
        """        
        DESCRIPTION
        ================
        This function switches the RF output.
        
        ARGUMENTS
        ================
        1. onoff: RF output ON/OFF
            Number: 1 or 0
            Type: int (1: ON, 0: OFF)
            Default: 0

        RETURNS
        ================
        Nothing.
        """
        self.com.open()
        if onoff==1:
            self.com.send('OUTP ON')
        else:
            self.com.send('OUTP OFF')
        self.com.close()
        return
        
    def query_output(self):
        """        
        DESCRIPTION
        ================
        This function queries the RF output status.
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. onoff: RF output ON/OFF
            Type: int (1: ON, 0: OFF)
        """
        self.com.open()
        self.com.send('OUTP?')
        ret = self.com.readline()
        self.com.close()
        ret = int(ret)
        return ret
  
#written by K.Urushihara
# 2017/08/29 T.Inaba: delete sys.path to pymeasure