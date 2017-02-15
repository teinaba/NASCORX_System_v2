#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import time, sys
sys.path.append('/home/amigos/pyinterface-master/')
import pyinterface

class cpz3177(object):
    '''
    DESCRIPTION
    ================
    This class cntrols the CPZ-3177.
    ////CPZ-3177 Specification////
    Function: A/D Converter
    Resolution: 12 bit
    CH number: 64 ch (single-ended)
    Voltage: -10 - +10 V
    Setting time: 60 usec

    ARGUMENTS
    ================
    1. dev: device number
        Type: int
        Default: 1
    '''

    def __init__(self, dev=1):
        self.dev = dev
        self.driver = pyinterface.gpg3100.gpg3100(ndev=self.dev)

    def set_mode(self, mode='single'):
        """        
        DESCRIPTION
        ================
        This function sets the input mode.
        
        ARGUMENTS
        ================
        1. mode: input mode
            Number: 'single' or 'diff'
            Type: string
            Default: 'single'
        
        RETURNS
        ================
        Nothing.
        """
        if mode=='single':
            self.driver.use_singleend()
        elif mode=='diff':
            self.driver.use_differential()
        else:
            print('!!!!ERROR!!!!')
            print('invalid mode: '+str(mode))
            print('available mode: "single" or "diff"')
        return

    def query_mode(self):
        """        
        DESCRIPTION
        ================
        This function queries the input mode.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. mode: input mode
            Type: string
        """
        ret = self.driver.read_single_diff()
        if ret=='AD_INPUT_SINGLE':
            mode = 'single'
        elif ret=='AD_INPUT_DIFF':
            mode = 'diff'
        else:
            mode = 'UNKNOWN'
        return mode

    def query_input(self):
        """        
        DESCRIPTION
        ================
        This function queries the A/D input voltage.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. voltage: A/D input voltage
            Type: list [V]
        """
        ret  = self.driver.input()
        voltage = ret
        return voltage

    def set_input_range(self, Vrange='AD_10V'):
        """        
        DESCRIPTION
        ================
        This function sets the A/D input range.
        
        ARGUMENTS
        ================
        1. Vrange: A/D input range
            Number: 'AD_0_5V', 'AD_0_10V', 'AD_2P5V', 'AD_5V' or 'AD_10V'
            Type: string
            Default: 'AD_10V'
        
        RETURNS
        ================
        Nothing.
        """
        Vlist = ['AD_0_5V', 'AD_0_10V', 'AD_2P5V', 'AD_5V', 'AD_10V']
        if Vrange in Vlist:
            self.driver.set_range(Vrange)
        else:
            print('!!!!ERROR!!!!')
            print('invalid range: '+str(Vrange))
            print('available mode: "AD_0_5V", "AD_0_10V", "AD_2P5V", "AD_5V" or "AD_10V"')
        return

    def query_input_range(self):
        """        
        DESCRIPTION
        ================
        This function queries the A/D input range.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. Vrange: A/D input range
            Type: string
        """
        ret  = self.driver.read_status_input_range()
        Vrange = str(ret[0])
        return Vrange


#written by K.Urushihara
