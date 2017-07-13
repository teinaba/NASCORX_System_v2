#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import time, sys
sys.path.append('/home/amigos/pyinterface-master/')
import pyinterface

class cpz340816(object):
    '''
    DESCRIPTION
    ================
    This class controls the CPZ-340816.
    ////CPZ-340816 Specification////
    Function: D/A Converter
    Resolution: 16 bit
    CH number: 16 ch
    Voltage: -10 - +10 V
    Current: 0 - 5 mA
    Setting time: 10 usec

    ARGUMENTS
    ================
    1. dev: device number
        Type: int
        Default: 1
    '''

    def __init__(self, dev=1):
        self.dev = dev
        self.driver = pyinterface.gpg3300.gpg3300(ndev=self.dev)

    def set_voltage(self, voltage=0, ch=None):
        """        
        DESCRIPTION
        ================
        This function sets the output voltage.
        
        ARGUMENTS
        ================
        1. voltage: output voltage
            Number: -10-10 [V]
            Type: float
            Default: 0
        2. ch: setting channel number
            Number: 0-15
            Type: int
            Default: None (setting all ch at the same time)
        
        RETURNS
        ================
        Nothing.
        """
        if abs(voltage)<=10.0:
            if 0<=ch<=15 or ch==None:
                self.driver.set_da_value(value=float(voltage), ch=ch)
            else:
                print('!!!!ERROR!!!!')
                print('invalid ch: '+str(ch))
        else:
            print('!!!!ERROR!!!!')
            print('invalid voltage: '+str(voltage))
        return

    def query_voltage(self):
        """        
        DESCRIPTION
        ================
        This function queries the output voltage.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. voltage: output voltage [V]
            Type: list
        """
        ret = self.driver.read_status_output_value()
        return ret

    def set_output(self, onoff=0):
        """        
        DESCRIPTION
        ================
        This function switches the D/A output.
        
        ARGUMENTS
        ================
        1. onoff: D/A output
            Number: 1 or 0
            Type: int (1: ON, 0: OFF)
            Default: 0
        
        RETURNS
        ================
        Nothing.
        """
        if onoff==1:
            self.driver.output()
        elif onoff==0:
            self.driver.stop_output()
        else:
            print('!!!!ERROR!!!!')
            print('invalid argument: '+str(onoff))
            print('this argument must be 1 or 0')
        return

    def query_output(self):
        """        
        DESCRIPTION
        ================
        This function queries the D/A output status.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. onoff: D/A output status
            Type: int (1: ON, 0: OFF)
        """
        ret  = self.driver.read_status_output()
        if ret==True:
            onoff = 1
        else:
            onoff = 0
        return onoff

    def close_board(self):
        """        
        DESCRIPTION
        ================
        This function close the board connection.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        Nothing.
        """
        self.driver.stop_output()
        self.driver.close()
        return

#written by K.Urushihara
