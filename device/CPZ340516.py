#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


# import modules
import pyinterface

class cpz340516(object):
    """
    DESCRIPTION
    ================
    This class controls the CPZ-340516.
    ////CPZ-340516 Specification////
    Function: D/A Converter
    Resolution: 16 bit
    CH number: 8 ch
    Voltage: 0 - 10 V (Current Control)
    Current: 0 - 100 mA
    Setting time: 100 usec

    ARGUMENTS
    ================
    1. dev: device number
        Type: int
        Default: 1
    """

    def __init__(self, dev=1):
        self.driver = pyinterface.gpg3300.gpg3300(ndev=dev)

    def set_Irange(self, mode='DA_0_100mA', ch=None):
        """        
        DESCRIPTION
        ================
        This function sets the range of output current.
        
        ARGUMENTS
        ================
        1. mode: range of output current
            Number: 'DA_0_100mA' or 'DA_0_1mA'
            Type: string
            Default: 'DA_0_100mA'
        2. ch: setting channel number
            Number: 0-7
            Type: int
            Default: None (setting all ch at the same time)
        
        RETURNS
        ================
        Nothing.
        """
        if mode == 'DA_0_1mA':
            if ch is None or 0 <= ch <= 7:
                self.driver.set_range(da_range=mode, ch=ch)
            else:
                print('!!!!ERROR!!!!')
                print('invalid ch: {0}'.format(ch))
                print('available ch: 0, 1, ... , 7 or None')
        elif mode == 'DA_0_100mA':
            if ch is None or 0 <= ch <= 7:
                self.driver.set_range(da_range=mode, ch=ch)
            else:
                print('!!!!ERROR!!!!')
                print('invalid ch: {0}'.format(ch))
                print('available ch: 0, 1, ... , 7 or None')
        else:
            print('!!!!ERROR!!!!')
            print('invalid mode: {0}'.format(mode))
            print('mode = "DA_0_1mA" or "DA_0_100mA"')
        return

    def query_Irange(self):
        """        
        DESCRIPTION
        ================
        This function queries the range of output current.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. mode: range of output current
            Type: list
        """
        ret = self.driver.read_status_output_range()
        mode = []
        for i in range(len(ret)):
            mode.append(str(ret[i]))
        return mode

    def set_current(self, current=0, ch=None):
        """        
        DESCRIPTION
        ================
        This function sets the output current.
        
        ARGUMENTS
        ================
        1. current: output current
            Number: 0-0.1 [A]
            Type: float
            Default: 0
        2. ch: setting channel number
            Number: 0-7
            Type: int
            Default: None (setting all ch at the same time)
        
        RETURNS
        ================
        Nothing.
        """
        Irange = self.query_Irange()
        if 0 <= ch <= 7:
            if Irange[ch] == 'DA_0_1mA':
                if 0 <= current <= 0.001:
                    self.driver.set_da_value(value=float(current), ch=ch)
                else:
                    print('!!!!ERROR!!!!')
                    print('invalid current: {0}'.format(current))
                    print('available current: 0-0.001 [A]')
            else:
                if 0 <= current <= 0.1:
                    self.driver.set_da_value(value=float(current), ch=ch)
                else:
                    print('!!!!ERROR!!!!')
                    print('invalid current: {0}'.format(current))
                    print('available current: 0-0.1 [A]')
        elif ch is None:
            if 'DA_0_1_mA' in Irange:
                if 0 <= current <= 0.001:
                    self.driver.set_da_value(value=float(current), ch=ch)
                else:
                    print('!!!!ERROR!!!!')
                    print('invalid current: {0}'.format(current))
                    print('available current: 0-0.001 [A]')
            else:
                if 0 <= current <= 0.1:
                    self.driver.set_da_value(value=float(current), ch=ch)
                else:
                    print('!!!!ERROR!!!!')
                    print('invalid current: {0}'.format(current))
                    print('available current: 0-0.1 [A]')
        else:
            print('!!!!ERROR!!!!')
            print('invalid ch: {0}'.format(ch))
            print('available ch: 0, 1, ... , 7 or None')
        return

    def query_current(self):
        """        
        DESCRIPTION
        ================
        This function queries the output current.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. current: output current
            Type: list
        """
        current = self.driver.read_status_output_value()
        return current

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
        if onoff == 1:
            self.driver.output()
        elif onoff == 0:
            self.driver.stop_output()
        else:
            print('!!!!ERROR!!!!')
            print('invalid argument: {0}'.format(onoff))
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
        ret = self.driver.read_status_output()
        if ret is True:
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

# written by K.Urushihara
# 2017/08/04 T.Inaba: debugged
