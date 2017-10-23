#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import pymeasure

class l218(object):
    """
    DESCRIPTION
    ================
    This class controls the Lakeshore model218.

    ARGUMENTS
    ================
    1. IP: IP address of the Prologix
        Type: string
        Default: '192.168.100.1'
    2. GPIB: GPIB address of the model218
        Number: 0-30
        Type: string
        Default: 1
    """

    def __init__(self, IP='192.168.101.78', GPIB=1):
        self.IP = IP
        self.GPIB = GPIB
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)

    def measure(self):
        """
        DESCRIPTION
        ================
        This function queries the temperature.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. The temperature [K]
             Type   : float list
             Length : 8
        """
        self.com.open()
        self.com.send('KRDG?')
        raw = self.com.readline()
        ret = raw.strip().split(',')
        temp = list(map(float, ret))
        self.com.close()
        return temp

    def measure_ch(self, ch=1):
        """
        DESCRIPTION
        ================
        This function queries the temperature.

        ARGUMENTS
        ================
        1. ch: a sensor channel
            Number: 1-8
            Type: int
            Default: 1

        RETURNS
        ================
        1. The temperature [K]: float
        """
        if 1 <= ch <= 8:
            self.com.open()
            self.com.send('KRDG? %d'%(ch))
            ret = self.com.readline()
            self.com.close()
            print(ret)
            temp = float(ret)
        else:
            print('***********************')
            print('!!!!WRONG ch NUMBER!!!!')
            print('SELECT ch from 1 to 8')
            print('***********************')
            quit()
        return temp

#written by K.Urushihara
# 2017/08/29 T.Inaba: delete sys.path to pymeasure
# 2017/10/23 T.Inaba: add measure method.