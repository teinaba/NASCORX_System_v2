#! /usr/bin/env python
# _*_ coding: UTF-8 _*_

# import modules
import urllib.request


class tr72w(object):
    """
    DESCRIPTION
    ================
    This class controls the thermometer TR-72W.

    ARGUMENTS
    ================
    1. IP: IP address of the TR-72W
        Type: string
        Default: '192.168.100.1'
    """

    def __init__(self, IP='192.168.100.1'):
        self.IP = IP
        self.url = 'http://'+self.IP+'/B/crrntdata/cdata.txt'
        
    def temp(self):
        """
        DESCRIPTION
        ================
        This function queries the room temperature.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. The room temperature [K]: float type
        """
        res = urllib.request.urlopen(self.url)
        page = res.read()
        decoded_page = page.decode('shift_jis')
        raw_data = decoded_page.split('\r\n')
        raw_T = raw_data[5].split('=')
        temperature = float(raw_T[1])
        return temperature

    def hum(self):
        """
        DESCRIPTION
        ================
        This function queries the room humidity.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. The room humidity [%]: float type
        """
        res = urllib.request.urlopen(self.url)
        page = res.read()
        decoded_page = page.decode('shift_jis')
        raw_data = decoded_page.split('\r\n')
        raw_H = raw_data[6].split('=')
        humidity = float(raw_H[1])
        return humidity

    def measure(self):
        """
        DESCRIPTION
        ================
        This function queries the room temperature and humidity both.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. A list containing 2 values.
            1. The room temperature [K]: float
            2. The room humidity [%]: float
        """
        res = urllib.request.urlopen(self.url)
        page = res.read()
        decoded_page = page.decode('shift_jis')
        raw_data = decoded_page.split('\r\n')
        raw_T1 = raw_data[5].split('=')
        raw_T2 = raw_data[6].split('=')
        temp = raw_T1
        hum = raw_T2
        return [temp, hum]


# History
# -------
# written by K.Urushihara
# 2017.10.23 T.Inaba : adapted to python3
