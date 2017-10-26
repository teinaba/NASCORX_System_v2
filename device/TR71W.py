#! /usr/bin/env python
# _*_ coding: UTF-8 _*_

# import modules
import urllib.request


class tr71w(object):
    """
    DESCRIPTION
    ================
    This class controls the thermometer TR-71W.

    ARGUMENTS
    ================
    1. IP: IP address of the TR-71W
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
        raw_T1 = raw_data[5].split('=')
        raw_T2 = raw_data[6].split('=')
        if raw_T1[1] != '----':
            temp1 = float(raw_T1[1])
        else:
            temp1 = 0
        if raw_T2[1] != '----':
            temp2 = float(raw_T2[1])
        else:
            temp2 = 0
        return temp1, temp2


# History
# -------
# written by K.Urushihara
# 2017.10.23 T.Inaba : adapted to python3
