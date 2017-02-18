#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import sys, time, datetime
sys.path.append('/home/amigos/NASCORX-master/device/')
import TR71W, TR72W, TPG261, L218

class monitor(object):
    '''
    DESCRIPTION
    ================
    This class monitors the Rx conditions.

    ARGUMENTS
    ================
    1. rt: name of the room temp. sensor registered in the IP_table
        Type: string
        Default: 'TR71W'
    2. vc: name of the vacuum sensor registered in the IP_table
        Type: string
        Default: 'TPG261'
    3. ct: name of the cryostat temp. sensor registered in the IP_table
        Type: string
        Default: 'L218'
    4. device_table: file path of the IP table
        Type: string
        Default: '/home/amigos/NASCORX-master/base/IP_table_115.txt'
    '''
    def __init__(self, rt='TR71W', vc='TPG261', ct='L218', device_table='/home/amigos/NASCORX-master/base/device_table_115.txt'):
        self.device1 = rt
        self.device2 = vc
        self.device3 = ct
        self.device_table = device_table
        lan1 = self._ip_search_(device=self.device1)
        self.IP1 = lan1[1]
        self.port1 = lan1[2]
        lan2 = self._ip_search_(device=self.device2)
        self.IP2 = lan2[1]
        self.port2 = lan2[2]
        lan3 = self._ip_search_(device=self.device3)
        self.IP3 = lan3[1]
        self.port3 = lan3[2]
        self.rtemp = TR71W.tr71w(self.IP1)
        self.vacuum = TPG261.tpg261(self.IP2, self.port2)
        self.ctemp = L218.l218(self.IP3, self.port3)

    def _ip_search_(self, device):
        f = open(self.device_table, 'r')
        for line in f:
            dev = line.strip().split(',')
            if device in dev:
                info1 = str(dev[1].strip())
                info2 = int(dev[2].strip())
                break
            else:
                pass
        f.close()
        ret = [device, info1, info2]
        return ret

    def measure(self):
        '''        
        DESCRIPTION
        ================
        This function measures the room temperature, dewar pressure and dewar temperature.
        
        ARGUMENTS
        ================

        RETURNS
        ================
        1. ut: universal time
            Type: string ('YYYY-MM-DD-hh-mm-ss')
        2. temp1: room temperature 1 [deg C]
            Type: float
        3. temp2: room temperature 2 [deg C]
            Type: float
        4. press: dewar pressure [torr]
            Type: float
        5. stage: dewar temperature [K]
            Type: float
        '''
        t = datetime.datetime.now()
        ut = t.strftime('%Y-%m-%d-%H-%M-%S')
        roomtemp = self.rtemp.temp()
        temp1 = roomtemp[0]
        temp2 = roomtemp[1]
        press = self.vacuum.query_pressure()
        stage = self.ctemp.measure()
        return [ut, temp1, temp2, press, stage]


#written by K.Urushihara
