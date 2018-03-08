#!/usr/bin/env python

# import modules
import sys
import time
import numpy
import matplotlib.pyplot
import NASCORX_System.device.L218 as L218
import NASCORX_System.device.TR71W as TR71W
import NASCORX_System.device.TR72W as TR72W
import NASCORX_System.device.TPG261 as TPG261
import NASCORX_System.base.Multi_Cryo as Multi_Cryo


class monitor_SIS(object):
    method = 'Monitor SIS parameters'
    ver = '2017.10.09'
    savedir = '/home/amigos/NASCORX_Measurement/SIStune/'

    def __init__(self):
        self.box = Multi_Cryo.multi_box()
        pass

    def get_bias(self):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        ad_sis = self.box.monior_sis()
        ad_hemt = self.box.monitor_hemt()
        ad_bias = ad_sis[0:24]+ad_hemt[0:48]
        ad_bias.insert(0, timestamp)
        return ad_bias


class monitor_conditions(object):
    method = 'Monitor Measurement-System Conditions'
    ver = '2017.10.05'
    device_table = '/home/amigos/NASCORX_System-master/NASCORX_System/base/device_table_115.txt'
    savedir = '/home/amigos/NASCORX_Measurement/SIStune/'

    def __init__(self):
        pass

    def initialize(self):
        return

    def _ip_search(self, device):
        f = open(self.device_table, 'r')
        for line in f:
            dev = line.strip().split(',')
            if device in dev:
                info1 = int(dev[1].strip())
                info2 = int(dev[2].strip())
                break
            else:
                pass
        f.close()
        ret = [device, info1, info2]
        return ret

    # Single Measure func
    # -------------------

    def get_4k_temp(self, device='L218'):
        conn = self._ip_search(device=device)
        ls = L218.l218(IP=conn[0], GPIB=conn[1])
        ret = ls.measure()
        return ret

    def get_cabin_temp1(self, device='TR71W'):
        conn = self._ip_search(device=device)
        ondo = TR71W.tr71w(IP=conn[0])
        ret = ondo.temp()
        return ret

    def get_cabin_temp2(self, device='TR72W'):
        conn = self._ip_search(device=device)
        ondo = TR72W.tr72w(IP=conn[0])
        ret = ondo.temp()
        return ret

    def get_vacuum(self, device='TPG261'):
        conn = self._ip_search(device=device)
        vacuum = TPG261.tpg261(IP=conn[0], GPIB=conn[1])
        ret = vacuum.query_pressure()
        return ret

    # Monitor Loop func
    # -----------------

    def monitor_cabin_temp(self, dev1='TR71W', dev2='TR72W'):
        # connect
        # -------
        conn1 = self._ip_search(device=dev1)
        conn2 = self._ip_search(device=dev2)
        ondo1 = TR71W.tr71w(IP=conn1[0])
        ondo2 = TR71W.tr71w(IP=conn2[0])

        while True:
            timestamp = time.strftime('%Y-%m-%d_%H%:%M:%S')
            temp1 = ondo1.temp()
            temp2 = ondo2.measure()
            print('Timestamp: {},   CH1: {} [K],   CH2: {} [K],   CH3: {} [K],   CH4: {} [%]'.
                  format(timestamp, temp1[0], temp1[1], temp2[0], temp2[1]))

            time.sleep(50)
            pass
        return

    def monitor_vacuum_loop(self, dev='TPG261', unit='torr'):
        # connect
        # -------
        conn = self._ip_search(device=dev)
        vacuum = TPG261.tpg261(IP=conn[0], GPIB=conn[1])

        # unit set
        # --------
        vacuum.set_unit(unit=unit)
        if unit == 'pascal': unit = 'Pa'

        while True:
            timestamp = time.strftime('%Y-%m-%d_%H%:%M:%S')
            press = vacuum.query_pressure()
            print('Timestamp: {},   Pressure: {} [{}]'.format(timestamp, press, unit))

            time.sleep(50)
            pass
        return

    def monitor_4k_loop(self, dev='L218'):
        # connect
        # -------
        conn = self._ip_search(device=dev)
        ls = L218.l218(IP=conn[0], GPIB=conn[1])

        while True:
            timestamp = time.strftime('%Y-%m-%d_%H%:%M:%S')
            temp = ls.measure()
            print('Timestamp: {0},  CH1: {1[0]},  CH2: {1[1]},  CH3: {1[2]},  CH4: {1[3]},  '
                  'CH5: {1[4]},  CH6: {1[5]},  CH7: {1[6]},  CH8: {1[7]}'.format(timestamp, temp))

            time.sleep(50)
        return


# History
# -------
# written by T.Inaba
