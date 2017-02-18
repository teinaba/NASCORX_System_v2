#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import sys, time
sys.path.append('/home/amigos/NASCORX-master/device/')
import A11713C

class attenuation(object):
    '''
    DESCRIPTION
    ================
    This class controls the prog. att. in the room temp. I.F.

    ARGUMENTS
    ================
    1. device: name of the prog. att. registered in the IP_table
        Type: string
        Default: 'A11713C1'
    2. device_table: file path of the IP table
        Type: string
        Default: '/home/amigos/NASCORX-master/base/IP_table_115.txt'
    '''
    def __init__(self, device='A11713C1', device_table='/home/amigos/NASCORX-master/base/device_table_115.txt'):
        self.device = device
        self.device_table = device_table
        lan = self._ip_search_(device=self.device)
        self.IP = lan[1]
        self.port = lan[2]
        self.pa = A11713C.a11713c(self.IP, self.port)

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

    def set_att(self, X1, Y1, X2, Y2, model='AG8494g', voltage='24V'):
        '''        
        DESCRIPTION
        ================
        This function sets the attenuation level.
        
        ARGUMENTS
        ================
        1. X1: attenuation level of ch '1X' [dB]
            Number: 0 -  11[dB]
            Type: int
            Default: Nothing.
        2. Y1: attenuation level of ch '1Y' [dB]
            Number: 0 -  11[dB]
            Type: int
            Default: Nothing.
        3. X2: attenuation level of ch '2X' [dB]
            Number: 0 -  11[dB]
            Type: int
            Default: Nothing.
        4. Y2: attenuation level of ch '2Y' [dB]
            Number: 0 -  11[dB]
            Type: int
            Default: Nothing.
        5. model: model of the attenuator
            Number: 'NA', 'AG8494g', 'AG8495g', 'AG8495k',
                    'AG8496g', 'AG8497k', 'AG84904k', 'AG84905m',
                    'AG84906k', 'AG84907k' or 'AG84908m'
                    *for AG849xh, use 'AG849xg'
                    *for AG84904l/m, use 'AG84904k'
                    *for AG84906l, use 'AG84906k'
                    *for AG84907l, use 'AG84907k'
            Type: string
            Default: 'AG8494g'
        6. voltage: supply voltage
            Number: 'OFF', '5V', '15V', '24V' or 'USER'
            Type: string
            Default: '24V'

        RETURNS
        ================
        Nothing.
        '''
        att_model = self.pa.query_model()
        att_voltage = self.pa.query_voltage()
        if model not in att_model:
            self.pa.set_model(model, '1X')
            self.pa.set_model(model, '1Y')
            self.pa.set_model(model, '2X')
            self.pa.set_model(model, '2Y')
        if voltage not in att_voltage:
            self.pa.set_voltage(voltage, 1)
            self.pa.set_voltage(voltage, 2)
        self.pa.set_level(X1, '1X')
        self.pa.set_level(Y1, '1Y')
        self.pa.set_level(X2, '2X')
        self.pa.set_level(Y2, '2Y')
        return

    def query_status(self):
        '''        
        DESCRIPTION
        ================
        This function queries the status of the prog. att.
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. model: model of the attenuator
            Type: list [1X, 1Y, 2X, 2Y]
                *for AG849xh, use 'AG849xg'
                *for AG84904l/m, use 'AG84904k'
                *for AG84906l, use 'AG84906k'
                *for AG84907l, use 'AG84907k'
        2. voltage: supply voltage
            Type: string list ('OFF', 'P5', 'P15', 'P24' or 'USER')
        '''
        att_model = self.pa.query_model()
        att_voltage = self.pa.query_voltage()
        return att_model, att_voltage


#written by K.Urushihara
