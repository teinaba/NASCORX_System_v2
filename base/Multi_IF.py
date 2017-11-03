#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


# import modules
import NASCORX_System.device.A11713B as A11713B
import NASCORX_System.device.A11713C as A11713C


class prog_att(object):
    """
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
        Default: '/home/amigos/NASCORX_System/base/IP_table_115.txt'
    """
    def __init__(self, device='A11713C1',
                 device_table='/home/amigos/NASCORX_System-master/NASCORX_System/base/device_table_115.txt'):
        self.device = device
        self.device_table = device_table
        lan = self._ip_search_(device=self.device)
        self.IP = lan[1]
        self.port = lan[2]
        self.pa = A11713C.a11713c(IP=self.IP, port=self.port)

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
        """
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
        """
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
        """
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
        """
        att_model = self.pa.query_model()
        att_voltage = self.pa.query_voltage()
        return att_model, att_voltage


class multi_prog_att(object):
    ver = '2017.09.27'
    config = 'A11713B x6'

    def __init__(self, device1='A11713B1', device2='A11713B2', device3='A11713B3',
                 device4='A11713B4', device5='A11713B5', device6='A11713B6',
                 device_table='/home/amigos/NASCORX_System-master/NASCORX_System/base/device_table_115.txt'):
        self.device_table = device_table

        # import driver control module
        self.driver1 = self.set_driver(device=device1)
        self.driver2 = self.set_driver(device=device2)
        self.driver3 = self.set_driver(device=device3)
        self.driver4 = self.set_driver(device=device4)
        self.driver5 = self.set_driver(device=device5)
        self.driver6 = self.set_driver(device=device6)
        return

    def _ip_search_(self, device):
        f = open(self.device_table, 'r')
        for line in f:
            dev = line.strip().split(',')
            if device in dev:
                info1 = str(dev[1].strip())
                info2 = int(dev[2].strip())
                info3 = str(dev[3].strip())
                break
            else:
                pass
        f.close()
        ret = [device, info1, info2, info3]
        return ret

    def set_driver(self, device):
        """
        DESCRIPTION
        ================
        This function returns the attenuation driver.

        ARGUMENTS
        ================
        1. device1-6: device name described in device table
            Number: "A11713Bn" (n=1,2,3,...,6)
            Type: string
            Default: 6 devices noted above

        RETURNS
        ================
        1. driver: attenuation driver
            Type: Instance
        """

        connect = self._ip_search_(device=device)
        IP = connect[1]
        port = connect[2]
        connection = connect[3]
        driver = A11713B.a11713b(IP=IP, port=port, connection=connection)
        return driver

    def get_configuration(self):
        return self.ver, self.config

    def communication_check(self):
        ret1 = self.driver1.qury_IDN()
        ret2 = self.driver2.qury_IDN()
        ret3 = self.driver3.qury_IDN()
        ret4 = self.driver4.qury_IDN()
        ret5 = self.driver5.qury_IDN()
        ret6 = self.driver6.qury_IDN()
        return[ret1, ret2, ret3, ret4, ret5, ret6]

    def set_attenuation(self, att=[11]*12):
        """
        DESCRIPTION
        ================
        This function sets the attenuation level for all Prog. Att.

        ARGUMENTS
        ================
        1. att: attenuation level of each prog. Att
            Number : 0, 1, 2, ..., 11
            Type   : int list
            Length : 12
            Default: [11] * 12

        RETURNS
        ================
        Nothing.
        """
        self.driver1.set_level(level=att[0], ch='1X')
        self.driver1.set_level(level=att[1], ch='1Y')
        self.driver2.set_level(level=att[2], ch='1X')
        self.driver2.set_level(level=att[3], ch='1Y')
        self.driver3.set_level(level=att[4], ch='1X')
        self.driver3.set_level(level=att[5], ch='1Y')
        self.driver4.set_level(level=att[6], ch='1X')
        self.driver4.set_level(level=att[7], ch='1Y')
        self.driver5.set_level(level=att[8], ch='1X')
        self.driver5.set_level(level=att[9], ch='1Y')
        self.driver6.set_level(level=att[10], ch='1X')
        self.driver6.set_level(level=att[11], ch='1Y')
        return

    def query_attenuation(self):
        """
        DESCRIPTION
        ================
        This function queries the attenuation level for all Prog. Att.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. attenuation: attenuation level
            Type  : list [1-1X, 1-1Y, 2-1X, 2-1Y, 3-1X, 3-1Y, 4-1X, 4-1Y, 5-1X, 5-1Y, 6-1X, 6-1Y]
            Length: 12
        """
        ret1 = self.driver1.query_level()
        ret2 = self.driver2.query_level()
        ret3 = self.driver3.query_level()
        ret4 = self.driver4.query_level()
        ret5 = self.driver5.query_level()
        ret6 = self.driver6.query_level()
        attenuation = ret1+ret2+ret3+ret4+ret5+ret6
        return attenuation


class multi_switch(object):
    ver = '2017.09.27'
    config = 'A11713C x1'

    def __init__(self, device='A11713C1',
                 device_table='/home/amigos/NASCORX_System-master/NASCORX_System/base/device_table_115.txt'):
        self.device_table = device_table
        self.driver = self.set_driver(device=device)
        return

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

    def set_driver(self, device):
        """
        DESCRIPTION
        ================
        This function returns the attenuation driver.

        ARGUMENTS
        ================
        1. device: device name described in device table
            Number: "A11713Cn" (n=integer)
            Type: string
            Default: "A11713C1".

        RETURNS
        ================
        1. driver: attenuation driver
            Type: Instance
        """
        connect = self._ip_search_(device=device)
        IP = connect[0]
        port = connect[1]
        driver = A11713C.a11713c(IP=IP, port=port)
        return driver

    def get_configuration(self):
        return self.ver, self.config

    def communication_check(self):
        ret = self.driver.query_IDN()
        return ret

    def switch(self, command):
        """
        DESCRIPTION
        ================
        This function switches IF switch CLOSE and OPEN following the command.

        ARGUMENTS
        ================
        1. command
            Number : "onoff-ch" (0=CLOSE, 1=OPEN. ch="(@bnn)")
            Type   : str List
            Default: Nothing.
            Example: "1-(@104)", "0-(@104, 107, 201, 206)", "0-(@101:109)"
            comment: See the Official Manual of Agilent 11713B/C

        RETURNS
        ================
        Nothing.
        """
        for string in command:
            temp = string.strip().split('-')
            onoff = temp[0].strip()
            ch = temp[1].strip()
            self.driver.switch(onoff=onoff, ch=ch)
        return

    def switch_all_ch_close(self):
        """
        DESCRIPTION
        ================
        This function Switched all IF switches CLOSE.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.driver.switch_all_ch_close()
        return

    def switch_all_ch_open(self):
        """
        DESCRIPTION
        ================
        This function Switched all IF switches OPEN.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.driver.switch_all_ch_open()
        return


class multi_IF(object):

    def __init__(self):
        self.patt_ctrl = multi_prog_att()
        self.switch_ctrl = multi_switch()
        pass


# History
# -------
# written by T.Inaba
# 2017/11/03 T.Inaba: modify set_driver.
