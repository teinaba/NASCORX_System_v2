#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


# import modules
import sys, time, numpy
sys.path.append('/home/amigos/pymeasure2-master/')
import pymeasure

sys.path.append('/home/amigos/NASCORX_System-master/device/')
import E8257D, MG3692C, FSW_0020


class firstlo(object):
    """
    DESCRIPTION
    ================
    This class controls the 1st Lo.

    ARGUMENTS
    ================
    1. multiplier: multiplier of the Lo chain
        Number: 1, 2, 3, ...
        Type: int
        Default: 6
    2. device: name of the 1st SG registered in the device_table
        Type: string
        Default: 'E8257D'
    3. device_table: file path of the IP table
        Type: string
        Default: '/home/amigos/NASCORX-master/base/device_table_115.txt'
    """

    def __init__(self, multiplier=6, device='E8257D',
                 device_table='/home/amigos/NASCORX_System-master/base/device_table_115.txt'):
        self.multiplier = int(multiplier)
        self.device = device
        self.device_table = device_table
        lan = self._ip_search_(device=self.device)
        self.IP = lan[1]
        self.GPIB = lan[2]
        self.sg = E8257D.e8257d(self.IP, self.GPIB)

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

    def start_osci(self, freq, power):
        """
        DESCRIPTION
        ================
        This function starts the first Lo oscillation.

        ARGUMENTS
        ================
        1. freq: first Lo frequency (multiplied) [GHz]
            Number: (10MHz-20GHz)*multiplier
            Type: float
            Default: Nothing.
        2. power: first Lo power
            Number: -20 - +24 [dBm]
            Type: float
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.end_osci()
        self.sg.set_power(0.0)
        self.change_freq(freq)
        self.sg.set_output(1)
        self.change_power(power)
        return

    def change_freq(self, freq):
        """
        DESCRIPTION
        ================
        This function changes the frequency of the first Lo.

        ARGUMENTS
        ================
        1. freq: first Lo frequency (multiplied)
            Number: (10MHz-20GHz)*multiplier [GHz]
            Type: float
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        RF = float(freq) / float(self.multiplier)
        if 0.01 <= RF <= 20.0:
            self.sg.set_freq(RF, 'GHz')
        else:
            print('!!!!ERROR!!!!')
            print('invalid freq: '.format(freq))
            print('available freq: '.format(0.01 * self.multiplier) + ' - '.format(20.0 * self.multiplier) + ' [GHz]')
        return

    def change_power(self, power):
        """
        DESCRIPTION
        ================
        This function changes the RF power continuously.

        ARGUMENTS
        ================
        1. power: first Lo power
            Number: -20 - +24 [dBm]
            Type: float
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        current = self.query_status()
        res = 0.5
        if -20.0 <= power <= 24.0:
            if current[1] <= power:
                seq = current[1]
                while (power - res) > seq:
                    seq = round(seq + res, 3)
                    self.sg.set_power(power=seq)
                    time.sleep(0.01)
                self.sg.set_power(power=power)
            elif current[1] > power:
                seq = current[1]
                while (power + res) < seq:
                    seq = round(seq - res, 3)
                    self.sg.set_power(power=seq)
                    time.sleep(0.01)
                self.sg.set_power(power=power)
            else:
                pass
        else:
            print('!!!!ERROR!!!!')
            print('invalid power: '.format(power))
            print('available power: -20 - +24 [dBm]')
        return

    def end_osci(self):
        """
        DESCRIPTION
        ================
        This function stops the 1st Lo oscillation.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        out = self.query_status()
        if out[2] == 1:
            if out[1] < 0.0:
                self.sg.set_output(0)
            else:
                self.change_power(0.0)
                self.sg.set_output(0)
        elif out[2] == 0:
            pass
        else:
            print('!!!WARNING!!! OUTPUT STATUS UNDEFINED')
        return

    def query_status(self):
        """
        DESCRIPTION
        ================
        This function queries status of the 1st Lo.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. freq: frequency of the 1st Lo.
            Type: float [GHz]
        2. power: power of the 1st Lo.
            Type: float [dBm]
        3. output: output status of the 1st Lo.
            Type: int (0:off, 1:on)
        """
        freq = self.sg.query_freq() * self.multiplier
        power = self.sg.query_power()
        output = self.sg.query_output()
        return [freq, power, output]


class secondlo(object):
    """
    DESCRIPTION
    ================
    This class controls the 2nd Lo.

    ARGUMENTS
    ================
    2. device: name of the 2nd SG registered in the device_table
        Type: string
        Default: 'MG3692C1'
    1. device_table: file path of the IP table
        Type: string
        Default: '/home/amigos/NASCORX-master/base/device_table_115.txt'
    """

    def __init__(self, device='MG3692C1',
                 device_table='/home/amigos/NASCORX-master/base/device_table_115.txt'):
        self.device = device
        self.device_table = device_table
        lan = self._ip_search_(device=self.device)
        if self.device in ['MG3692C1', 'MG3692C2']:
            self.IP = lan[1]
            self.GPIB = lan[2]
            self.sg = MG3692C.mg3692c(IP=self.IP, GPIB=self.GPIB)
        elif self.device in ['FSW00201', 'FSW00202']:
            self.IP = lan[1]
            self.port = lan[2]
            self.sg = FSW_0020.fsw_0020(IP=self.IP, port=self.port)
        else:
            print('!!!!ERROR!!!!')
            print('CANNOT FIND: '.format(device))

    def _ip_search_(self, device):
        f = open(self.device_table, 'r')
        for line in f:
            dev = line.strip().split(',')
            if device in dev:
                info1 = str(dev[1].strip())
                info2 = int(dev[2].strip())
                ret = [device, info1, info2]
                break
            else:
                pass
        f.close()
        return ret

    def start_osci(self, freq, power):
        """
        DESCRIPTION
        ================
        This function starts the 2nd Lo oscillation.

        ARGUMENTS
        ================
        1. freq: 2nd Lo frequency
            Number: 2 - 20 [GHz]
            Type: float
            Default: Nothing.
        2. power: first Lo power
            Number: -20 - +30 [dBm]
            Type: float
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.change_freq(freq)
        self.change_power(power)
        self.sg.set_output(1)
        return

    def change_freq(self, freq):
        """
        DESCRIPTION
        ================
        This function changes the frequency of the second Lo.

        ARGUMENTS
        ================
        1. freq: second Lo frequency
            Number: 2 - 20 [GHz]
            Type: float
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        if 2.0 <= freq <= 20.0:
            self.sg.set_freq(freq)
        else:
            print('!!!!ERROR!!!!')
            print('invalid freq: '.format(freq))
            print('available freq: 2 - 20 [GHz]')
        return

    def change_power(self, power):
        """
        DESCRIPTION
        ================
        This function changes the RF power.

        ARGUMENTS
        ================
        1. power: second Lo power
            Number: -20 - +30 [dBm]
            Type: float
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        if -20.0 <= power <= 30.0:
            self.sg.set_power(power)
        else:
            print('!!!!ERROR!!!!')
            print('invalid power: '.format(power))
            print('available power: -20 - +30 [dBm]')
        return

    def end_osci(self):
        """
        DESCRIPTION
        ================
        This function stops the 2nd Lo oscillation.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        out = self.query_status()
        if out[2] == 1:
            self.sg.set_output(onoff=0)

        elif out[2] == 0:
            pass
        else:
            print('!!!WARNING!!! OUTPUT STATUS UNDEFINED')
        return

    def query_status(self):
        """
        DESCRIPTION
        ================
        This function queries status of the 2nd Lo.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. freq: frequency of the 2nd Lo.
            Type: float [GHz]
        2. power: power of the 2nd Lo.
            Type: float [dBm]
        3. output: output status of the 2nd Lo.
            Type: int (0:off, 1:on)
        """
        freq = self.sg.query_freq()
        power = self.sg.query_power()
        output = self.sg.query_output()
        return [freq, power, output]


class multi_firstlo(object):
    """
    DESCRIPTION
    ================
    This class controls the 1st Lo for both 100GHz and 200 GHz.

    ARGUMENTS
    ================
    1. multiplier: multiplier of the Lo chain
        Number: 1, 2, 3, ...
        Type: int
        Default: 6
    2. device1: name of the 1st SG of 100GHz registered in the device_table
        Type: string
        Default: 'E8257D1'
    3. device2: name of the 1st SG of 100GHz registered in the device_table
        Type: string
        Default: 'E8257D2'
    4. device_table: file path of the IP table
        Type: string
        Default: '/home/amigos/NASCORX-master/base/device_table_115.txt'
    """

    def __init__(self, multiplier=6, device1='E8257D1', device2='E8257D2',
                 device_table='/home/amigos/NASCORX_System-master/base/device_table_115.txt'):
        # define
        self.multiplier = int(multiplier)
        self.device1 = device1
        self.device2 = device2
        self.device_table = device_table
        # import E8257D control modules
        self.sg1 = firstlo(multiplier=self.multiplier, device=self.device1)
        self.sg2 = firstlo(multiplier=self.multiplier, device=self.device2)

    def start_osci(self, freq, power):
        """
        DESCRIPTION
        ================
        This function starts the first Lo oscillation
        for both 100GHz and 200GHz

        ARGUMENTS
        ================
        1. freq: first Lo frequency (multiplied) [GHz]
            Number: (10MHz-20GHz) * multiplier
            Type: float list
            Length: 2
            Default: Nothing.
        2. power: first Lo power
            Number: -20 - +24 [dBm]
            Type: float list
            Length: 2
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.sg1.start_osci(freq[0], power[0])
        self.sg2.start_osci(freq[1], power[1])
        return

    def change_freq(self, freq):
        """
        DESCRIPTION
        ================
        This function changes the frequency of the first Lo
        for both 100GHz and 200GHz.

        ARGUMENTS
        ================
        1. freq: first Lo frequency (multiplied)
            Number: (10MHz-20GHz) * multiplier [GHz]
            Type: float list
            Length: 2
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.sg1.change_freq(freq[0])
        self.sg2.change_freq(freq[1])

    def change_power(self, power):
        """
        DESCRIPTION
        ================
        This function changes the RF power continuously
        for both 100GHz and 200GHz.

        ARGUMENTS
        ================
        1. power: first Lo power
            Number: -20 - +24 [dBm]
            Type: float list
            Length: 2
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.sg1.change_power(power[0])
        self.sg2.change_power(power[1])
        return

    def end_osci(self):
        """
        DESCRIPTION
        ================
        This function stops the 1st Lo oscillation
        for both 100GHz and 200GHz.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.sg1.end_osci()
        self.sg2.end_osci()
        return

    def query_status(self):
        """
        DESCRIPTION
        ================
        This function queries status of the 1st Lo
        for both 100GHz and 200GHz.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. SG status list x2 (100GHz, 200GHz)
            CONTENTS
            ============
            1. freq: frequency of the 1st Lo.
                Type: float [GHz]
            2. power: power of the 1st Lo.
                Type: float [dBm]
            3. output: output status of the 1st Lo.
                Type: int (0:off, 1:on)
        """
        sg_status1 = self.sg1.query_status()
        sg_status2 = self.sg2.query_status()
        # output frequency -> Lo frequency
        sg_status1[0] *= self.multiplier
        sg_status2[0] *= self.multiplier
        return sg_status1, sg_status2


class multi_secondlo(object):
    """
    DESCRIPTION
    ================
    This class controls the 2nd Lo for both 100GHz and 200GHz.

    ARGUMENTS
    ================
    1. device: name of the 2nd SG registered in the device_table
        Type: string
        DEVICES
        ============
            1. device1: for 110GHz.
                Default: 'MG3692C1'
            2. device2: for 110GHz.
                Default: 'MG3692C2'
            3. device3: for 200GHz.
                Default: 'FSW00201'
            4. device4: for 200GHz.
                Default: 'FSW00202'
    2. device_table: file path of the IP table
        Type: string
        Default: '/home/amigos/NASCORX-master/base/device_table_115.txt'
    """

    def __init__(self, device1='MG3692C1', device2='MG3692C2',
                 device3='FSW00201', device4='FSW00202',
                 device_table='/home/amigos/NASCORX-master/base/device_table_115.txt'):
        # define
        self.device1 = device1
        self.device2 = device2
        self.device3 = device3
        self.device4 = device4
        self.device_table = device_table
        # import SG control modules
        self.sg1 = secondlo(device=self.device1)
        self.sg2 = secondlo(device=self.device2)
        self.sg3 = secondlo(device=self.device3)
        self.sg4 = secondlo(device=self.device4)

    def start_osci(self, freq, power):
        """
        DESCRIPTION
        ================
        This function starts the 2nd Lo oscillation
        for both 100GHz and 200GHz(2x2=devices).

        ARGUMENTS
        ================
        1. freq: 2nd Lo frequency
            Number: 2 - 20 [GHz]
            Type: float list
            Length: 4
            Default: Nothing.
        2. power: first Lo power
            Number: -20 - +30 [dBm]
            Type: float list
            Length: 4
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.sg1.start_osci(freq[0], power[0])
        self.sg2.start_osci(freq[1], power[1])
        self.sg3.start_osci(freq[2], power[2])
        self.sg4.start_osci(freq[3], power[3])
        return

    def change_freq(self, freq):
        """
        DESCRIPTION
        ================
        This function changes the frequency of the second Lo
        for both 100GHz and 200GHz.

        ARGUMENTS
        ================
        1. freq: second Lo frequency
            Number: 2 - 20 [GHz]
            Type: float list
            Length: 4
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.sg1.change_freq(freq[0])
        self.sg2.change_freq(freq[1])
        self.sg3.change_freq(freq[2])
        self.sg4.change_freq(freq[3])
        return

    def change_power(self, power):
        """
        DESCRIPTION
        ================
        This function changes the RF power for both 100GHz and 200GHz.

        ARGUMENTS
        ================
        1. power: second Lo power
            Number: -20 - +30 [dBm]
            Type: float list
            length: 4
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.sg1.change_power(power[0])
        self.sg2.change_power(power[1])
        self.sg3.change_power(power[2])
        self.sg4.change_power(power[3])
        return

    def end_osci(self):
        """
        DESCRIPTION
        ================
        This function stops the 2nd Lo oscillation for both 100GHz and 200GHz.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.sg1.end_osci()
        self.sg2.end_osci()
        self.sg3.end_osci()
        self.sg4.end_osci()
        return

    def query_status(self):
        """
        DESCRIPTION
        ================
        This function queries status of the 2nd Lo for both 100GHz and 200GHz.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. SG status list x4 (100GHz, 200GHz)
            CONTENTS
            ============
            1. freq: frequency of the 1st Lo.
                Type: float [GHz]
            2. power: power of the 1st Lo.
                Type: float [dBm]
            3. output: output status of the 1st Lo.
                Type: int (0:off, 1:on)
        """
        sg_status1 = self.sg1.query_status()
        sg_status2 = self.sg2.query_status()
        sg_status3 = self.sg3.query_status()
        sg_status4 = self.sg4.query_status()
        return sg_status1, sg_status2, sg_status3, sg_status4

# written by K.Urushihara
