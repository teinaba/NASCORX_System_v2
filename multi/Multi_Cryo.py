#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import sys, time
import numpy as np
sys.path.append('/home/amigos/NASCORX_System-master/device/')
import CPZ3177, CPZ340516, CPZ340816

class mixer(object):
    """
    DESCRIPTION
    ================
    This class controls the SIS mixer.

    ARGUMENTS
    ================
    1. sisda: name of the D/A board of SIS mixer registered in the IP_table
        Type: string
        Default: 'CPZ340816a'
    2. loda: name of the D/A board of LO attenuator registered in the IP_table
        Type: string
        Default: 'CPZ340516a'
    3. sisad: name of the A/D board of SIS mixer registered in the IP_table
        Type: string
        Default: 'CPZ3177a'
    4. device_table: file path of the IP table
        Type: string
        Default: '/home/amigos/NASCORX-master/base/IP_table_115.txt'
    """

    def __init__(self, sisda='CPZ340816a', loda='CPZ340516a', sisad='CPZ3177a',
                 device_table='/home/amigos/NASCORX_System-master/base/device_table_115.txt'):
        self.sisda=sisda
        self.loda=loda
        self.sisad=sisad
        self.device_table = device_table
        self.nsisda = self._board_search_(device=self.sisda)
        self.nloda = self._board_search_(device=self.loda)
        self.nsisad = self._board_search_(device=self.sisad)
        self.davc = CPZ340816.cpz340816(dev=self.nsisda[1])
        self.dacc = CPZ340516.cpz340516(dev=self.nloda[1])
        self.ad = CPZ3177.cpz3177(dev=self.nsisad[1])
        self.dacc.set_Irange(mode='DA_0_100mA')
        self.ad.set_mode(mode='diff')
        self.ad.set_input_range(Vrange='AD_5V')

    def _board_search_(self, device):
        f = open(self.device_table, 'r')
        for line in f:
            dev = line.strip().split(',')
            if device in dev:
                info1 = int(dev[1].strip())
                break
            else:
                pass
        f.close()
        ret = [device, info1]
        return ret

    def close_box(self):
        """        
        DESCRIPTION
        ================
        This function close the remote connection.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        Nothing.
        """
        self.davc.close_board()
        self.dacc.close_board()
        self.ad.close_board()
        return 

    def set_sisv(self, Vmix, ch):
        '''        
        DESCRIPTION
        ================
        This function sets the mixer bias.
        
        ARGUMENTS
        ================
        1. Vmix: mixer bias [mV]
            Number: 0 - 50 [mV]
            Type: float
            Default: Nothing.
        2. ch: channel of the SIS mixeres.
            Number: 0 - 15
            Type: int
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        '''
        Vmix_Limit = 30.0 # mV
        ch_range = range(16) #numberes of channel
        Vda = (1.0/3.0)*float(Vmix) #mixer bias[mV] --> D/A voltage[V]
        if 0.0<=Vmix<=Vmix_Limit:
            if ch in ch_range:
                self.davc.set_voltage(voltage=Vda, ch=ch)
                self.davc.set_output(onoff=1)
            else:
                print('!!!!ERROR!!!!')
                print('invalid ch: '+str(ch))
                print('available ch: '+str(ch_range[0])+' - '+str(ch_range[-1]))
        else:
            print('!!!!ERROR!!!!')
            print('invalid voltage: '+str(Vmix))
            print('available voltage: 0.0 - '+str(Vmix_Limit)+' [mV]')
        return

    def query_sisv(self):
        '''        
        DESCRIPTION
        ================
        This function queries the mixer bias.
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. Vmix: mixer bias [mV]
            Type: float list
        '''
        ret = self.davc.query_voltage()
        Vmix = []
        for i in ret:
            Vmix.append(i*3.0)
        return Vmix

    def monitor_sis(self):
        '''        
        DESCRIPTION
        ================
        This function queries the mixer monitor voltage.
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. voltage: monitor voltage [V]
            Type: float list
        '''
        ret = self.ad.query_input()
        Vmix_mon = []
        for i in range(len(ret)):
            Vmix_mon.append(float(ret[i]))
        return Vmix_mon

    def set_loatt(self, att, ch):
        '''        
        DESCRIPTION
        ================
        This function sets the 1st Lo attenuation level.
        
        ARGUMENTS
        ================
        1. att: attenuation level [mA]
            Number: 0 - 100 [mA]
            Type: float
            Default: Nothing.
        2. ch: channel of the Lo attenuator.
            Number: 0 - 7
            Type: int
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        '''
        if 0.0<=att<=100.0:
            if 0<=ch<=7:
                self.dacc.set_current(current=float(att)*1e-3, ch=ch)
                self.dacc.set_output(onoff=1)
            else:
                print('!!!!ERROR!!!!')
                print('invalid ch: '+str(ch))
                print('available ch: 0 - 7')
        else:
            print('!!!!ERROR!!!!')
            print('invalid att: '+str(att))
            print('available att: 0.0 - 100.0 [mA]')
        return

    def query_loatt(self):
        '''        
        DESCRIPTION
        ================
        This function queries the 1st Lo attenuation level.
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. att: attenuation level [mA]
            Type: float list
        '''
        ret = self.dacc.query_current()
        att = []
        for i in range(len(ret)):
            att.append(float(ret[i])*1e+3)
        return att


class hemt(object):
    '''
    DESCRIPTION
    ================
    This class controls the HEMT amplifire.

    ARGUMENTS
    ================
    1. hemtda: name of the D/A board of SIS mixer registered in the IP_table
        Type: string
        Default: 'CPZ340816'
    2. hemtad: name of the A/D board of SIS mixer registered in the IP_table
        Type: string
        Default: 'CPZ3177'
    3. device_table: file path of the IP table
        Type: string
        Default: '/home/amigos/NASCORX-master/base/IP_table_115.txt'
    '''

    def __init__(self, hemtda='CPZ340816b', hemtad='CPZ3177b', device_table='/home/amigos/NASCORX-master/base/device_table_115.txt'):
        self.hemtda=hemtda
        self.hemtad=hemtad
        self.device_table = device_table
        self.nhemtda = self._board_search_(device=self.hemtda)
        self.nhemtad = self._board_search_(device=self.hemtad)
        self.da = CPZ340816.cpz340816(dev=self.nhemtda[1])
        self.ad = CPZ3177.cpz3177(dev=self.nhemtad[1])
        self.ad.set_mode(mode='single')
        self.ad.set_input_range(Vrange='AD_10V')

    def _board_search_(self, device):
        f = open(self.device_table, 'r')
        for line in f:
            dev = line.strip().split(',')
            if device in dev:
                info1 = int(dev[1].strip())
                break
            else:
                pass
        f.close()
        ret = [device, info1]
        return ret

    def close_box(self):
        """        
        DESCRIPTION
        ================
        This function close the remote connection.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        Nothing.
        """
        self.da.close_board()
        self.ad.close_board()
        return 

    def set_Vd(self, voltage, ch):
        '''        
        DESCRIPTION
        ================
        This function sets the drain voltage.
        
        ARGUMENTS
        ================
        1. voltage: drain voltage [V]
            Number: 0 -- 2.0 [V]
            Type: float
            Default: Nothing.
        2. ch: channel of the HEMT amplifire.
            Number: 0 -- 15
            Type: int
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        '''
        if 0.0<=voltage<=2.0:
            if 0<=ch<=15:
                self.davc.set_voltage(voltage=voltage, ch=ch)
                self.davc.set_output(onoff=1)
            else:
                print('!!!!ERROR!!!!')
                print('invalid ch: '+str(ch))
                print('available ch: 0 -- 15')
        else:
            print('!!!!ERROR!!!!')
            print('invalid voltage: '+str(voltage))
            print('available voltage: 0.0 -- 2.0 [V]')
        return

    def set_Vg(self, voltage, ch):
        '''        
        DESCRIPTION
        ================
        This function sets the gate voltage.
        
        ARGUMENTS
        ================
        1. voltage: gate voltage [V]
            Number: -2.5 -- +2.5 [V]
            Type: float
            Default: Nothing.
        2. ch: channel of the HEMT amplifire.
            Number: 0 -- 15
            Type: int
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        '''
        if -2.5<=voltage<=2.5:
            if 0<=ch<=15:
                self.davc.set_voltage(voltage=voltage, ch=ch)
                self.davc.set_output(onoff=1)
            else:
                print('!!!!ERROR!!!!')
                print('invalid ch: '+str(ch))
                print('available ch: 0 -- 15')
        else:
            print('!!!!ERROR!!!!')
            print('invalid voltage: '+str(voltage))
            print('available voltage: -2.5 -- +2.5 [V]')
        return

    def monitor_hemt(self):
        '''        
        DESCRIPTION
        ================
        This function queries the HEMT monitor voltage.
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. voltage: monitor voltage [V]
            Type: float list
        '''
        ret = self.davc.query_voltage()
        voltage = ret
        return voltage


class multi_mixer(object):
    """
    DESCRIPTION
    ================
    This class controls the SIS mixer for multi system.

    ARGUMENTS
    ================
    Later
    """

    def __init__(self, sisda1='CPZ340816a', sisda2='CPZ340816b', sisda3='CPZ340816c',
                 loda1='CPZ340516a', loda2='CPZ340516b', sisad='CPZ3177a',
                 device_table='/home/amigos/NASCORX_System-master/base/device_table_115.txt'):
        # define
        self.sisda1 = sisda1    # for beam 1-2
        self.sisda2 = sisda2    # for beam 3-4
        self.sisda3 = sisda3    # for 230GHz
        self.loda1 = loda1    # for ch 1-8
        self.loda2 = loda2    # for ch 9-10
        self.sisad = sisad
        self.device_table = device_table
        # search board number
        self.nsisda1 = self._board_search_(device=self.sisda1)
        self.nsisda2 = self._board_search_(device=self.sisda2)
        self.nsisda3 = self._board_search_(device=self.sisda3)
        self.nloda1 = self._board_search_(device=self.loda1)
        self.nloda2 = self._board_search_(device=self.loda2)
        self.nsisad = self._board_search_(device=self.sisad)
        # import board control module
        self.davc1 = CPZ340816.cpz340816(dev=self.nsisda1[1])    # vc = voltage control ??
        self.davc2 = CPZ340816.cpz340816(dev=self.nsisda2[1])
        self.davc3 = CPZ340816.cpz340816(dev=self.nsisda3[1])
        self.dacc1 = CPZ340516.cpz340516(dev=self.nloda1[1])    # cc = current control ??
        self.dacc2 = CPZ340516.cpz340516(dev=self.nloda2[1])
        self.ad = CPZ3177.cpz3177(dev=self.nsisad[1])
        # settings
        self.dacc1.set_Irange(mode='DA_0_100mA')
        self.dacc2.set_Irange(mode='DA_0_100mA')
        self.ad.set_mode(mode='diff')
        self.ad.set_input_range(Vrange='AD_5V')

    def _board_search_(self, device):
        f = open(self.device_table, 'r')
        for line in f:
            dev = line.strip().split(',')
            if device in dev:
                info1 = int(dev[1].strip())
                break
            else:
                pass
        f.close()
        ret = [device, info1]
        return ret

    def close_box(self):
        """
        DESCRIPTION
        ================
        This function close the remote connection.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.davc1.close_board()
        self.davc2.close_board()
        self.davc3.close_board()
        self.dacc1.close_board()
        self.dacc2.close_board()
        self.ad.close_board()
        return

    def set_sisv(self, Vmix):
        """
        DESCRIPTION
        ================
        This function sets the mixer bias for all mixers.

        ARGUMENTS
        ================
        1. Vmix: mixer bias [mV]
            Number: 0 - 50 [mV]
            Type: float list
            Length: 12
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        Vmix_Limit = 30.0    # [mV]
        Vda = [1.0/3.0 * float(value) for value in Vmix]
        #Vda = (1.0/3.0) * np.array(Vmix)    # mixer bias[mV] --> D/A voltage[V] ## arrayをfloat指定しなくて平気かな？
        for i in range(12):
            if 0 <= i <= 3:    # for beam 1-2
                if 0.0 <= Vmix[i] <= Vmix_Limit:
                    self.davc1.set_voltage(voltage=Vda[i], ch=i)
                    self.davc1.set_output(onoff=1)
                else:
                    print('!!!!ERROR!!!!')
                    print('invalid voltage: {0}'.format(Vmix))
                    print('available voltage: 0.0 - {0} [mV]'.format(Vmix_Limit))
            elif 4 <= i <= 7:    # for beam 3-4
                if 0.0 <= Vmix[i] <= Vmix_Limit:
                    self.davc2.set_voltage(voltage=Vda[i], ch=i-4)
                    self.davc2.set_output(onoff=1)
                else:
                    print('!!!!ERROR!!!!')
                    print('invalid voltage: {0}'.format(Vmix))
                    print('available voltage: 0.0 - {0} [mV]'.format(Vmix_Limit))
            elif 8 <= i <= 11:    # for 230GHz
                if 0.0 <= Vmix[i] <= Vmix_Limit:
                    self.davc3.set_voltage(voltage=Vda[i], ch=i-8)
                    self.davc3.set_output(onoff=1)
                else:
                    print('!!!!ERROR!!!!')
                    print('invalid voltage: '.format(Vmix))
                    print('available voltage: 0.0 - {0} [mV]'.format(Vmix_Limit))
            return

    def query_sisv(self):
        """
        DESCRIPTION
        ================
        This function queries the mixer bias for all mixers.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. Vmix: mixer bias [mV]
            Type: float list
            Length: 12
        """
        ret1 = self.davc1.query_voltage()
        ret2 = self.davc2.query_voltage()
        ret3 = self.davc3.query_voltage()
        ret = ret1[:4] + ret2[:4] + ret3[:4]
        Vmix = [float(value)*3.0 for value in ret]
        return Vmix

    def monitor_sis(self):
        """
        DESCRIPTION
        ================
        This function queries the mixer monitor voltage and current for all mixers.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. voltage: monitor voltage [V]
            Type: float list
            Length: 12
        2. current: monitor current [V]
            Type: float list
            Length: 12
        """
        ret_raw = self.ad.query_input()
        ret = list(map(float, ret_raw))    # ret --> float
        sisV_mon = ret[0::2]
        sisI_mon = ret[1::2]
        return sisV_mon, sisI_mon

    def set_loatt(self, att):
        """
        DESCRIPTION
        ================
        This function sets the 1st Lo attenuation level for all channels.

        ARGUMENTS
        ================
        1. att: attenuation level [mA]
            Number: 0 - 100 [mA]
            Type: float list
            Length: 12
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        for ch in range(10):
            if 0.0 <= att[ch] <= 100.0:
                if 0 <= ch <= 7:
                    self.dacc1.set_current(current=float(att[ch])*1e-3, ch=ch)
                    self.dacc1.set_output(onoff=1)
                elif 8 <= ch <= 9:
                    self.dacc2.set_current(current=float(att[ch])*1e-3, ch=ch-8)
                    self.dacc2.set_output(onoff=1)
            else:
                print('!!!!ERROR!!!!')
                print('invalid att: {0}'.format(att[ch]))
                print('available att: 0.0 - 100.0 [mA]')
        return

    def query_loatt(self):
        """
        DESCRIPTION
        ================
        This function queries the 1st Lo attenuation level for all channels.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. att: attenuation level [mA]
            Type: float list
            Length: 12
        """
        ret1 = self.dacc1.query_current()
        ret2 = self.dacc2.query_current()
        ret = ret1 + ret2[:2]
        att = [float(value)*3.0 for value in ret]
        return att


class multi_hemt(object):
    """
    DESCRIPTION
    ================
    This class controls the HEMT amplifier.

    ARGUMENTS
    ================
    1. hemtda: name of the D/A board of SIS mixer registered in the IP_table
        Type: string
        Default: 'CPZ340816'
    2. hemtad: name of the A/D board of SIS mixer registered in the IP_table
        Type: string
        Default: 'CPZ3177'
    3. device_table: file path of the IP table
        Type: string
        Default: '/home/amigos/NASCORX-master/base/IP_table_115.txt'
    """

    def __init__(self, hemtda1='CPZ340816a', hemtda2='CPZ340816b', hemtad='CPZ3177b',
                 device_table='/home/amigos/NASCORX-master/base/device_table_115.txt'):
        # define
        self.hemtda1 = hemtda1
        self.hemtda2 = hemtda2
        self.hemtad = hemtad
        self.device_table = device_table
        # search board number
        self.nhemtda1 = self._board_search_(device=self.hemtda1)
        self.nhemtda2 = self._board_search_(device=self.hemtda2)
        self.nhemtad = self._board_search_(device=self.hemtad)
        # import board control module
        self.da1 = CPZ340816.cpz340816(dev=self.nhemtda1[1])
        self.da2 = CPZ340816.cpz340816(dev=self.nhemtda2[1])
        self.ad = CPZ3177.cpz3177(dev=self.nhemtad[1])
        # settings
        self.ad.set_mode(mode='single')
        self.ad.set_input_range(Vrange='AD_10V')

    def _board_search_(self, device):
        f = open(self.device_table, 'r')
        for line in f:
            dev = line.strip().split(',')
            if device in dev:
                info1 = int(dev[1].strip())
                break
            else:
                pass
        f.close()
        ret = [device, info1]
        return ret

    def close_box(self):
        """
        DESCRIPTION
        ================
        This function close the remote connection for all boards.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        self.da1.close_board()
        self.da2.close_board()
        self.ad.close_board()
        return

    def set_Vd(self, voltage):
        """
        DESCRIPTION
        ================
        This function sets the drain voltage for all amplifiers.

        ARGUMENTS
        ================
        1. voltage: drain voltage [V]
            Number: 0 -- 2.0 [V]
            Type: float list
            Length: 8
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        for i in range(8):
            if 0.0 <= voltage[i] <= 2.0:
                if 0 <= i <= 3:
                    self.da1.set_voltage(voltage=voltage[i], ch=4+i*3)
                    self.da1.set_output(onoff=1)
                elif 4 <= i <= 7:
                    self.da2.set_voltage(voltage=voltage[i], ch=4+(i-4)*3)
                    self.da2.set_output(onoff=1)
            else:
                print('!!!!ERROR!!!!')
                print('invalid voltage: '.format(voltage[i]))
                print('available voltage: 0.0 -- 2.0 [V]')
        return

    def set_Vg1(self, voltage):
        """
        DESCRIPTION
        ================
        This function sets the gate voltage 1 for all amplifiers.

        ARGUMENTS
        ================
        1. voltage: gate voltage [V]
            Number: -2.5 -- +2.5 [V]
            Type: float list
            Length: 8
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        for i in range(8):
            if -2.5 <= voltage[i] <= 2.5:
                if 0 <= i <= 3:
                    self.da1.set_voltage(voltage=voltage[i], ch=5+i*3)
                    self.da1.set_output(onoff=1)
                elif 4 <= i <= 7:
                    self.da2.set_voltage(voltage=voltage[i], ch=5+(i-4)*3)
                    self.da2.set_output(onoff=1)
            else:
                print('!!!!ERROR!!!!')
                print('invalid voltage: '.format(voltage[i]))
                print('available voltage: -2.5 -- +2.5 [V]')

    def set_Vg2(self, voltage):
        """
        DESCRIPTION
        ================
        This function sets the gate voltage 2 for all amplifiers.

        ARGUMENTS
        ================
        1. voltage: gate voltage [V]
            Number: -2.5 -- +2.5 [V]
            Type: float list
            Length: 8
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        for i in range(8):
            if -2.5 <= voltage[i] <= 2.5:
                if 0 <= i <= 3:
                    self.da1.set_voltage(voltage=voltage[i], ch=6+i*3)
                    self.da1.set_output(onoff=1)
                elif 4 <= i <= 7:
                    self.da2.set_voltage(voltage=voltage[i], ch=6+(i-4)*3)
                    self.da2.set_output(onoff=1)
            else:
                print('!!!!ERROR!!!!')
                print('invalid voltage: '.format(voltage[i]))
                print('available voltage: -2.5 -- +2.5 [V]')

    def monitor_hemt(self):
        """
        DESCRIPTION
        ================
        This function queries the HEMT monitor voltage.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. voltage: monitor voltage [V]
            Type: float list
        """
        ret = self.ad.query_voltage()
        voltage = ret
        return voltage


# written by K.Urushihara
# 2017/07/18 T.Inaba: add multi_mixer, multi_hemt
# 2017/07/21 T.Inaba: minor changes (np.array->list comprehension, double quotation)