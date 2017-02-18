#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import sys, time
sys.path.append('/home/amigos/NASCORX-master/device/')
import CPZ3177, CPZ340516, CPZ340816

class mixer(object):
    '''
    DESCRIPTION
    ================
    This class controls the SIS mixer.

    ARGUMENTS
    ================
    1. sisda: name of the D/A board of SIS mixer registered in the IP_table
        Type: string
        Default: 'CPZ340816'
    2. loda: name of the D/A board of LO attenuator registered in the IP_table
        Type: string
        Default: 'CPZ340516'
    3. sisad: name of the A/D board of SIS mixer registered in the IP_table
        Type: string
        Default: 'CPZ3177'
    4. device_table: file path of the IP table
        Type: string
        Default: '/home/amigos/NASCORX-master/base/IP_table_115.txt'
    '''

    def __init__(self, sisda='CPZ340816a', loda='CPZ340516a', sisad='CPZ3177a', device_table='/home/amigos/NASCORX-master/base/device_table_115.txt'):
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

#written by K.Urushihara
