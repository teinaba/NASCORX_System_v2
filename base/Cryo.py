#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


# import modules
import NASCORX_System.device.CPZ3177 as CPZ3177
import NASCORX_System.device.CPZ340516 as CPZ340516
import NASCORX_System.device.CPZ340816 as CPZ340816


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
                 device_table='/home/amigos/NASCORX_System-master/NASCORX_System/base/device_table_115.txt'):
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
        """
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
        """
        Vmix_Limit = 30.0 # mV
        ch_range = range(16)  # number of channel
        Vda = (1.0/3.0)*float(Vmix)  # mixer bias[mV] --> D/A voltage[V]
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
        """
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
        """
        ret = self.davc.query_voltage()
        Vmix = []
        for i in ret:
            Vmix.append(i*3.0)
        return Vmix

    def monitor_sis(self):
        """
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
        """
        ret = self.ad.query_input()
        Vmix_mon = []
        for i in range(len(ret)):
            Vmix_mon.append(float(ret[i]))
        return Vmix_mon

    def set_loatt(self, att, ch):
        """
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
        """
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
        """
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
        """
        ret = self.dacc.query_current()
        att = []
        for i in range(len(ret)):
            att.append(float(ret[i])*1e+3)
        return att


class hemt(object):
    """
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
    """

    def __init__(self, hemtda='CPZ340816b', hemtad='CPZ3177b', device_table='/home/amigos/NASCORX-System-master/base/device_table_115.txt'):
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
        """
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
        """
        if 0.0<=voltage<=2.0:
            if 0<=ch<=15:
                self.da.set_voltage(voltage=voltage, ch=ch)
                self.da.set_output(onoff=1)
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
        """
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
        """
        if -2.5<=voltage<=2.5:
            if 0<=ch<=15:
                self.da.set_voltage(voltage=voltage, ch=ch)
                self.da.set_output(onoff=1)
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


class box(object):
    device_table = '/home/amigos/NASCORX_System-master/NASCORX_System/base/device_table_115.txt'
    ver = '2017.11.06'

    def __init__(self, boxda='CPZ340816a', loda='CPZ340516a', boxad='CPZ3177a'):
        # define
        # ------
        self.boxda = boxda
        self.loda = loda
        self.boxad = boxad
        # search board number
        # -------------------
        self.nboxda = self._board_search_(device=self.boxda)
        self.nloda = self._board_search_(device=self.loda)
        self.nboxad = self._board_search_(device=self.boxad)
        # import board control module
        # ---------------------------
        self.davc = CPZ340816.cpz340816(dev=self.nboxda[1])
        self.dacc = CPZ340516.cpz340516(dev=self.nloda[1])
        self.ad = CPZ3177.cpz3177(dev=self.nboxad[1])
        # settings
        # --------
        self.dacc.set_Irange(mode='DA_0_100mA')
        self.ad.set_mode(mode='diff')
        self.ad.set_input_range(Vrange='AD_5V')
        return

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

    def set_sisv(self, Vmix):
        """
        DESCRIPTION
        ================
        This function sets the mixer bias.

        ARGUMENTS
        ================
        1. Vmix: mixer bias [mV]
            Number: 0 - 50 [mV]
            Type: float list
            Length: 2
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        # check input list length
        # -----------------------
        if len(Vmix) == 2:
            pass
        else:
            msg = '{0}\n{1}\n{2}'.format('Input Invalid Length Error',
                                         'Invalid list Length: {0}'.format(len(Vmix)),
                                         'Available Length: 2')
            raise ValueError(msg)
        # set SIS V
        # ---------
        Vmix_Limit = 30.0  # [mV]
        Vda = [1.0 / 3.0 * float(value) for value in Vmix]
        for i in range(2):
            if -Vmix_Limit <= Vmix[i] <= Vmix_Limit:
                self.davc.set_voltage(voltage=Vda[i], ch=i)
            elif Vmix[i] is None:
                pass
            else:
                msg = '{0}\n{1}\n{2}'.format('Input Invalid Value Error',
                                             'Invalid Voltage: {0} [V]'.format(Vmix),
                                             'Available Voltage: 0.0 -- {0} [mV]'.format(Vmix_Limit))
                raise ValueError(msg)
        self.davc.set_output(onoff=1)
        return

    def query_sisv(self):
        """
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
            Length: 2
        """
        ret = self.davc.query_voltage()
        Vmix = [float(value) * 3.0 for value in ret]
        return Vmix

    def monitor_sis(self, divided=None):
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
        ret = list(map(float, ret_raw))  # ret --> float
        sisV_mon = ret[0::2]  # [0:23:2]のほうがいい??
        sisI_mon = ret[1::2]
        if divided is True:
            return sisV_mon, sisI_mon
        else:
            return ret

    def set_loatt(self, att):
        """
        DESCRIPTION
        ================
        This function sets the 1st Lo attenuation level for all channels.

        ARGUMENTS
        ================
        1. att: attenuation level [mA]
            Number: 0 - 100 [mA]
            Type: float
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        if 0.0 <= att <= 100.0:
            self.dacc.set_current(current=float(att)*1e-3, ch=0)
        elif att is None:
            pass
        else:
            msg = '{0}\n{1}\n{2}'.format('Input Invalid Value Error',
                                         'Invalid att: {0}'.format(att),
                                         'Available att: 0.0 -- 100.0 [mA]')
            raise ValueError(msg)
        self.dacc.set_output(onoff=1)
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
            Length: 8
        """
        ret = self.dacc.query_current()
        att = float(ret[0]) * 1e+3
        return att

    def set_Vd(self, voltage):
        """
        DESCRIPTION
        ================
        This function sets the drain voltage for all amplifiers.

        ARGUMENTS
        ================
        1. voltage: drain voltage [V]
            Number: 0 -- 2.0 [V].da
            Type: float list
            Length: 4
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        # Check input list length
        # -----------------------
        if len(voltage) == 4:
            pass
        else:
            msg = '{0}\n{1}\n{2}'.format('Input Invalid Length Error',
                                         'Invalid list Length: {0}'.format(len(voltage)),
                                         'Available Length: 4')
            raise ValueError(msg)

        # Set Vd
        # ------
        for i in range(4):
            if 0.0 <= voltage[i] <= 2.0:
                self.davc.set_voltage(voltage=voltage[i], ch=4+i*3)
            elif voltage[i] is None:
                pass
            else:
                msg = '{0}\n{1}\n{2}'.format('Input Invalid Value Error',
                                             'Invalid Voltage: {0} [V]'.format(voltage[i]),
                                             'Available Voltage: 0.0 -- 2.0 [V]')
                raise ValueError(msg)
        self.davc.set_output(onoff=1)
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
            Length: 4
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        # Check input list length
        # -----------------------
        if len(voltage) == 4:
            pass
        else:
            msg = '{0}\n{1}\n{2}'.format('Input Invalid Length Error',
                                         'Invalid list Length: {0}'.format(len(voltage)),
                                         'Available Length: 4')
            raise ValueError(msg)
        # set Vg1
        # -------
        for i in range(4):
            if -2.5 <= voltage[i] <= 2.5:
                self.davc.set_voltage(voltage=voltage[i], ch=5+i*3)
            elif voltage[i] is None:
                pass
            else:
                msg = '{0}\n{1}\n{2}'.format('Input Invalid Value Error',
                                             'Invalid Voltage: {0} [V]'.format(voltage[i]),
                                             'Available Voltage: -2.5 -- +2.5 [V]')
                raise ValueError(msg)
        self.davc.set_output(onoff=1)
        return

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
            Length: 4
            Default: Nothing.

        RETURNS
        ================
        Nothing.
        """
        # Check input list length
        # -----------------------
        if len(voltage) == 4:
            pass
        else:
            msg = '{0}\n{1}\n{2}'.format('Input Invalid Length Error',
                                         'Invalid list Length: {0}'.format(len(voltage)),
                                         'Available Length: 4')
            raise ValueError(msg)
        # Set Vg2
        # -------
        for i in range(4):
            if -2.5 <= voltage[i] <= 2.5:
                self.davc.set_voltage(voltage=voltage[i], ch=6+i*3)
            elif voltage[i] is None:
                pass
            else:
                msg = '{0}\n{1}\n{2}'.format('Input Invalid Value Error',
                                             'Invalid Voltage: {0} [V]'.format(voltage[i]),
                                             'Available Voltage: -2.5 -- +2.5 [V]')
                raise ValueError(msg)
        self.davc.set_output(onoff=1)
        return

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
        ret = self.ad.query_input()
        voltage = list(map(float, ret))
        return voltage

# History
# -------
# written by K.Urushihara
# 2017/11/06 T.Inaba: debugged and create box class.
