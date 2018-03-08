#! /usr/bin/env python

# import modules
import time
import numpy
import matplotlib.pyplot

import pymeasure
from NASCORX_System.base import config_handler
from NASCORX_System.base import sis
from NASCORX_System.device import ML2437A


class Vsweep(object):
    method = 'SIS Trx Measurement'
    ver = '2017.12.22'
    BE_num = 16
    savedir = '/home/amigos/NASCORX_Measurement/1223/'

    def __init__(self):
        pass

    def run(self, initV=0.0, finV=8.0, interval=0.1, onoff='cnf', lo=[0]*10):

        # Print Welcome massage
        # ----------------------
        print('\n\n'
              ' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n'
              '   NASCO RX : SIS VSWEEP Measurement \n'
              ' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n'
              ' ver - {}\n'
              '\n\n'.format(self.ver))

        # get using channel and Lo att
        # ----------------------------
        print('== SIS CHANNEL LIST ==\n'
              '   CH1  : Beam2-L\n'
              '   CH2  : Beam2-R\n'
              '   CH3  : Beam3-L\n'
              '   CH4  : Beam3-R\n'
              '   CH5  : Beam4-L\n'
              '   CH6  : Beam4-R\n'
              '   CH7  : Beam5-L\n'
              '   CH8  : Beam5-R\n'
              '   CH9  : Beam1-LU\n'
              '   CH10 : Beam1-LL\n'
              '   CH11 : Beam1-RU\n'
              '   CH12 : Beam1-RL\n')
        ch1 = int(input(' IF1 ?  :  '))
        ch2 = int(input(' IF2 ?  :  '))
        lo1 = float(input(' IF1 - Lo att [mA] ?  : '))
        lo2 = float(input(' IF2 - Lo att [mA] ?  : '))

        # channel handling
        # ----------------
        if ch1 == 9 or ch1 == 10:
            loch1 = 9
        elif ch1 == 11 or ch1 == 12:
            loch1 = 10
        else:
            loch1 = ch1
            pass

        if ch2 == 9 or ch2 == 10:
            loch2 = 9
        elif ch2 == 11 or ch2 == 12:
            loch2 = 10
        else:
            loch2 = ch2
            pass

        # create onoff and Lo-list
        # ------------------------
        onoff = [0]*12
        onoff[ch1-1] = 1
        onoff[ch2-1] = 1

        lo = [0]*10
        lo[loch1-1] = lo1
        lo[loch2-2] = lo2

        # Input value check
        # -----------------
        repeat = self.input_value_check(initV=initV, finV=finV, interval=interval)

        """
        # Check available unit
        # --------------------
        if onoff == 'cnf':
            onoff = config_handler.Config_handler.check_sis_state()
        else:
            pass
        """
        # Set Driver
        # ----------
        self.driver = sis.multi_box()
        print('PCI board drivers are set.')

        # set PowerMeter
        # --------------
        self.setup_powermeter()

        # == Main ========

        # Lo setting
        # ----------
        if lo is None: pass
        else: self.driver.set_loatt(att=lo)

        # HEMT setting
        # ------------
        self.driver.set_Vd(voltage=[1.1]*8)
        self.driver.set_Vg1(voltage=[1.6]*8)
        self.driver.set_Vg2(voltage=[1.6]*8)

        # Measurement part
        # ----------------
        data = self.measure(repeat=repeat, initV=initV, interval=interval, onoff=onoff)

        # Calculate Y-factor and Tsys
        # ---------------------------
        data = self.trx_calculate(data=data)

        # Data saving
        # -----------
        datetime = time.strftime('%Y%m%d-%H%M%S')
        filename = self.savedir + 'c4sis_vsweep_{}.csv'.format(datetime)
        header = 'D/A-V,' \
                 '2L-V,2L-I,2R-V,2R-I,3L-V,3L-I,3R-V,3R-I,4L-V,4L-I,4R-V,4R-I,5L-V,5L-I,5R-V,5R-I,' \
                 '1LU-V,1LU-I,1LL-V,1LL-I,1RU-V,1RU-I,1RL-V,1RL-I,Phot1,Phot2,' \
                 '2L-V,2L-I,2R-V,2R-I,3L-V,3L-I,3R-V,3R-I,4L-V,4L-I,4R-V,4R-I,5L-V,5L-I,5R-V,5R-I,' \
                 '1LU-V,1LU-I,1LL-V,1LL-I,1RU-V,1RU-I,1RL-V,1RL-I,Pcold1,Pcold2,Yfac1,Yfac2,Trx1,Trx2'
        numpy.savetxt(filename, data, fmt='%.5f', header=header, delimiter=',')

        # Reconfigure SIS bias setting
        # ----------------------------
        self.driver.close_box()

        # Data loading
        # ------------
        data = numpy.loadtxt(filename, skiprows=1, delimiter=',')

        # plot part
        # ---------
        self.ttlplot(data=data, initV=initV, finV=finV, datetime=datetime, lo=lo1, IF=1, ch=ch1)
        self.ttlplot(data=data, initV=initV, finV=finV, datetime=datetime, lo=lo2, IF=2, ch=ch2)

        # Output on Terminal
        # ------------------
        print('\n\n')
        print('Datetime : {}'.format(datetime))
        print('IF1      : ch{}'.format(ch1))
        print('IF1      : ch{}'.format(ch2))
        print('Lo1      : {} [mA]'.format(lo1))
        print('Lo2      : {} [mA]'.format(lo2))
        return

    def measure(self, repeat, initV=0.0, interval=0.1, onoff=[0]*12):
        data = []

        # HOT measurement
        # ---------------
        # TODO : HOT IN
        data_hot = self.sweep_sisv(repeat, initV=initV, interval=interval, onoff=onoff, index=True)

        # SKY measurement
        # ---------------
        # TODO : HOT OUT, OBS SKY
        data_sky = self.sweep_sisv(repeat, initV=initV, interval=interval, onoff=onoff, index=False)

        # data arrangement
        # ----------------
        hot_arr = numpy.array(data_hot)
        sky_arr = numpy.array(data_sky)
        ret = numpy.concatenate((hot_arr, sky_arr), axis=1)
        return ret

    def sweep_sisv(self, repeat, initV=0.0, interval=0.1, onoff=[0]*12, index=True):
        data = []
        setV_list = [0] * 12

        for i in range(repeat+1):
            temp = []

            # set bias
            # --------
            setV = initV + i * interval
            for j in range(12):
                if onoff[j] == 1: setV_list[j] = setV
                else: pass
            self.driver.set_sisv(Vmix=setV_list)

            time.sleep(0.1)

            # get data
            # --------
            pow1 = self.pm1.measure()            # PowerMeter1
            pow2 = self.pm2.measure()            # PowerMeter2
            ad = self.driver.monitor_sis()

            # data arrangement
            # ----------------
            if index is True :
                temp.append(setV)
            # AD data scaling --
            for j in range(24):
                if j % 2 == 0:
                    temp.append(ad[j]*1e+1)      # AD[V] --> bias [mV]
                elif j % 2 == 1:
                    temp.append(ad[j]*1e+3)      # AD[V] --> current [uA]
            temp.append(pow1)
            temp.append(pow2)
            data.append(temp)
        return data

    def trx_calculate(self, data, Tamb=300):
        # dB Y-factor --
        Yfac1 = data[:, 25] - data[:, 51]
        Yfac2 = data[:, 26] - data[:, 52]

        # Tsys calculation --
        Tsys1 = (Tamb - 77 * 10**(Yfac1/10)) / (10**(Yfac1/10) - 1)
        Tsys2 = (Tamb - 77 * 10**(Yfac2/10)) / (10**(Yfac2/10) - 1)

        # data arrangement --
        data = [numpy.append(data[i], t) for i, t in enumerate(Yfac1)]
        data = [numpy.append(data[i], t) for i, t in enumerate(Yfac2)]
        data = [numpy.append(data[i], t) for i, t in enumerate(Tsys1)]
        data = [numpy.append(data[i], t) for i, t in enumerate(Tsys2)]
        return data

    def ttlplot(self, data, initV, finV, datetime, lo, IF, ch):

        fig = matplotlib.pyplot.figure()
        fig.subplots_adjust(right=0.75)

        # data list --
        x = data[:, ch*2-1]      # SIS bias [mV]
        sisi = data[:, ch*2]     # SIS current [uA]
        hot1 = data[:, 25]
        cold1 = data[:, 51]
        hot2 = data[:, 26]
        cold2 = data[:, 52]
        Tsys1 = data[:, -2]
        Tsys2 = data[:, -1]

        if IF == 1:
            hot = hot1
            cold = cold1
            Tsys = Tsys1
        elif IF == 2:
            hot = hot2
            cold = cold2
            Tsys = Tsys2
        else:
            return

        # define axes --
        ax1 = fig.add_subplot(1, 1, 1)
        t_ax = ax1.twinx()
        p_ax = ax1.twinx()
        p_ax.spines['right'].set_position(('axes', 1.16))

        # plot lines --
        p1, = ax1.plot(x, Tsys, color='green', marker=None, ls='-', label='Tsys')
        p2, = t_ax.plot(x, sisi, color='black', marker=None, ls='-', label='sis-IV')
        p3, = p_ax.plot(x, hot, color='red', marker=None, ls='-', label='HOT')
        p4, = p_ax.plot(x, cold, color='blue', marker=None, ls='-', label='COLD')
        lines = [p1, p2, p3, p4]

        # set labels --
        ax1.set_xlabel('SIS V [mV]')
        ax1.set_ylabel('Trx [K]')
        t_ax.set_ylabel('SIS V [uA]')
        p_ax.set_ylabel('Power')

        # set sisI limits --
        if min(sisi) <= -200: Imin = -200
        else: Imin = min(sisi) - 20
        if 200 <= max(sisi): Imax = 200
        else: Imax = max(sisi) + 20

        ax1.set_xlim(initV, finV)
        ax1.set_ylim(25, 200)      # TODO
        t_ax.set_ylim(Imin, Imax)
        #p_ax.set_ylim(0, 20000)  # TODO

        # set label colors --
        ax1.yaxis.label.set_color('green')
        ax1.tick_params(axis='y', colors='green')
        t_ax.yaxis.label.set_color('black')
        t_ax.tick_params(axis='y', colors='black')
        p_ax.yaxis.label.set_color('red')
        p_ax.tick_params(axis='y', colors='red')

        # set legend --
        ax1.legend(lines, [l.get_label() for l in lines], loc='upper left', ncol=2)

        # others --
        ax1.set_title('SIS V sweep IF{} - ch{} : {}'.format(IF, ch, datetime))
        ax1.grid(which='both', color='gray', ls='--')
        ax1.patch.set_facecolor('lightgray')
        ax1.patch.set_alpha(0.1)

        fig.savefig(self.savedir+'c4sis_vsweep-IF{}-ch{}_lo{}mA_{}.png'.format(IF, ch, lo, datetime))
        fig.show()
        return

    def input_value_check(self, initV, finV, interval):
        Vmix_limit = 30  # [mv]
        if -Vmix_limit <= initV <= finV <= Vmix_limit:
            pass
        else:
            msg = '{0}\n{1}\n{2}'.format('-- Input Invalid Value Error --',
                                         '    !!! Invalid Voltage !!!',
                                         'Available Voltage: -30 -- 30 [mV]')
            raise ValueError(msg)
        repeat = int(abs(initV - finV) / interval)
        return repeat

    def setup_powermeter(self):
        ip1 = '192.168.100.113'
        ip2 = '192.168.100.116'
        port1 = 13
        port2 = 13
        self.pm1 = ML2437A.ml2437a(IP=ip1, GPIB=port1)
        self.pm2 = ML2437A.ml2437a(IP=ip2, GPIB=port2)
        return

