#! /usr/bin/env python

# import modules
import time
import numpy
import matplotlib.pyplot

from NASCORX_System.base import sis
from NASCORX_System.device import ML2437A
from NASCORX_System.base import Motor

class Losweep(object):
    method = 'Lo sweep Measurement'
    ver = '2017.12.22'
    savedir = '/home/amigos/NASCORX_Measurement/1223/'

    def __init__(self):
        pass

    def run(self, initI=0.0, finI=2.0, interval=0.05, integ=0.1, hemt=None):

        # Print Welcome massage
        # ----------------------
        print('\n\n'
              '=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'
              'NASCO RX : Lo Sweep Measurement'
              '=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'
              'ver{}'
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
        sisv1 = float(input(' IF1 - SIS bias [mV] ?  : '))
        sisv2 = float(input(' IF2 - SIS bias [mV] ?  : '))

        # input value check
        # -----------------
        repeat = self.input_value_check(initI=initI, finI=finI, interval=interval)

        # create setV-list
        # ----------------
        setVlist = [0]*12
        setVlist[ch1-1] = sisv1
        setVlist[ch2-1] = sisv2

        # Set Driver
        # ----------
        self.driver = sis.multi_box()
        print('PCI board drivers are set.')

        # set PowerMeter
        # --------------
        self.setup_powermeter()

        # == Main ========

        # HEMT setting
        # ------------
        if hemt is True:
            self.driver.set_Vd(voltage=[1.2]*8)
            self.driver.set_Vg1(voltage=[0.5]*8)
            self.driver.set_Vg2(voltage=[-0.5]*8)
        elif hemt is None: pass

        # Lo setting
        # ----------
        if lo is None: pass
        else:
            _ = [50]*10
            self.driver.set_loatt(att=_)
            input('If setting the SG, input Y')

        # SIS bias setting
        # ----------------
        self.driver.set_sisv(Vmix=setVlist)

        # Measurement part
        # ----------------
        result = self.measure(repeat=repeat, initI=initI, interval=interval, integ=integ)

        # Calculate Y-Factor and Tsys
        # ---------------------------
        result = self.trx_calculate(data=result)

        # Data saving part
        # ----------------
        datetime = time.strftime('%Y%m%d-%H%M')
        filename = self.savedir + 'c4sis_losweep_{}.csv'.format(datetime)
        header = 'Loatt,' \
                 '2L-V,2L-I,2R-V,2R-I,3L-V,3L-I,3R-V,3R-I,4L-V,4L-I,4R-V,4R-I,5L-V,5L-I,5R-V,5R-I,' \
                 '1LU-V,1LU-I,1LL-V,1LL-I,1RU-V,1RU-I,1RL-V,1RL-I,Phot1,Phot2,' \
                 '2L-V,2L-I,2R-V,2R-I,3L-V,3L-I,3R-V,3R-I,4L-V,4L-I,4R-V,4R-I,5L-V,5L-I,5R-V,5R-I,' \
                 '1LU-V,1LU-I,1LL-V,1LL-I,1RU-V,1RU-I,1RL-V,1RL-I,Pcold1,Pcold2,Yfac1,Yfac2,Trx1,Trx2'
        numpy.savetxt(filename, result, fmt='%.5f', header=header, delimiter=',')

        # Close sis driver
        # ----------------------------
        self.driver.close_box()

        # Data loading
        # ------------
        data = numpy.loadtxt(filename, skiprows=1, delimiter=',')

        # Plot part
        # ---------
        self.ttlplot(data=data, initI=initI, finI=finI, datetime=datetime, sisv=sisv1, IF=1, ch=ch1)
        self.ttlplot(data=data, initI=initI, finI=finI, datetime=datetime, sisv=sisv2, IF=2, ch=ch2)
        return


    def input_value_check(self, initI=0.0, finI=2.0, interval=0.05):
        if 0 <= initI <= finI <= 100:
            pass
        else:
            msg = '{0}\n{1}\n{2}'.format('-- Input Invalid Value Error --',
                                         '    !!! Invalid Current !!!',
                                         'Available Voltage: 0 -- 100 [mA]')
            raise ValueError(msg)
        repeat = int(abs(initI - finI) / interval)
        return repeat

    def measure(self, repeat, initI=2.0, interval=0.05, integ=0.1):
        # HOT measurement
        # ---------------
        # TODO : HOT IN
        data_hot = self.sweep_lo(repeat, initI=initI, interval=interval, index=True)

        # HOT --> COLD
        # ---------------
        self.chopper = Motor.chopper()
        self.chopper.rot_chopper()

        # COLD measurement
        # ----------------
        # TODO : HOT OUT, OBS SKY
        data_sky = self.sweep_lo(repeat, initI=initI, interval=interval, index=True)

        # data arrangement
        # ----------------
        hot_arr = numpy.array(data_hot)
        sky_arr = numpy.array(data_sky)
        ret = numpy.concatenate((hot_arr, sky_arr), axis=1)
        return ret

    def sweep_lo(self, repeat, initI=2.0, interval=0.05, index=True, pow=10):
        result = []

        for i in range(repeat+1):
            temp = []
            setI = [initI + i * interval] * 10

            # bias set
            # --------
            self.driver.set_loatt(att=setI)

            time.sleep(0.1)

            # data receive
            # ------------
            pow1 = self.pm1.measure()            # PowerMeter1
            pow2 = self.pm2.measure()            # PowerMeter2
            ad = self.driver.monitor_sis()

            # data arrangement
            # ----------------
            if index is True:
                temp.append(setI)
            # AD data scaling --
            for j in range(24):
                if j % 2 == 0:
                    temp.append(ad[j]*1e+1)      # AD[V] --> bias [mV]
                elif j % 2 == 1:
                    temp.append(ad[j]*1e+3)      # AD[V] --> current [uA]
            temp.append(pow1)
            temp.append(pow2)
            result.append(temp)
        return result

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

    def ttlplot(self, data, initI, finI, datetime, sisv, IF, ch):
        fig = matplotlib.pyplot.figure()
        fig.subplots_adjust(right=0.75)

        # data list --
        x = data[:, 0]      # SIS bias [mV]
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

        ax1.set_xlim(initI, finI)
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

        fig.savefig(self.savedir+'c4sis_losweep-IF{}-ch{}_sisv{}mV_{}.png'.format(IF, ch, sisv, datetime))
        fig.show()
        return


    def setup_powermeter(self):
        ip1 = '192.168.100.113'
        ip2 = '192.168.100.116'
        port1 = 13
        port2 = 13
        self.pm1 = ML2437A.ml2437a(IP=ip1, GPIB=port1)
        self.pm2 = ML2437A.ml2437a(IP=ip2, GPIB=port2)
        return



# History
# -------
# 2017/12/18 : written by T.Inaba
