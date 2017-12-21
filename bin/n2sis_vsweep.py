#! /usr/bin/env python

# import modules
# --------------
import time

import numpy
import matplotlib.pyplot

#import equipment_nanten
from NASCORX_System.base import sis


class Vsweep(object):
    method = 'Trx V sweep Measurement'
    ver = '2017.12.18'
    savedir = '/home/amigos/NASCORX_Measurement/SIStune/'

    def __init__(self):
        #self.dfs = equipment_nanten.dfs()
        #self.m4 = equipment_nanten.m4()
        #self.hot = equipment_nanten.hot_load()
        pass

    def run(self, initV=0.0, finV=6.0, interval=0.1, lo=0, integ=0.1, fmt='csv'):

        # Print Welcome massage
        # ---------------------
        print('\n\n'
              ' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n'
              '   N2 RX : SIS Tsys curve Measurement \n'
              ' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n'
              ' ver - {}\n'
              '\n\n'.format(self.ver))

        # Input value check
        # -----------------
        repeat = self.input_value_check(initV=initV, finV=finV, interval=interval)

        # Set Driver
        # ----------
        self.driver = sis.mixer()
        print('PCI board drivers are set.')


        # == Main ========

        # Lo attenuator setting
        # ---------------------
        self.driver.set_loatt(att=lo, ch=0)

        # Measurement part
        # ----------------
        result = self.measure(repeat=repeat, initV=initV, interval=interval, integ=integ)

        # Calculate Y-Factor and Tsys
        # ---------------------------
        result = self.trx_calculate(result=result)

        # Data saving part
        # ----------------
        datetime = time.strftime('%Y%m%d-%H%M')
        filename = self.savedir + 'n2sis_vsweep_lo{}mA_{}.{}'.format(lo, datetime, fmt)
        header = 'DA-V,Vhot,Ihot,Phot1,Phot2,Vcold,Icold,Pcold1,Pcold2,Yfac1,Yfac2,Tsys1,Tsys2'
        numpy.savetxt(filename, result, fmt='%.5f', header=header, delimiter=',')

        # Close sis driver
        # ----------------------------
        self.driver.close_box()

        # Data loading
        # ------------
        data = numpy.loadtxt(filename, skiprows=1, delimiter=',')

        # Plot part
        # ---------
        self.ttlplot(data=data, initV=initV, finV=finV, datetime=datetime, lo=lo)
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

    def measure(self, repeat, initV=0.0, interval=0.1, ch=1, integ=0.1):
        # HOT measurement
        # ---------------
        # TODO : HOT IN
        data_hot = self.sweep_sisv(repeat, initV=initV, interval=interval, integ=integ, pow=3000, col0=True)

        # COLD measurement
        # ----------------
        # TODO : HOT OUT, OBS SKY
        data_sky = self.sweep_sisv(repeat, initV=initV, interval=interval, integ=integ, pow=1400, col0=False)

        # data arrangement
        # ----------------
        hot_arr = numpy.array(data_hot)
        sky_arr = numpy.array(data_sky)
        ret = numpy.concatenate((hot_arr, sky_arr), axis=1)
        return ret

    def sweep_sisv(self, repeat, initV=0.0, interval=0.1, integ=0.1, pow=10., col0=True):
        result = []

        for i in range(repeat+1):
            temp = []
            setV = initV + i * interval

            # bias set
            # --------
            self.driver.set_sisv(Vmix=setV, ch=0)

            time.sleep(0.2)

            # data receive
            # ------------
            # spec = self.client.oneshot(integtime=integ, repeat=1, start=None)
            #spec = self.dfs.oneshot(1, integ, 0)
            ad = self.driver.monitor_sis()

            # data arrangement
            # ----------------
            if col0 is True: temp.append(setV)
            temp.append(ad[0]*1e+1)      # AD[V] --> bias [mV]
            temp.append(ad[1]*1e+3)    # AD[V] --> current [uA]
            #power = numpy.sum(spec, axis=1)
            temp.append(pow)
            temp.append(pow)
            result.append(temp)
        return result

    def trx_calculate(self, result, Tamb=300):
        # dB Y-factor --
        Yfac1 = 10*numpy.log10(result[:, 3]/result[:, 7])
        Yfac2 = 10*numpy.log10(result[:, 4]/result[:, 8])

        # Tsys calculation --
        Tsys1 = (Tamb*result[:, 7]) / (result[:, 3]-result[:, 7])
        Tsys2 = (Tamb*result[:, 8]) / (result[:, 4]-result[:, 8])

        # data arrangement --
        result = [numpy.append(result[i], t) for i, t in enumerate(Yfac1)]
        result = [numpy.append(result[i], t) for i, t in enumerate(Yfac2)]
        result = [numpy.append(result[i], t) for i, t in enumerate(Tsys1)]
        result = [numpy.append(result[i], t) for i, t in enumerate(Tsys2)]
        return result

    def ttlplot(self, data, initV, finV, lo, datetime):
        fig = matplotlib.pyplot.figure()
        fig.subplots_adjust(right=0.75)

        # data list --
        x = data[:, 1]      # SIS bias [mV]
        sisi = data[:, 2]   # SIS current [uA]
        hot1 = data[:, 3]
        cold1 = data[:, 7]
        hot2 = data[:, 4]
        cold2 = data[:, 8]
        Tsys1 = data[:, 11]
        Tsys2 = data[:, 12]

        # define axes --
        ax1 = fig.add_subplot(1, 1, 1)
        t_ax = ax1.twinx()
        p_ax = ax1.twinx()
        p_ax.spines['right'].set_position(('axes', 1.16))

        # plot lines --
        p1, = ax1.plot(x, Tsys1, color='green', marker=None, ls='-', label='Tsys')
        p2, = t_ax.plot(x, sisi, color='black', marker=None, ls='-', label='sis-IV')
        p3, = p_ax.plot(x, hot1, color='red', marker=None, ls='-', label='HOT')
        p4, = p_ax.plot(x, cold1, color='blue', marker=None, ls='-', label='sky')
        lines = [p1, p2, p3, p4]

        # set labels --
        ax1.set_xlabel('SIS V [mV]')
        ax1.set_ylabel('Tsys [K]')
        t_ax.set_ylabel('SIS V [uA]')
        p_ax.set_ylabel('Count')

        # set sisI limits --
        if min(sisi) <= -200: Imin = -200
        else: Imin = min(sisi) - 20
        if 200 <= max(sisi): Imax = 200
        else: Imax = max(sisi) + 20

        ax1.set_xlim(initV, finV)
        ax1.set_ylim(50, 400)      # TODO
        t_ax.set_ylim(Imin, Imax)
        p_ax.set_ylim(0, 20000)  # TODO

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
        ax1.set_title('SIS tuning : V sweep IF1 - {}'.format(datetime))
        ax1.grid(which='both', color='gray', ls='--')
        ax1.patch.set_facecolor('lightgray')
        ax1.patch.set_alpha(0.1)

        fig.savefig(self.savedir+'n2sis_vsweep_lo{}mA_{}.png'.format(lo, datetime))
        fig.show()
        return


if __name__ == '__main__':
    vs = Vsweep()
    vs.run(initV=0.0, finV=2.0)


# History
# -------
# 2017/12/18 : written by T.Inaba
