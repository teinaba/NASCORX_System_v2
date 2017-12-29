#! /usr/bin/env python

# import modules
import os
import time
import numpy
import threading
import matplotlib.pyplot

from NASCORX_System.base import sis


class Losweep(object):
    method = 'Lo sweep Measurement'
    ver = '2017.12.28'
    savedir = '/home/necst/data/experiment/rx'
    switch = False
    driver = None

    # parameters --
    initi = - 8.0  # initial current
    fini = 8.0     # finish current
    interval = 0.1
    sisv = 0.0

    def __init__(self):
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        pass

    def loop(self):
        while True:
            if self.switch:
                time.sleep(0.1)
                self.run(self.initi, self.fini, self.interval, self.sisv)
                self.switch = False
            else:
                pass
            time.sleep(0.1)
        return

    def subscriber(self, req):
        self.switch = req.switch
        self.initi = req.initi
        self.fini = req.fini
        self.interval = req.interval
        self.sisv = req.sisv
        return

    def run(self, initi=0.0, fini=2.0, interval=0.05, sisv=0.0, integ=0.1):

        # Print Welcome massage
        # ----------------------
        print('\n\n'
              '=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'
              'NASCO RX : Lo Sweep Measurement'
              '=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'
              'ver{}'
              '\n\n'.format(self.ver))

        # input value check
        # -----------------
        repeat = self.input_value_check(initi=initi, fini=fini, interval=interval)

        # Set Driver
        # ----------
        self.driver = sis.mixer()
        print('PCI board drivers are set.')

        # SIS bias setting
        # ----------------
        self.driver.set_sisv(Vmix=sisv, ch=0)

        # Measurement part
        # ----------------
        result = self.measure(repeat=repeat, initi=initi, interval=interval, integ=integ)

        # Calculate Y-Factor and Tsys
        # ---------------------------
        result = self.trx_calculate(result=result)

        # Data saving part
        # ----------------
        datetime = time.strftime('%Y%m%d-%H%M')
        filepath = os.path.join(self.savedir, 'n2sis_losweep_sisv{}mV_{}.csv'.format(sisv, datetime))
        header = 'Loatt,Vhot,Ihot,Phot1,Phot2,Vcold,Icold,Pcold1,Pcold2,Yfac1,Yfac2,Tsys1,Tsys2'
        numpy.savetxt(filepath, result, fmt='%.5f', header=header, delimiter=',')

        # Close sis driver
        # ----------------------------
        self.driver.close_box()

        # Data loading
        # ------------
        data = numpy.loadtxt(filepath, skiprows=1, delimiter=',')

        # Plot part
        # ---------
        self.ttlplot(data=data, initi=initi, fini=fini, datetime=datetime, sisv=sisv)
        return


    def input_value_check(self, initi=2.0, fini=5.0, interval=0.05):
        if 0 <= initi <= fini <= 100:
            pass
        else:
            msg = '{0}\n{1}\n{2}'.format('-- Input Invalid Value Error --',
                                         '    !!! Invalid Current !!!',
                                         'Available Voltage: 0 -- 100 [mA]')
            raise ValueError(msg)
        repeat = int(abs(initi - fini) / interval)
        return repeat

    def measure(self, repeat, initi=2.0, interval=0.05, integ=0.1):
        # HOT measurement
        # ---------------
        # TODO : HOT IN
        data_hot = self.sweep_lo(repeat, initi=initi, interval=interval, integ=integ, col0=True, pow=3000)

        # COLD measurement
        # ----------------
        # TODO : HOT OUT, OBS SKY
        data_sky = self.sweep_lo(repeat, initi=initi, interval=interval, integ=integ, col0=False, pow=1500)

        # data arrangement
        # ----------------
        hot_arr = numpy.array(data_hot)
        sky_arr = numpy.array(data_sky)
        ret = numpy.concatenate((hot_arr, sky_arr), axis=1)
        return ret

    def sweep_lo(self, repeat, initi=2.0, interval=0.05, integ=0.1, col0=True, pow=10):
        result = []

        for i in range(repeat+1):
            temp = []
            setI = initi + i * interval

            # bias set
            # --------
            self.driver.set_loatt(att=setI, ch=0)
            time.sleep(0.2)

            # data receive
            # ------------
            # spec = self.client.oneshot(integtime=integ, repeat=1, start=None)
            # spec = self.dfs.oneshot(1, integ, 0)
            ad = self.driver.monitor_sis()

            # data arrangement
            # ----------------
            if col0 is True : temp.append(setI)
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

    def ttlplot(self, data, initi, fini, sisv, datetime):
        fig = matplotlib.pyplot.figure()
        fig.subplots_adjust(right=0.75)

        # data list --
        x = data[:, 0]      # Lo att [mA]
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
        p2, = t_ax.plot(x, sisi, color='black', marker=None, ls='-', label='SIS I')
        p3, = p_ax.plot(x, hot1, color='red', marker=None, ls='-', label='HOT')
        p4, = p_ax.plot(x, cold1, color='blue', marker=None, ls='-', label='sky')
        lines = [p1, p2, p3, p4]

        # set labels --
        ax1.set_xlabel('Lo att [mA]')
        ax1.set_ylabel('Tsys [K]')
        t_ax.set_ylabel('SIS I [uA]')
        p_ax.set_ylabel('Count')

        # set limits --
        if min(sisi) <= -200: Imin = -200
        else: Imin = min(sisi) - 20
        if 200 <= max(sisi): Imax = 200
        else: Imax = max(sisi) + 20

        ax1.set_xlim(initi, fini)
        ax1.set_ylim(50, 400)
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
        ax1.set_title('SIS tuning : Lo sweep IF1 - {}')
        ax1.grid(which='both', color='gray', ls='--')
        ax1.patch.set_facecolor('lightgray')
        ax1.patch.set_alpha(0.1)
        figpath = os.path.join(self.savedir, 'n2sis_losweep_sisv{}mV_{}.png'.format(sisv, datetime))
        fig.savefig(figpath)
        return


if __name__ == '__main__':
    import rospy
    from NASCORX.msg import sislosweep_msg

    rospy.init_node('sis_losweep')
    rospy.loginfo('ROS_sis-vlosweep Start')
    lo = Losweep()
    rospy.Subscriber("sis_losweep", sislosweep_msg, lo.subscriber)
    rospy.spin()


# History
# -------
# 2017/12/18 : written by T.Inaba
# 2017/12/28 T.Inaba : (1) add ROS method.
