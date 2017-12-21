#! /usr/bin/env python

# import modules
import time
import numpy
import matplotlib.pyplot

from NASCORX_System.base import sis


class IV_curve(object):
    method = 'IV Curve Measurement'
    ver = '2017.12.18'
    savedir = '/home/amigos/NASCORX_Measurement/SIStune/'
    switch = False

    # parameters --
    initV = - 8.0
    finV = 8.0
    interval = 0.1
    onoff = [0] * 2
    lo = [0] * 2

    def __init__(self):
        #self.loop()
        pass

    def loop(self):
        while True:
            if self.switch:
                self.run(self.initV, self.finV, self.interval, self.lo, self.onoff)
                self.switch = False
            else:
                pass
        return

    def switch_subscriber(self, req):
        self.switch = True
        return

    def args_subscriber(self, req):
        self.initV = req.initV
        self.finV = req.finV
        self.interval = req.interval
        self.lo = req.lo
        self.onoff = req.onoff
        return

    def run(self, initV=-8.0, finV=8.0, interval=0.1, lo=0, filefmt='csv'):

        # Print Welcome massage
        # ---------------------
        print('\n\n'
              ' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n'
              '   N2 RX : SIS IV curve Measurement \n'
              ' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n'
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

        # Lo setting
        # ----------
        self.driver.set_loatt(att=lo, ch=0)

        # Measurement part
        # ----------------
        AD_data = self.measure(repeat=repeat, initV=initV, interval=interval)

        # Data saving part
        # ----------------
        datetime = time.strftime('%Y%m%d-%H%M')
        filename = self.savedir + 'n2sisIV_lo{}mA_{}.{}'.format(lo, datetime, filefmt)
        header = 'D/A-SISV,1L-V,1L-I,1R-V,1R-I'
        numpy.savetxt(filename, AD_data, fmt='%.5f', header=header, delimiter=',')

        # Close sis driver
        # ----------------
        self.driver.close_box()

        # Data loading
        # ------------
        AD_data = numpy.loadtxt(filename, skiprows=1, delimiter=',')

        # Plot part
        # ---------
        self.ttlplot(AD_data=AD_data, initV=initV, finV=finV, lo=lo, datetime=datetime)

        # Output information on terminal
        # ------------------------------
        print('\n'
              ' ======== SIS IV Curve MEASUREMENT ========\n'
              ' Time Stamp    : {}\n'
              ' Start SISV    : {} [mV]\n'
              ' Finish SISV   : {} [mV]\n'
              ' Lo Attenuation: {} [mA]\n'.format(datetime, initV, finV, lo))
        return

    def input_value_check(self, initV, finV, interval):
        Vmix_limit = 30  # [mv]
        if -Vmix_limit <= initV <= finV <= Vmix_limit:
            pass
        else:
            msg = '{0}\n{1}\n{2}'.format('-- Invalid Input Value Error --',
                                         '    !!! Invalid Voltage !!!',
                                         'Available Voltage: -30 -- 30 [mV]')
            raise ValueError(msg)
        repeat = int(abs(initV - finV) / interval)
        return repeat

    def divplot(self, AD_data, ch, initV, finV, datetime):
        fig = matplotlib.pyplot.figure()
        ax = fig.add_subplot(1, 1, 1)
        label = '230GHz-SIS'
        ax.plot(AD_data[:, 1+2*(ch-1)], AD_data[:, 2+2*(ch-1)], label=label)
        ax.set_xlabel('SIS V [mV]')
        ax.set_ylabel('SIS I [uA]')
        ax.set_xlim([initV, finV])
        ax.legend(loc='upper left', prop={'size': 7})
        ax.grid(color='gray', linestyle='--')

        fig.tight_layout()
        # fig.subplots_adjust(top=0.9)
        figname = self.savedir + 'n2sisIV_ch{}_{}.png'.format(ch, datetime)
        fig.savefig(figname)
        fig.show()
        return

    def ttlplot(self, AD_data, initV, finV, lo, datetime):
        fig = matplotlib.pyplot.figure(figsize=(9, 7))
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(AD_data[:, 1], AD_data[:, 2], label='sis IV')
        ax.set_xlim([initV, finV])
        ax.set_ylim([-10, 500])
        ax.set_xlabel('SIS V [mV]')
        ax.set_ylabel('SIS I [uA]')
        ax.legend(loc='upper left', prop={'size': 7}, numpoints=1)
        ax.grid(color='gray', linestyle='--')

        fig.tight_layout()
        fig.subplots_adjust(top=0.9)
        fig.suptitle('SIS IV Curve Measurement: {}'.format(datetime), fontsize=15)
        figname = self.savedir + 'n2sisIV_lo{}mA_{}.png'.format(lo, datetime)
        fig.savefig(figname)
        fig.show()
        return

    def measure(self, repeat, initV=0.0, interval=0.1):
        AD_data = []

        # Measurement part
        # ----------------
        for i in range(repeat+1):
            temp = []
            setV = initV + i * interval
            self.driver.set_sisv(Vmix=setV, ch=0)
            time.sleep(0.2)
            ret = self.driver.monitor_sis()
            temp.append(setV)

            # scaling
            # -------
            for j in range(4):
                if j % 2 == 0:
                    temp.append(ret[j]*1e+1)  # AD[V] --> SIS bias [mV]
                elif j % 2 == 1:
                    temp.append(ret[j]*1e+3)  # AD[V] --> SIS current [uA]
            AD_data.append(temp)
        return AD_data

if __name__ == '__main__':
    iv = IV_curve()
    iv.run(initV=0, finV=3)


"""
if __name__ == '__main__':
    import rospy
    from std_msgs.msg import String

    rospy.init_node('SIS-IV')
    rospy.loginfo('ROS_sis-iv Start')
    iv = IV_curve()
    rospy.Subscriber("sis-iv_args", String, iv.args_subscriber)
    rospy.Subscriber("sis-iv_switch", String, iv.run)
    rospy.spin()
"""

# History
# -------
# 2017/12/18 : written by T.Inaba
