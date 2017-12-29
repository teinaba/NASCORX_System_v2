#! /usr/bin/env python

# import modules
import os
import sys
import time
import numpy
import threading
import matplotlib.pyplot

sys.path.append('/home/necst/ros/src/NASCORX')
from NASCORX_System.base import sis


class IV_curve(object):
    method = 'IV Curve Measurement'
    ver = '2017.12.28'
    savedir = '/home/necst/data/experiment/rx'
    switch = False
    driver = None

    # parameters --
    initv = - 8.0
    finv = 8.0
    interval = 0.1
    lo = 0.0

    def __init__(self):
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        pass

    def loop(self):
        while True:
            if self.switch:
                time.sleep(0.1)
                self.run(self.initv, self.finv, self.interval, self.lo)
                self.switch = False
            else:
                pass
            time.sleep(0.1)
        return

    def subscriber(self, req):
        self.switch = req.switch
        self.initv = req.initv
        self.finv = req.finv
        self.interval = req.interval
        self.lo = req.lo
        return

    def run(self, initv=-8.0, finv=8.0, interval=0.1, lo=0.0):

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
        repeat = self.input_value_check(initv=initv, finv=finv, interval=interval)

        # Set Driver
        # ----------
        self.driver = sis.mixer()
        print('PCI board drivers are set.')

        # Lo setting
        # ----------
        self.driver.set_loatt(att=lo, ch=0)

        # Measurement part
        # ----------------
        ad_data = self.measure(repeat=repeat, initv=initv, interval=interval)

        # Data saving part
        # ----------------
        datetime = time.strftime('%Y%m%d-%H%M')
        filepath = os.path.join(self.savedir, 'n2sisIV_lo{}mA_{}.csv'.format(lo, datetime))
        header = 'D/A-SISV,1L-V,1L-I,1R-V,1R-I'
        numpy.savetxt(filepath, ad_data, fmt='%.5f', header=header, delimiter=',')

        # Close sis driver
        # ----------------
        self.driver.close_box()

        # Data loading
        # ------------
        ad_data = numpy.loadtxt(filepath, skiprows=1, delimiter=',')

        # Plot part
        # ---------
        self.ttlplot(ad_data=ad_data, initv=initv, finv=finv, lo=lo, datetime=datetime)

        # Output information on terminal
        # ------------------------------
        print('\n'
              ' ======== SIS IV Curve MEASUREMENT ========\n'
              ' Time Stamp    : {}\n'
              ' Start SISV    : {} [mV]\n'
              ' Finish SISV   : {} [mV]\n'
              ' Lo Attenuation: {} [mA]\n'.format(datetime, initv, finv, lo))
        return

    def input_value_check(self, initv, finv, interval):
        vmix_limit = 30  # [mv]
        if -vmix_limit <= initv <= finv <= vmix_limit:
            pass
        else:
            msg = '{0}\n{1}\n{2}'.format('-- Invalid Input Value Error --',
                                         '    !!! Invalid Voltage !!!',
                                         'Available Voltage: -30 -- 30 [mV]')
            raise ValueError(msg)
        repeat = int(abs(initv - finv) / interval)
        return repeat

    def ttlplot(self, ad_data, initv, finv, lo, datetime):
        fig = matplotlib.pyplot.figure(figsize=(9, 7))
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(ad_data[:, 1], ad_data[:, 2], label='sis IV')
        ax.set_xlim([initv, finv])
        ax.set_ylim([-10, 500])
        ax.set_xlabel('SIS V [mV]')
        ax.set_ylabel('SIS I [uA]')
        ax.legend(loc='upper left', prop={'size': 7}, numpoints=1)
        ax.grid(color='gray', linestyle='--')

        fig.tight_layout()
        fig.subplots_adjust(top=0.9)
        fig.suptitle('SIS IV Curve Measurement: {}'.format(datetime), fontsize=15)
        figpath = os.path.join(self.savedir, 'n2sisIV_lo{}mA_{}.png'.format(lo, datetime))
        fig.savefig(figpath)
        return

    def measure(self, repeat, initv=0.0, interval=0.1):
        ad_data = []

        # Measurement part
        # ----------------
        for i in range(repeat+1):
            temp = []
            setv = initv + i * interval
            self.driver.set_sisv(Vmix=setv, ch=0)
            time.sleep(0.2)
            ret = self.driver.monitor_sis()
            temp.append(setv)

            # scaling
            # -------
            for j in range(4):
                if j % 2 == 0:
                    temp.append(ret[j]*1e+1)  # AD[V] --> SIS bias [mV]
                elif j % 2 == 1:
                    temp.append(ret[j]*1e+3)  # AD[V] --> SIS current [uA]
            ad_data.append(temp)
        return ad_data


if __name__ == '__main__':
    import rospy
    from NASCORX.msg import sisiv_msg

    rospy.init_node('sis_iv')
    rospy.loginfo('ROS_sis-iv Start')
    iv = IV_curve()
    rospy.Subscriber("sis_iv", sisiv_msg, iv.subscriber)
    rospy.spin()

# History
# -------
# 2017/12/18 : written by T.Inaba
# 2017/12/28 T.Inaba : (1) add ROS method (2) delete divplot method.
