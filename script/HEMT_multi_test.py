#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


# import modules
import time
import numpy
import matplotlib.pyplot as plt
from NASCORX_System.base import sis


class Hemt_multi_test(object):
    savedir = "/home/amigos/NASCORX_Measurement/HEMT/"

    def __init__(self):
        pass

    def get_datetime(self):
        datetime = time.strftime('%Y%m%d-%H%M%S')
        return datetime

    # +-+-+-+-+-+-+-+-+-+-+-+
    #  Measurement functions
    # +-+-+-+-+-+-+-+-+-+-+-+

    def swp_vd_meas(self):
        # opening procedure
        self.mhemt = sis.multi_hemt()
        AD_data = []

        # measurement part
        for i in range(21):
            # 0-2.0 [mV]
            setVd = 0.1 * i
            setVd_list = [setVd] * 8
            self.mhemt.set_Vd(voltage=setVd_list)
            time.sleep(0.5)
            ret = self.mhemt.monitor_hemt()  # data = 4*8 -> 1-32ch
            ret.insert(0, setVd)
            AD_data.append(ret[0:33])

        # closing procedure
        self.mhemt.close_box()

        # saving data
        datetime = time.strftime('%Y%m%d-%H%M%S')
        filename = self.savedir + 'Hemt_multi_test_SwpVD_{}.csv'.format(datetime)
        # sorry for long string
        header = 'DA-Vd,' \
                 '1L-VD,1L-Vg1,1L-Vg2,1L-ID,1R-VD,1R-Vg1,1R-Vg2,1R-ID,' \
                 '2L-VD,2L-Vg1,2L-Vg2,2L-ID,2R-VD,2R-Vg1,2R-Vg2,2R-ID,' \
                 '3L-VD,3L-Vg1,3L-Vg2,3L-ID,3R-VD,3R-Vg1,3R-Vg2,3R-ID,' \
                 '4L-VD,4L-Vg1,4L-Vg2,4L-ID,4R-VD,4R-Vg1,4R-Vg2,4R-ID'
        numpy.savetxt(filename, AD_data, fmt='%.5f', header=header, delimiter=",")

        # output on terminal
        print('\n'
              '========  HEMT Sweep Vd MEASUREMENT ========\n'
              'Time Stamp : {}\n'
              'Beam 1-2   : \n'
              'Beam 3-4   : \n'.format(datetime))

        # 取り急ぎ
        AD_data = numpy.loadtxt(filename, skiprows=1, delimiter=",")

        # plot part
        fig = plt.figure(figsize=(9, 9))
        ## list comprehensionだとかっこいいけど、まあいっか

        ## figure-1: Beam 1-2: setVd vs monVd
        ax1 = fig.add_subplot(2, 2, 1)
        ax1.plot(AD_data[:, 0], AD_data[:, 1], label='1LCP', ls='-', marker='.')
        ax1.plot(AD_data[:, 0], AD_data[:, 5], label='1RCP', ls='-', marker='.')
        ax1.plot(AD_data[:, 0], AD_data[:, 9], label='2LCP', ls='-', marker='.')
        ax1.plot(AD_data[:, 0], AD_data[:, 13], label='2RCP', ls='-', marker='.')
        ax1.set_xlabel('Drain Voltage (D/A) [V]')
        ax1.set_ylabel('Drain Voltage (Monitor) [V]')
        ax1.set_title("Beam 1-2: Vd D/A vs. Vd Monitor")
        ax1.legend(loc='upper left', prop={'size': 9}, numpoints=1)
        ax1.grid()

        ## figure-2: Beam 1-2: monVd vs monID
        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(AD_data[:, 1], AD_data[:, 4], label='1LCP', ls='-', marker='.')
        ax2.plot(AD_data[:, 5], AD_data[:, 8], label='1RCP', ls='-', marker='.')
        ax2.plot(AD_data[:, 9], AD_data[:, 12], label='2LCP', ls='-', marker='.')
        ax2.plot(AD_data[:, 13], AD_data[:, 16], label='2RCP', ls='-', marker='.')
        ax2.set_xlabel('Drain Voltage (Monitor) [V]')
        ax2.set_ylabel('Drain Current (Monitor) [A]')
        ax2.set_title("Beam 1-2: Vd monitor vs. Drain Current")
        ax2.legend(loc='upper left', prop={'size': 9}, numpoints=1)
        ax2.grid()

        ## figure-3: Beam 3-4: setVd vs monVd
        ax3 = fig.add_subplot(2, 2, 3)
        ax3.plot(AD_data[:, 0], AD_data[:, 17], label='3LCP', ls='-', marker='.')
        ax3.plot(AD_data[:, 0], AD_data[:, 21], label='3RCP', ls='-', marker='.')
        ax3.plot(AD_data[:, 0], AD_data[:, 25], label='4LCP', ls='-', marker='.')
        ax3.plot(AD_data[:, 0], AD_data[:, 29], label='4RCP', ls='-', marker='.')
        ax3.set_xlabel('Drain Voltage (D/A) [V]')
        ax3.set_ylabel('Drain Voltage (Monitor) [V]')
        ax3.set_title("Beam 3-4: Vd D/A vs. Vd Monitor")
        ax3.legend(loc='upper left', prop={'size': 9}, numpoints=1)
        ax3.grid()

        ## figure-4: Beam 3-4: monVd vs monID
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.plot(AD_data[:, 17], AD_data[:, 20], label='3LCP', ls='-', marker='.')
        ax4.plot(AD_data[:, 21], AD_data[:, 24], label='3RCP', ls='-', marker='.')
        ax4.plot(AD_data[:, 25], AD_data[:, 28], label='4LCP', ls='-', marker='.')
        ax4.plot(AD_data[:, 29], AD_data[:, 32], label='4RCP', ls='-', marker='.')
        ax4.set_xlabel('Drain Voltage (Monitor) [V]')
        ax4.set_ylabel('Drain Current (Monitor) [10 mA]')
        ax4.set_title("Beam 3-4: Vd monitor vs. Drain Current")
        ax4.legend(loc='upper left', prop={'size': 9}, numpoints=1)
        ax4.grid()

        # list comprehension
        # ax = [fig.add_subplot(2, 2, i+1) for i in range(4)]

        fig.tight_layout()
        fig.subplots_adjust(top=0.9)
        fig.suptitle('HEMT Multi test: Swp Vd Measurement -- Datetime:{0}'.format(datetime), fontsize=14)
        # if needed
        figname = self.savedir + 'Hemt_multi_test_SwpVD_{}.png'.format(datetime)
        plt.savefig(figname)
        plt.show()

        return

    def swp_vg1_meas(self):
        # opening procedure
        self.mhemt = sis.multi_hemt()
        AD_data = []

        # measurement part
        for i in range(51):
            # -2.5- -2.5 [mV]
            set_Vg1 = - 2.5 + 0.1 * i
            set_Vg1_list = [set_Vg1] * 8
            self.mhemt.set_Vg1(voltage=set_Vg1_list)
            time.sleep(0.5)
            ret = self.mhemt.monitor_hemt()  # data = 4*8 -> 1-32ch
            ret.insert(0, set_Vg1)
            AD_data.append(ret[0:33])

        # closing procedure
        self.mhemt.close_box()

        # saving data
        datetime = time.strftime('%Y%m%d-%H%M%S')
        filename = self.savedir + 'Hemt_multi_test_SwpVg1_{}.csv'.format(datetime)
        # sorry for long string
        header = 'DA-Vg1' \
                 '1L-VD,1L-Vg1,1L-Vg2,1L-ID,1R-VD,1R-Vg1,1R-Vg2,1R-ID,' \
                 '2L-VD,2L-Vg1,2L-Vg2,2L-ID,2R-VD,2R-Vg1,2R-Vg2,2R-ID,' \
                 '3L-VD,3L-Vg1,3L-Vg2,3L-ID,3R-VD,3R-Vg1,3R-Vg2,3R-ID,' \
                 '4L-VD,4L-Vg1,4L-Vg2,4L-ID,4R-VD,4R-Vg1,4R-Vg2,4R-ID,'
        numpy.savetxt(fname=filename, X=AD_data, fmt='%.5f', header=header, delimiter=",")

        # output on terminal
        print('\n'
              '========  HEMT Sweep Vg1 MEASUREMENT ========\n'
              'Time Stamp  : {}\n'
              'Beam 1-2  : \n'
              'Beam 3-4  : \n'.format(datetime))

        # 取り急ぎ
        AD_data = numpy.loadtxt(filename, skiprows=1, delimiter=",")

        # plot part
        fig = plt.figure(figsize=(10, 5))
        ## figure-1: Beam 1-2 - setVg1 vs monVg1
        ax1 = fig.add_subplot(1, 2, 1)
        ax1.plot(AD_data[:, 0], AD_data[:, 2], label='1LCP', ls='-', marker='.')
        ax1.plot(AD_data[:, 0], AD_data[:, 6]+0.2, label='1RCP+0.2', ls='-', marker='.')
        ax1.plot(AD_data[:, 0], AD_data[:, 10]+0.4, label='2LCP+0.4', ls='-', marker='.')
        ax1.plot(AD_data[:, 0], AD_data[:, 14]+0.6, label='2RCP+0.6', ls='-', marker='.')
        ax1.set_xlabel('Gate1 Voltage (D/A) [V]')
        ax1.set_ylabel('Gate1 Voltage (Monitor) [V]')
        ax1.set_title("Beam 1-2: Vg1 D/A vs. Vg1 Monitor")
        ax1.legend(loc='upper left', prop={'size': 9}, numpoints=1)
        ax1.grid()

        ## figure-1: Beam 3-4 - setVg1 vs monVg1
        ax1 = fig.add_subplot(1, 2, 2)
        ax1.plot(AD_data[:, 0], AD_data[:, 18], label='3LCP', ls='-', marker='.')
        ax1.plot(AD_data[:, 0], AD_data[:, 22]+0.2, label='3RCP+0.2', ls='-', marker='.')
        ax1.plot(AD_data[:, 0], AD_data[:, 26]+0.4, label='4LCP+0.4', ls='-', marker='.')
        ax1.plot(AD_data[:, 0], AD_data[:, 30]+0.6, label='4RCP+0.6', ls='-', marker='.')
        ax1.set_xlabel('Gate1 Voltage (D/A) [V]')
        ax1.set_ylabel('Gate1 Voltage (Monitor) [V]')
        ax1.set_title("Beam 3-4: Vg1 D/A vs. Vg1 Monitor")
        ax1.legend(loc='upper left', prop={'size': 9}, numpoints=1)
        ax1.grid()

        fig.tight_layout()
        fig.subplots_adjust(top=0.9)
        fig.suptitle('HEMT Multi test: Swp Vg1 -- Datetime:{0}'.format(datetime), fontsize=14)
        # if needed
        figname = self.savedir + 'Hemt_multi_test_SwpVg1_{}.png'.format(datetime)
        plt.savefig(figname)
        plt.show()
        return

    def swp_vg2_meas(self):
        # opening procedure
        self.mhemt = sis.multi_hemt()
        AD_data = []

        # measurement part
        for i in range(51):
            # -2.5 - -2.5 [mV]
            set_Vg2 = - 2.5 + 0.1 * i
            set_Vg2_list = [set_Vg2] * 8
            self.mhemt.set_Vg2(voltage=set_Vg2_list)
            time.sleep(0.5)
            ret = self.mhemt.monitor_hemt()  # data = 4*8 -> 1-32ch
            ret.insert(0, set_Vg2)
            AD_data.append(ret[0:33])

        # closing procedure
        self.mhemt.close_box()

        # saving data
        datetime = time.strftime('%Y%m%d-%H%M%S')
        filename = self.savedir + 'Hemt_multi_test_SwpVg2_{}.csv'.format(datetime)
        # sorry for long string
        header = 'DA-Vg2' \
                 '1L-VD,1L-Vg1,1L-Vg2,1L-ID,1R-VD,1R-Vg1,1R-Vg2, 1R-ID,' \
                 '2L-VD,2L-Vg1,2L-Vg2,2L-ID,2R-VD,2R-Vg1,2R-Vg2, 2R-ID,' \
                 '3L-VD,3L-Vg1,3L-Vg2,3L-ID,3R-VD,3R-Vg1,3R-Vg2, 3R-ID,' \
                 '4L-VD,4L-Vg1,4L-Vg2,4L-ID,4R-VD,4R-Vg1,4R-Vg2, 4R-ID'
        numpy.savetxt(fname=filename, X=AD_data, fmt='%.5f', header=header, delimiter=",")

        # output on terminal
        print('\n'
              '========  HEMT Sweep Vg2 MEASUREMENT ========\n'
              'Time Stamp  : {}\n'
              'Bias Box 1  : \n'
              'Bias Box 2  : \n'.format(datetime))

        # 取り急ぎ
        AD_data = numpy.loadtxt(filename, skiprows=1, delimiter=",")

        # plot part
        fig = plt.figure(figsize=(10, 5))
        ## figure-1: setVg1 vs monVg1
        ax1 = fig.add_subplot(1, 2, 1)
        ax1.plot(AD_data[:, 0], AD_data[:, 3], label='1LCP', ls='-', marker='.')
        ax1.plot(AD_data[:, 0], AD_data[:, 7]+0.2, label='1RCP+0.2', ls='-', marker='.')
        ax1.plot(AD_data[:, 0], AD_data[:, 11]+0.4, label='2LCP+0.4', ls='-', marker='.')
        ax1.plot(AD_data[:, 0], AD_data[:, 15]+0.6, label='2RCP+0.6', ls='-', marker='.')
        ax1.set_xlabel('Gate2 Voltage (D/A) [V]')
        ax1.set_ylabel('Gate2 Voltage (Monitor) [V]')
        ax1.set_title("Beam 1-2: Vg2 D/A vs. Vg2 Monitor")
        ax1.legend(loc='upper left', prop={'size': 9}, numpoints=1)
        ax1.grid(True)

        ## figure-2: Beam 2 - setVg1 vs monVg1
        ax2 = fig.add_subplot(1, 2, 2)
        ax2.plot(AD_data[:, 0], AD_data[:, 19], label='3LCP', ls='-', marker='.')
        ax2.plot(AD_data[:, 0], AD_data[:, 23]+0.2, label='3RCP+0.2', ls='-', marker='.')
        ax2.plot(AD_data[:, 0], AD_data[:, 27]+0.4, label='4LCP+0.4', ls='-', marker='.')
        ax2.plot(AD_data[:, 0], AD_data[:, 31]+0.6, label='4RCP+0.6', ls='-', marker='.')
        ax2.set_xlabel('Gate2 Voltage (D/A) [V]')
        ax2.set_ylabel('Gate2 Voltage (Monitor) [V]')
        ax2.set_title("Beam 3-4: Vg2 D/A vs. Vg2 Monitor")
        ax2.legend(loc='upper left', prop={'size': 9}, numpoints=1)
        ax2.grid(True)

        fig.tight_layout()
        fig.subplots_adjust(top=0.9)
        fig.suptitle('HEMT Multi test: Swp Vg2 -- Datetime:{0}'.format(datetime), fontsize=14)
        # if needed
        figname = self.savedir + 'Hemt_multi_test_SwpVg2_{}.png'.format(datetime)
        plt.savefig(figname)
        plt.show()
        return

    def allan_vd_meas(self, Vd=1.0, repeat=10000, interval=0.5):
        # opening procedure
        self.mhemt = sis.multi_hemt()
        AD_data = []

        ## Bias set
        setVd = Vd
        setVd_list = [setVd] * 8
        self.mhemt.set_Vg2(voltage=setVd_list)

        # measurement part
        reftime = time.time()
        for i in range(repeat):
            TimeStamp = time.time()
            ret = self.mhemt.monitor_hemt()
            Vdmon_list = ret[0:32:4]
            # Vdmon_list.insert(0, TimeStamp)
            AD_data.append(Vdmon_list)
            while True:
                if interval * (i+1) <= time.time() - reftime:
                    break
                continue

        # closing procedure
        self.mhemt.close_box()

        datetime = time.strftime('%Y%m%d-%H%M%S')
        filename = self.savedir + 'Hemt_multi_test_allan_Vd_{}.csv'.format(datetime)
        header = '1L-VD,1R-VD,2L-VD,2R-VD,3L-VD,3R-VD,4L-VD,4R-VD,'
        numpy.savetxt(fname=filename, X=AD_data, fmt='%.5f', header=header, delimiter=",")

        # output on terminal
        print('\n'
              '========  HEMT Allan Vd MEASUREMENT  ========\n'
              'Time Stamp  : {}\n'
              'Repeat      : {}\n'
              'Interval [s]: {}\n'
              'Vd [V]      : {}\n'
              'Bias Box 1  : \n'
              'Bias Box 2  : \n'.format(datetime, repeat, interval, Vd))

        # 取り急ぎ
        AD_data = numpy.loadtxt(filename, skiprows=1, delimiter=",")

        # plot part
        xaxis = numpy.linspace(1, repeat, repeat)
        fig = plt.figure(figsize=(5, 5))
        ## figure-1: time vs VD
        ax1 = fig.add_subplot(1, 1, 1)
        ax1.plot(xaxis, AD_data[:, 0], label='1LCP', ls='-', marker='.')
        ax1.plot(xaxis, AD_data[:, 1], label='1RCP', ls='-', marker='.')
        ax1.plot(xaxis, AD_data[:, 2], label='2LCP', ls='-', marker='.')
        ax1.plot(xaxis, AD_data[:, 3], label='2RCP', ls='-', marker='.')
        ax1.plot(xaxis, AD_data[:, 4], label='3LCP', ls='-', marker='.')
        ax1.plot(xaxis, AD_data[:, 5], label='3RCP', ls='-', marker='.')
        ax1.plot(xaxis, AD_data[:, 6], label='4LCP', ls='-', marker='.')
        ax1.plot(xaxis, AD_data[:, 7], label='4RCP', ls='-', marker='.')

        ax1.set_xlabel('Data Number')
        ax1.set_ylabel('Drain Voltage (Monitor) [V]')
        ax1.set_title("Beam 1-4: time variation of VD")
        ax1.legend(loc='upper left', prop={'size': 9}, numpoints=1)
        ax1.grid(True)

        figname = self.savedir + 'Hemt_multi_test_allan_Vd_{}.png'.format(datetime)
        plt.savefig(figname)
        plt.show()

        return






