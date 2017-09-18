#! /usr/bin/env python
# _*_ coding: UTF-8 _*_

# import modules
import time
import sys
import numpy
import matplotlib.pyplot as plt
sys.path.append('/home/amigos/NASCORX_System-master/multi/')
import Multi_Cryo


class SIS_tune(object):
    savedir = ''

    def __init__(self):
        pass

    def input_value_check(self, iniV, finV, interval):
        if 0 <= iniV and 0 < finV:
            pass
        else:
            raise ValueError('!!!Coming soon!!!')
        repeat = int((iniV - finV) / interval)
        return repeat

    def IVcurve_plt_options(self, ax):
        [_ax.set_xticklabels('') for i, _ax in enumerate(ax) if i/2<1]
        [_ax.set_yticklabels('') for i, _ax in enumerate(ax) if i%2!=0]
        [_ax.set_xlabel('SIS V [mV]') for i, _ax in enumerate(ax) if i/2>=2]
        [_ax.set_ylabel('SIS I [uA]') for i, _ax in enumerate(ax) if i%2==0]
        [_ax.legend(loc='upper left', prop={'size': 7}, numpoints=1) for _ax in ax]
        [_ax.grid(True) for _ax in ax]
        return

    def IVcurve_meas(self, iniV=0.0, finV=6.0, interval=0.1):
        # input value check
        repeat = self.input_value_check(iniV=iniV, finV=finV, interval=interval)

        # Opening procedure
        self.mmix = Multi_Cryo.multi_mixer()
        AD_data = []

        # Measurement part
        for i in range(repeat+1):
            setV = iniV + i * interval
            setV_list = [setV] * 12
            self.mmix.set_sisv(voltage=setV_list)
            time.sleep(0.5)
            ret = self.mmix.monitor_sis()
            ret.insert(0, setV)  # add setV to column 1
            AD_data.append(ret[0:25])

        # Closing procedure
        self.mmix.close_box()

        # Saving data
        datetime = time.strftime('%Y%m%d-%H%M%S')
        filename = self.savedir + 'SISIV_{}.csv'.format(datetime)
        header = 'D/A-SISV' \
                 '1L-V,1L-I,1R-V,1R-I,2L-V,2L-I,2R-V,2R-I,' \
                 '3L-V,3L-I,3R-V,3R-I,4L-V,4L-I,4R-V,4R-I,' \
                 'LCP-USB-V,LCP-USB-I,LCP-LSB-V,LCP-LSB-I,' \
                 'RCP-USB-V,RCP-USB-I,RCP-LSB-V,RCP-LSB-I'
        numpy.savetxt(filename, fmt='%.5f', header=header)

        # plot part
        ## figure 1: Beam 1-2
        #### list-comprehension
        fig = plt.figure()
        ax = [fig.add_subplot(2, 2, i+1) for i in range(4)]
        labels = ['Beam1-LCP', 'Beam1-RCP', 'Beam2-LCP', 'Beam2-RCP']
        [_ax.plot(AD_data[:, 1+2*i], AD_data[:, 2+2*i], label=labels[i]) for i, _ax in enumerate(ax)]
        self.IVcurve_plt_options(ax=ax)

        fig.suptitle('SIS IV Curve: Beam 1-2', fontsize=13)
        figname = self.savedir + 'SIS-IVcurve_Beam1-2_{}.png'.format(datetime)
        fig.savefig(figname)
        fig.show()

        ## figure 2: Beam 3-4
        fig = plt.figure()
        ax = [fig.add_subplot(2, 2, i+1) for i in range(4)]
        labels = ['Beam3-LCP', 'Beam3-RCP', 'Beam4-LCP', 'Beam4-RCP']
        [_ax.plot(AD_data[:, 9+2*i], AD_data[:, 10+2*i], label=labels[i]) for i, _ax in enumerate(ax)]
        self.IVcurve_plt_options(ax=ax)

        fig.suptitle('SIS IV Curve: Beam 3-4', fontsize=13)
        figname = self.savedir + 'SIS-IVcurve_Beam3-4_{}.png'.format(datetime)
        fig.savefig(figname)
        fig.show()

        ## figure 3: 230GHz - 2 Pol x 2SB
        fig = plt.figure()
        ax = [fig.add_subplot(2, 2, i+1) for i in range(4)]
        labels = ['230GHz-LCP-USB', '230GHz-LCP-LSB', '230GHz-RCP-USB', '230GHz-RCP-LSB']
        [_ax.plot(AD_data[:, 17+2*i], AD_data[:, 18+2*i], label=labels[i]) for i, _ax in enumerate(ax)]
        self.IVcurve_plt_options(ax=ax)

        fig.suptitle('SIS IV Curve: 230GHz', fontsize=13)
        figname = self.savedir + 'SIS-IVcurve_230GHz_{}.png'.format(datetime)
        fig.savefig(figname)
        fig.show()
        return

    def Trx_meas(self, iniV=0.0, finV=6.0, interval=0.1, savefmt=txt):
        # input value check
        repeat = self.input_value_check(iniV=iniV, finV=finV, interval=interval)

        # Opening procedure
        self.mmix = Multi_Cryo.multi_mixer()
        AD_data = []

        # Measurement part
        ## HOT Measurement
        # insert HOT method
        for i in range(repeat+1):
            setV = iniV + i * interval
            setV_list = [setV] * 12
            self.mmix.set_sisv(voltage=setV_list)
            time.sleep(0.5)
            # get spectrum (or total power from XFFTS)
            # get_power_method
            ret = self.mmix.monitor_sis()
            ret.insert(0, setV)
            AD_data.append(ret[0:25])

        ## SKY Measurement
        # obs sky method
        for i in range(repeat+1):
            setV = iniV + i * interval
            setV_list = [setV] * 12
            self.mmix.set_sisv(voltage=setV_list)
            time.sleep(0.5)
            # get spectrum (or total power from XFFTS)
            # get_power_method
            ret = self.mmix.monitor_sis()
            ret.insert(0, setV)
            AD_data.append(ret[0:25])

        # Closing procedure
        self.mmix.close_box()

        # Calculate Y-factor and Trx


        # Saving data
        datetime = time.strftime('%Y%m%d-%H%M%S')
        filename = self.savedir + 'SISIV_{}.csv'.format(datetime)
        header = 'D/A-SISV' \
                 '1L-V,1L-I,1L-PHOT,1L-Pcold,1L-Trx,1R-V,1R-I,1R-PHOT,1R-Pcold,1R-Trx,' \
                 '2L-V,2L-I,2L-PHOT,2L-Pcold,2L-Trx,2R-V,2R-I,2R-PHOT,2R-Pcold,2R-Trx,' \
                 '3L-V,3L-I,3L-PHOT,3L-Pcold,3L-Trx,3R-V,3R-I,3R-PHOT,3R-Pcold,3R-Trx,' \
                 '4L-V,4L-I,4L-PHOT,4L-Pcold,4L-Trx,2R-V,4R-I,4R-PHOT,4R-Pcold,4R-Trx,' \
                 'LCP-USB-V,LCP-USB-I,LCP-USB-PHOT,LCP-USB-Pcold,LCP-USB-Trx,' \
                 'LCP-LSB-V,LCP-LSB-I,LCP-LSB-PHOT,LCP-LSB-Pcold,LCP-LSB-Trx,' \
                 'RCP-USB-V,RCP-USB-I,RCP-USB-PHOT,RCP-USB-Pcold,RCP-USB-Trx,' \
                 'RCP-LSB-V,RCP-LSB-I,RCP-LSB-PHOT,RCP-LSB-Pcold,RCP-LSB-Trx'
        numpy.savetxt(filename, fmt='%.5f', header=header)


        return

    def Trx_fine_meas(self, V):
        return

    def allan_fine_meas(self):
        return