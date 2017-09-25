#! /usr/bin/env python
# _*_ coding: UTF-8 _*_

# import modules
import time
import numpy
import matplotlib.pyplot
import NASCORX_System.base.Multi_Cryo as Multi_Cryo

class SIS_tune(object):
    savedir = '/home/amigos/NASCORX_Measurement/SIStune/'

    def __init__(self):
        pass

    def input_value_check(self, initV, finV, interval):
        if 0 <= initV and 0 < finV:
            pass
        else:
            raise ValueError('!!!Coming soon!!!')
        repeat = int(abs(initV - finV) / interval)
        return repeat

    def IVcurve_plot_options(self, ax, initV, finV):
        [_ax.set_xticklabels('') for i, _ax in enumerate(ax) if i/2<1]
        [_ax.set_yticklabels('') for i, _ax in enumerate(ax) if i%2!=0]
        [_ax.set_xlabel('SIS V [mV]') for i, _ax in enumerate(ax) if i/2>=1]
        [_ax.set_ylabel('SIS I [uA]') for i, _ax in enumerate(ax) if i%2==0]
        [_ax.set_xlim([initV, finV]) for _ax in ax]
        [_ax.legend(loc='upper left', prop={'size': 7}, numpoints=1) for _ax in ax]
        [_ax.grid(color='gray', linestyle='--') for _ax in ax]
        return

    def IVcurve_plot_divplot(self, AD_data, initV, finV, datetime):
        ## figure 1: Beam 1-2
        #### list-comprehension
        fig = matplotlib.pyplot.figure()
        ax = [fig.add_subplot(2, 2, i+1) for i in range(4)]
        labels = ['Beam1-LCP', 'Beam1-RCP', 'Beam2-LCP', 'Beam2-RCP']
        [_ax.plot(AD_data[:, 1+2*i], AD_data[:, 2+2*i], label=labels[i]) for i, _ax in enumerate(ax)]
        self.IVcurve_plot_options(ax=ax, initV=initV, finV=finV)

        fig.tight_layout()
        fig.subplots_adjust(top=0.9)
        fig.suptitle('SIS IV Curve: Beam 1-2\n{}'.format(datetime), fontsize=13)
        figname = self.savedir + 'SIS-IVcurve_Beam1-2_{}.png'.format(datetime)
        fig.savefig(figname)
        fig.show()

        ## figure 2: Beam 3-4
        fig = matplotlib.pyplot.figure()
        ax = [fig.add_subplot(2, 2, i+1) for i in range(4)]
        labels = ['Beam3-LCP', 'Beam3-RCP', 'Beam4-LCP', 'Beam4-RCP']
        [_ax.plot(AD_data[:, 9+2*i], AD_data[:, 10+2*i], label=labels[i]) for i, _ax in enumerate(ax)]
        self.IVcurve_plot_options(ax=ax, initV=initV, finV=finV)

        fig.tight_layout()
        fig.subplots_adjust(top=0.9)
        fig.suptitle('SIS IV Curve: Beam 3-4\n{}'.format(datetime), fontsize=13)
        figname = self.savedir + 'SIS-IVcurve_Beam3-4_{}.png'.format(datetime)
        fig.savefig(figname)
        fig.show()

        ## figure 3: 230GHz - 2 Pol x 2SB
        fig = matplotlib.pyplot.figure()
        ax = [fig.add_subplot(2, 2, i+1) for i in range(4)]
        labels = ['230GHz-LCP-USB', '230GHz-LCP-LSB', '230GHz-RCP-USB', '230GHz-RCP-LSB']
        [_ax.plot(AD_data[:, 17+2*i], AD_data[:, 18+2*i], label=labels[i]) for i, _ax in enumerate(ax)]
        self.IVcurve_plot_options(ax=ax, initV=initV, finV=finV)

        fig.tight_layout()
        fig.subplots_adjust(top=0.9)
        fig.suptitle('SIS IV Curve: 230GHz\n{}'.format(datetime), fontsize=13)
        figname = self.savedir + 'SIS-IVcurve_230GHz_{}.png'.format(datetime)
        fig.savefig(figname)
        fig.show()
        return

    def IVcurve_plot_oneplot(self, AD_data, initV, finV, datetime):
        fig = matplotlib.pyplot.figure(figsize=(9, 7))
        ax = [fig.add_subplot(3, 4, i+1) for i in range(12)]
        labels = ['Beam1-LCP', 'Beam1-RCP', 'Beam2-LCP', 'Beam2-RCP',
                  'Beam3-LCP', 'Beam3-RCP', 'Beam4-LCP', 'Beam4-RCP',
                  '230GHz-LCP-USB', '230GHz-LCP-LSB', '230GHz-RCP-USB', '230GHz-RCP-LSB']
        [_ax.plot(AD_data[:, 1+2*i], AD_data[:, 2+2*i], label=labels[i]) for i, _ax in enumerate(ax)]
        [_ax.set_xlim([initV, finV]) for _ax in ax]
        [_ax.set_ylim([-10, 500]) for _ax in ax]
        [_ax.set_xticklabels('') for i, _ax in enumerate(ax) if i/4<2]
        [_ax.set_yticklabels('') for i, _ax in enumerate(ax) if i%4!=0]
        [_ax.set_xlabel('SIS V [mV]') for i, _ax in enumerate(ax) if i/4>=2]
        [_ax.set_ylabel('SIS I [uA]') for i, _ax in enumerate(ax) if i%4==0]
        [_ax.legend(loc='upper left', prop={'size': 7}, numpoints=1) for _ax in ax]
        [_ax.grid(color='gray', linestyle='--') for _ax in ax]

        fig.tight_layout()
        fig.subplots_adjust(top=0.9)
        fig.suptitle('SIS IV Curve Measurement: {}'.format(datetime), fontsize=15)
        figname = self.savedir + 'SIS-IVcurve_{}.png'.format(datetime)
        fig.savefig(figname)
        fig.show()
        return

    def IVcurve_meas_part(self, repeat, initV=0.0, interval=0.1):
        # Opening procedure
        self.mmix = Multi_Cryo.multi_mixer()
        AD_data = []

        # Measurement part
        for i in range(repeat+1):
            setV = initV + i * interval
            setV_list = [setV] * 12
            self.mmix.set_sisv(Vmix=setV_list)
            time.sleep(0.5)
            ret = self.mmix.monitor_sis()
            ret.insert(0, setV)  # add setV to column 1
            AD_data.append(ret[0:25])

        # Closing procedure
        self.mmix.close_box()
        return

    def IVcurve_meas_part_onoff(self, repeat, initV=0.0, interval=0.1, onoff=[0]*12):
        # Opening procedure
        self.mmix = Multi_Cryo.multi_mixer()
        AD_data = []

        # Measurement part
        for i in range(repeat+1):
            temp = []
            setV = initV + i * interval
            setV_list = [0] * 12
            for j in range(12):
                if onoff[j] == 1: setV_list[j] = setV
                else: pass
            self.mmix.set_sisv(Vmix=setV_list)
            time.sleep(0.2)
            ret = self.mmix.monitor_sis()
            temp.append(setV)
            for j in range(24):
                if j % 2 == 0:
                    temp.append(ret[j]*1e+1)
                elif j % 2 == 1:
                    temp.append(ret[j]*1e+3)
            AD_data.append(temp)

        # Closing procedure
        self.mmix.close_box()

        return AD_data

    def IVcurve_meas(self, initV=0.0, finV=6.0, interval=0.1, onoff=[1]*12, filefmt='csv'):
        # input value check
        repeat = self.input_value_check(initV=initV, finV=finV, interval=interval)

        # measurement
        if 0 in onoff:
            AD_data = self.IVcurve_meas_part_onoff(repeat, initV=initV, interval=interval, onoff=onoff)
        elif 0 not in onoff:
            AD_data = self.IVcurve_meas_part(repeat, initV=initV, interval=interval)

        # Data saving
        datetime = time.strftime('%Y%m%d-%H%M')
        filename = self.savedir + 'SISIV_{}.{}'.format(datetime, filefmt)
        header = 'D/A-SISV,' \
                 '1L-V,1L-I,1R-V,1R-I,2L-V,2L-I,2R-V,2R-I,' \
                 '3L-V,3L-I,3R-V,3R-I,4L-V,4L-I,4R-V,4R-I,' \
                 'LCP-USB-V,LCP-USB-I,LCP-LSB-V,LCP-LSB-I,' \
                 'RCP-USB-V,RCP-USB-I,RCP-LSB-V,RCP-LSB-I'
        numpy.savetxt(filename, AD_data, fmt='%.5f', header=header, delimiter=',')

        # Data loading
        AD_data = numpy.loadtxt(filename, skiprows=1, delimiter=',')

        # Plot part
        self.IVcurve_plot_oneplot(AD_data=AD_data, initV=initV, finV=finV, datetime=datetime)

        # Output on terminal
        print('\n'
        '======== SIS IV Curve MEASUREMENT ========\n'
        'Time Stamp    : {}\n'
        'Start SISV    : {} [mV]\n'
        'Finish SISV   : {} [mV]\n'
        'Lo Attenuation: !!coming soon!! [mA]\n'.format(datetime, initV, finV))

        return

    def Trx_meas(self, initV=0.0, finV=6.0, interval=0.1, filefmt='csv'):
        # input value check
        repeat = self.input_value_check(initV=initV, finV=finV, interval=interval)

        # Opening procedure
        self.mmix = Multi_Cryo.multi_mixer()
        AD_data = []

        # Measurement part
        ## HOT Measurement
        # insert HOT method
        for i in range(repeat+1):
            setV = initV + i * interval
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
            setV = initV + i * interval
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
        filename = self.savedir + 'SISIV_{}.{}'.format(datetime, filefmt)
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

    def SIS_tune(self):
        return

    # ++++++++++++++++
    # Sub functions
    # ++++++++++++++++

    def get_allan_data(self, box, repeat=1000, interval=0.5):
        # Opening procedure
        AD_data = []

        # Measurement part
        for i in range(repeat):
            timestamp = time.strftime('%Y-%m-%d_%H:%M:%S')
            ret = box.monitor_sis()
            ret.insert(0, timestamp)
            AD_data.append(ret[0:26])
            time.sleep(interval)
        return AD_data
