#! /usr/bin/env python

# import modules
import time
import rospy
import numpy
import matplotlib.pyplot

from ..base import sis
from NASCORX_XFFTS.data_client import data_client


class Losweep(object):
    method = 'Lo sweep Measurement'
    ver = '2017.12.18'
    savedir = '/home/amigos/NASCORX_Measurement/SIStune/'

    def __init__(self):
        pass

    def run(self, initI=0.0, finI=2.0, interval=0.05, sisv=0, integ=0.1):
        # Initialization Section
        # ======================

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
        repeat = self.input_value_check(initI=initI, finI=finI, interval=interval)

        # XFFTS client
        # ------------
        client = data_client()

        # Set Driver
        # ----------
        self.driver = sis.mixer()
        print('PCI board drivers are set.')


        # == Main ========

        # SIS bias setting
        # ----------------
        self.driver.set_sisv(sisv)

        # Measurement part
        # ----------------
        result = self.measure(repeat=repeat, initI=initI, interval=interval, integ=integ)

        # Calculate Y-Factor and Tsys
        # ---------------------------
        result = self.trx_calculate(result=result)

        # Data saving part
        # ----------------
        datetime = time.strftime('%Y%m%d-%H%M')
        filename = self.savedir + 'Losweep_sisv{}mV_{}.csv'.format(sisv, datetime)
        header = 'Loatt,Vhot,Ihot,Phot1,Phot2,Vcold,Icold,Pcold1,Pcold2,Yfac1,Yfac2,Tsys1,Tsys2'
        numpy.savetxt(filename, result, fmt='%.5f', header=header, delimiter=',')

        # Close sis driver
        # ----------------------------
        self.driver.close_box()

        # Data loading
        # ------------
        data = numpy.loadtxt(filename, skiprows=1, delimiter=',')

        # Plot part
        # ---------
        self.ttlplot(data=data, initI=initI, finI=finI, datetime=datetime, sisv=sisv)
        return


    def input_value_check(self, initI=2.0, finI=5.0, interval=0.05):
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
        result = []

        # HOT measurement
        # ---------------
        data_hot = self.sweep_lo(repeat, initI=initI, interval=interval, integ=integ)

        # COLD measurement
        # ----------------
        data_sky = self.sweep_lo(repeat, initI=initI, interval=interval, integ=integ)

        # data arrangement
        # ----------------
        result.append(data_hot)
        result.append(data_sky)
        ret = numpy.array(result)
        return ret

    def sweep_lo(self, repeat, initI=2.0, interval=0.05, integ=0.1):
        result = []

        for i in range(repeat+1):
            temp = []
            setI = initI + i * interval

            # bias set
            # --------
            self.driver.set_loatt(att=setI, ch=0)

            time.sleep(0.2)

            # data receive
            # ------------
            # spec = self.client.oneshot(integtime=integ, repeat=1, start=None)
            spec = data_client.oneshot(1, integ, 0)
            ad = self.driver.monitor_sis()

            # data arrangement
            # ----------------
            temp.append(setI)
            temp.append(ad[0]*1e+1)      # AD[V] --> bias [mV]
            temp.append(ad[1]*1e+3)    # AD[V] --> current [uA]
            power = numpy.sum(spec, axis=1)
            temp.append(power)
            result.append(temp)
        return result

    def ttlplot(self, AD_data, initI, finI, datetime, sisv):
        fig = matplotlib.pyplot.figure(figsize=(9, 7))
        ax = [fig.add_subplot(3, 4, i+1) for i in range(12)]
        axtw = [_ax.twinx() for _ax in ax]
        labels = ['Beam1-LCP', 'Beam1-RCP', 'Beam2-LCP', 'Beam2-RCP',
                  'Beam3-LCP', 'Beam3-RCP', 'Beam4-LCP', 'Beam4-RCP',
                  '230GHz-LCP-USB', '230GHz-LCP-LSB', '230GHz-RCP-USB', '230GHz-RCP-LSB']
        # For first plot
        ax[1].plot(AD_data[:, ], AD_data[:, ], label='HOT', color='Red')
        ax[1].plot(AD_data[:, ], AD_data[:, ], label='COLD', color='Blue')
        axtw[1].plot(AD_data[:, ], AD_data[:, ], label='Trx', color='Green')
        # HOT plot
        [_ax.plot(AD_data[:, 1+2*i], AD_data[:, 2+2*i], label=labels[i], color='Red')
         for i, _ax in enumerate(ax) if i>=2]
        # COLD plot
        [_ax.plot(AD_data[:, 1+2*i], AD_data[:, 2+2*i], label=labels[i], color='Blue')
         for i, _ax in enumerate(ax) if i>=2]
        # Trx plot
        [_ax.plot(AD_data[:, 1+2*i], AD_data[:, 2+2*i], label=labels[i], color='Green')
         for i, _ax in enumerate(axtw) if i>=2]
        # Options
        [_ax.set_xlim([initI, finI]) for _ax in ax]
        [_ax.set_ylim([-10, 500]) for _ax in ax]
        [_ax.set_xticklabels('') for i, _ax in enumerate(ax) if i/4<2]
        [_ax.set_yticklabels('') for i, _ax in enumerate(ax) if i%4!=0]
        [_ax.set_xlabel('Lo Attenuation [mA]') for i, _ax in enumerate(ax) if i/4>=2]
        [_ax.set_ylabel('Power [dBm]') for i, _ax in enumerate(ax) if i%4==0]
        [_ax.legend(loc='upper left', prop={'size': 7}, numpoints=1) for _ax in ax]
        [_ax.grid(color='gray', linestyle='--') for _ax in ax]

        fig.tight_layout()
        fig.subplots_adjust(top=0.9)
        fig.suptitle('SIS IV Curve Measurement: {}'.format(datetime), fontsize=15)
        figname = self.savedir + 'SIS-IVcurve_{}.png'.format(datetime)
        fig.savefig(figname)
        fig.show()
        return

