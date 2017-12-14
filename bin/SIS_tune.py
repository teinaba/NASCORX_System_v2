#! /usr/bin/env python

# import modules
import time
import numpy
import matplotlib.pyplot
from ..bin import config_handler
from ..base import Multi_Cryo


class IV_curve(object):
    method = 'IV Curve Measurement'
    ver = '2017.11.09'
    savedir = '/home/amigos/NASCORX_Measurement/SIStune/'

    def __init__(self):
        pass

    def run(self, initV=0.0, finV=8.0, interval=0.1, driver=None, onoff='cnf', reconfigure=True,
            filefmt='csv'):

        # Print Welcome massage
        # ----------------------
        print('\n\n'
              ' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n'
              '   NASCO RX : SIS IV curve Measurement  \n'
              ' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n'
              ' ver - {}\n'
              '\n\n'.format(self.ver))

        # Input value check
        # -----------------
        repeat = self.input_value_check(initV=initV, finV=finV, interval=interval)

        # Check available unit
        # --------------------
        if onoff == 'cnf':
            onoff = config_handler.Config_handler.check_sis_state()
        else:
            pass

        # Set Driver
        # ----------
        if driver is None: self.driver = Multi_Cryo.multi_box()
        else: self.driver = driver
        print('\n'
              ' PCI board drivers are set.\n'
              '\n')

        # == Main ========
        # Measurement part
        # ----------------
        AD_data = self.IV_measure(repeat=repeat, initV=initV, interval=interval, onoff=onoff)
        lo_att = self.driver.query_loatt()

        # Data saving part
        # ----------------
        # TODO : Loどうする??
        datetime = time.strftime('%Y%m%d-%H%M')
        filename = self.savedir + 'SISIV_{}.{}'.format(datetime, filefmt)
        header = 'D/A-SISV,' \
                 '1L-V,1L-I,1R-V,1R-I,2L-V,2L-I,2R-V,2R-I,' \
                 '3L-V,3L-I,3R-V,3R-I,4L-V,4L-I,4R-V,4R-I,' \
                 'LCP-USB-V,LCP-USB-I,LCP-LSB-V,LCP-LSB-I,' \
                 'RCP-USB-V,RCP-USB-I,RCP-LSB-V,RCP-LSB-I'
        numpy.savetxt(filename, AD_data, fmt='%.5f', header=header, delimiter=',')

        # Reconfigure SIS bias setting
        # ----------------------------
        if driver is None: self.driver.close_box()
        if reconfigure is True: self.IV_recofigure()

        # Data loading
        # ------------
        AD_data = numpy.loadtxt(filename, skiprows=1, delimiter=',')

        # Plot part
        # ---------
        self.IV_ttlplot(AD_data=AD_data, initV=initV, finV=finV, datetime=datetime)

        # Output information on terminal
        # ------------------------------
        print('\n'
              ' ======== SIS IV Curve MEASUREMENT ========\n'
              ' Time Stamp    : {}\n'
              ' Start SISV    : {} [mV]\n'
              ' Finish SISV   : {} [mV]\n'
              ' Lo Attenuation: !!coming soon!! [mA]\n\n'.format(datetime, initV, finV))
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

    def IV_plot_options(self, ax, initV, finV):
        [_ax.set_xticklabels('') for i, _ax in enumerate(ax) if i/2<1]
        [_ax.set_yticklabels('') for i, _ax in enumerate(ax) if i%2!=0]
        [_ax.set_xlabel('SIS V [mV]') for i, _ax in enumerate(ax) if i/2>=1]
        [_ax.set_ylabel('SIS I [uA]') for i, _ax in enumerate(ax) if i%2==0]
        [_ax.set_xlim([initV, finV]) for _ax in ax]
        [_ax.legend(loc='upper left', prop={'size': 7}, numpoints=1) for _ax in ax]
        [_ax.grid(color='gray', linestyle='--') for _ax in ax]
        return

    def IV_divplot(self, AD_data, initV, finV, datetime):
        ## figure 1: Beam 1-2
        #### list-comprehension
        fig = matplotlib.pyplot.figure()
        ax = [fig.add_subplot(2, 2, i+1) for i in range(4)]
        labels = ['Beam1-LCP', 'Beam1-RCP', 'Beam2-LCP', 'Beam2-RCP']
        [_ax.plot(AD_data[:, 1+2*i], AD_data[:, 2+2*i], label=labels[i]) for i, _ax in enumerate(ax)]
        self.IV_plot_options(ax=ax, initV=initV, finV=finV)

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
        self.IV_plot_options(ax=ax, initV=initV, finV=finV)

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
        self.IV_plot_options(ax=ax, initV=initV, finV=finV)

        fig.tight_layout()
        fig.subplots_adjust(top=0.9)
        fig.suptitle('SIS IV Curve: 230GHz\n{}'.format(datetime), fontsize=13)
        figname = self.savedir + 'SIS-IVcurve_230GHz_{}.png'.format(datetime)
        fig.savefig(figname)
        fig.show()
        return

    def IV_ttlplot(self, AD_data, initV, finV, datetime):
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

    def IV_measure(self, repeat, initV=0.0, interval=0.1, onoff=[0]*12):
        AD_data = []
        setV_list = [0] * 12

        # Measurement part
        # ----------------
        for i in range(repeat+1):
            temp = []

            # set bias --
            setV = initV + i * interval
            for j in range(12):
                if onoff[j] == 1: setV_list[j] = setV
                else: pass
            self.driver.set_sisv(Vmix=setV_list)

            time.sleep(0.2)

            # get data --
            ret = self.driver.monitor_sis()

            # data arrangement
            # ----------------
            temp.append(setV)
            # scaling -- TODO : 計算処理を後でまとめた方が効率がいい??
            for j in range(24):
                if j % 2 == 0:
                    temp.append(ret[j]*1e+1)  # AD[V] --> bias [mV]
                elif j % 2 == 1:
                    temp.append(ret[j]*1e+3)  # AD[V] --> current [uA]
            AD_data.append(temp)
        return AD_data

    def IV_recofigure(self):
        # get params --
        params = config_handler.Config_handler.load_sis_params(ret='array')
        # set bias --
        self.driver.set_sisv(Vmix=params[:, 0])
        # self.box.set_Vd(voltage=params[:, 1])
        # self.box.set_Vg1(voltage=params[:, 2])
        # self.box.set_Vg2(voltage=params[:, 3])
        self.driver.set_loatt(att=params[:, 4])
        return


class Trx_curve(object):
    method = 'SIS Trx Measurement'
    ver = '2017.11.09'
    BE_num = 16
    savedir = '/home/amigos/NASCORX_Measurement/SIStune/'

    def __init__(self):
        pass

    def run(self, initV=0.0, finV=8.0, interval=0.1, driver=None, onoff='cnf', reconfigure=True,
            integ=0.1, filefmt='csv'):

        # Print Welcome massage
        # ----------------------
        print('\n\n'
              ' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n'
              '   NASCO RX : SIS Tsys curve Measurement \n'
              ' =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n'
              ' ver - {}\n'
              '\n\n'.format(self.ver))

        # Input value check
        # -----------------
        repeat = self.input_value_check(initV=initV, finV=finV, interval=interval)

        # Check available unit
        # --------------------
        if onoff == 'cnf':
            onoff = config_handler.Config_handler.check_sis_state()
        else:
            onoff = [0]*12
            pass

        # Set Driver
        # ----------
        if driver is None: self.driver = Multi_Cryo.multi_box()
        else: self.driver = driver
        print('PCI board drivers are set.')

        # == Main ========

        # Measurement part
        # ----------------
        data = self.Trx_measure(repeat=repeat, initV=initV, interval=interval, onoff=onoff, integ=integ)
        lo_att = self.driver.query_loatt()

        # Calculate Y-factor and Tsys
        # ---------------------------
        data = self.Trx_calculate(data=data)

        # Data saving
        # -----------
        datetime = time.strftime('%Y%m%d-%H%M%S')
        filename = self.savedir + 'SIS_Tsys_curve_{}.{}'.format(datetime, filefmt)
        header = 'D/A-V,' \
                 '{0}-V,{0}-I,{1}-V,{1}-I,{2}-V,{2}-I,{3}-V,{3}-I,{4}-V,{4}-I,{5}-V,{5}-I,' \
                 '{6}-V,{6}-I,{7}-V,{7}-I,{8}-V,{8}-I,{9}-V,{9}-I,{10}-V,{10}-I,{11}-V,{11}-I,' \
                 'Phot1,Phot2,Phot3,Phot4,Phot5,Phot6,Phot7,Phot8,' \
                 'Phot9,Phot10,Phot11,Phot12,Phot13,Phot14,Phot15,Phot16' \
                 '{0}-V,{0}-I,{1}-V,{1}-I,{2}-V,{2}-I,{3}-V,{3}-I,{4}-V,{4}-I,{5}-V,{5}-I,' \
                 '{6}-V,{6}-I,{7}-V,{7}-I,{8}-V,{8}-I,{9}-V,{9}-I,{10}-V,{10}-I,{11}-V,{11}-I,' \
                 'Psky1,Psky2,Psky3,Psky4,Psky5,Psky6,Psky7,Psky8,' \
                 'Psky9,Psky10,Psky11,Psky12,Psky13,Psky14,Psky15,Psky16' \
                 'Yfac1,Yfac2,Yfac3,Yfac4,Yfac5,Yfac6,Yfac7,Yfac8,' \
                 'Yfac9,Yfac10,Yfac11,Yfac12,Yfac13,Yfac14,Yfac15,Yfac16,' \
                 'Tsys1,Tsys2,Tsys3,Tsys4,Tsys5,Tsys6,Tsys7,Tsys8,' \
                 'Tsys9,Tsys10,Tsys11,Tsys12,Tsys13,Tsys14,Tsys15,Tsys16'
        numpy.savetxt(filename, data, fmt='%.5f', header=header, delimiter=',')

        # Reconfigure SIS bias setting
        # ----------------------------
        #TODO --> test
        if driver is None: self.driver.close_box()
        if reconfigure is True: self.Trx_recofigure()

        # Data loading
        # ------------
        data = numpy.loadtxt(filename, skiprows=1, delimiter=',')


        return

    def Trx_measure(self, repeat, initV=0.0, interval=0.1, onoff=[0]*12, integ=0.1):
        data = []

        # HOT measurement
        # ---------------
        # TODO : HOT IN
        data_hot = self.Trx_sweep_sisv(repeat, initV=initV, interval=interval, onoff=onoff, integ=integ)

        # SKY measurement
        # ---------------
        # TODO : HOT OUT, OBS SKY
        data_sky = self.Trx_sweep_sisv(repeat, initV=initV, interval=interval, onoff=onoff, integ=integ)

        # data arrangement
        # ----------------
        data.append(data_hot)
        data.append(data_sky)
        ret = numpy.array(data)
        return ret

    def Trx_sweep_sisv(self, repeat, initV=0.0, interval=0.1, onoff=[0]*12, integ=0.1):
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
            # spec = self.client.oneshot(integtime=integ, repeat=1, start=None)
            spec = self.dfs.oneshot(1, integ, 0)    # TODO : DFS controller
            ad = self.driver.monitor_sis()

            # data arrangement
            # ----------------
            temp.append(setV)
            # AD data scaling --
            for j in range(24):
                if j % 2 == 0:
                    temp.append(ad[j]*1e+1)         # AD[V] --> bias [mV]
                elif j % 2 == 1:
                    temp.append(ad[j]*1e+3)         # AD[V] --> current [uA]
            # spectrum analysis --
            power = numpy.sum(spec, axis=1)
            temp.append(power)
            data.append(temp)
        return data

    def Trx_calculate(self, data, Tamb=300):
        # dB scale Y-factor
        # -----------------
        Yfac = [10*numpy.log10(data[25+i]/data[65+i]) for i in range(16)]

        # Tsys calculation
        # ----------------
        Tsys = [(Tamb*data[65+i]) / (data[25+i]-data[65+i]) for i in range(16)]

        # append data
        # -----------
        data = numpy.concatenate((data, Yfac), axis=0)
        data = numpy.concatenate((data, Tsys), axis=0)
        return data

    def Trx_plot_oneplot(self, AD_data, initV, finV, datetime):
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
        [_ax.set_xlim([initV, finV]) for _ax in ax]
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

    def Trx_recofigure(self):
        # get params --
        params = config_handler.Config_handler.load_sis_params(ret='array')
        # set bias --
        self.driver.set_sisv(Vmix=params[:, 0])
        # self.box.set_Vd(voltage=params[:, 1])
        # self.box.set_Vg1(voltage=params[:, 2])
        # self.box.set_Vg2(voltage=params[:, 3])
        self.driver.set_loatt(att=params[:, 4])
        return


class Swp_Lo_Measure(object):
    method = 'Sweep Lo Measurement'
    ver = '2017.10.05'
    savedir = '/home/amigos/NASCORX_Measurement/SIStune/'

    def __init__(self):
        pass

    def swpLo_meas(self, initI=0.0, finI=2.0, interval=0.05, onoff=[1]*10):
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
        if 0 <= initI <= finI <= 100:
            pass
        else:
            raise ValueError('!!!Coming soon!!!')
        repeat = int(abs(initI - finI) / interval)


        # Operation Section
        # =================

        # Opening procedure
        # -----------------
        self.box = Multi_Cryo.multi_box()
        print('Open Devices'
              '============')
        AD_data = []

        # Measurement part
        # ----------------

        # 1. HOT Measurement
        # move.hot functions
        for i in range(repeat+1):
            temp = []
            setI = initI + i * interval
            setI_list = [0] * 10
            for j in range(10):
                if onoff[i] == 1: setI_list[j] = setI
                else: pass
            self.box.set_loatt(att=setI_list)
            time.sleep(0.2)
            # get IF power function


        # 2. COLD Measurement
        for i in range(repeat+1):
            temp = []
            setI = initI + i * interval
            setI_list = [0] * 10
            for j in range(10):
                if onoff[i] == 1: setI_list[j] = setI
                else: pass
            self.box.set_loatt(att=setI_list)
            time.sleep(0.2)
            # get IF power function

    def Swp_Lo_plot_oneplot(self, AD_data, initI, finI, datetime):
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

"""
class SIS_tune(object):
    method = 'SIS Tuning Handler'
    ver = '2017.10.05'
    savedir = '/home/amigos/NASCORX_Measurement/SIStune/'

    def __init__(self):
        self.IV_handler = IV_curve()
        self.Trx_handler = Trx_Measure()
        self.Lo_handler = Swp_Lo_Measure()

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


class SIS_tuner(object):
    method = 'SIS_controller'
    ver = '2017.10.08'

    def __init__(self):
        self.iv_meas = IV_curve()
        self.trx_meas = Trx_Measure()
        self.swplo_meas = Swp_Lo_Measure()
        self.sis_tune = SIS_tune()
        pass

    def tst(self):
        return
"""

# History
# -------
# written by T.Inaba
