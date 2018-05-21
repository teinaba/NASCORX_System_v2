#! /usr/bin/env python

# import modules
import time
import rospy
import numpy
import matplotlib.pyplot

from NASCORX_System.base import config_handler
from NASCORX_System.base import sis
from NASCORX_XFFTS.data_client import data_client


class Vsweep(object):
    method = 'SIS Trx Measurement'
    ver = '2017.05.21'
    BE_num = 16
    savedir = '/home/amigos/NASCORX_Measurement/SIStune/'

    def __init__(self):
        pass

    def run(self, initV=0.0, finV=8.0, interval=0.1, driver=None, onoff='cnf', reconfigure=False,
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
            pass

        # XFFTS client
        # ------------
        client = data_client()

        # Set Driver
        # ----------
        if driver is None: self.driver = sis.multi_box()
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

        data_hot = self.Trx_sweep_sisv(repeat, initV=initV, interval=interval, onoff=onoff, integ=integ)

        # SKY measurement
        # ---------------

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
            spec = data_client.oneshot(1, integ, 0)
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

    def Trx_plot(self, sis_v, sis_i, tsys, phot, pcold, if_name, datetime,
                 labelfs=15, titlefs=12, legendfs=10, legendloc='upper left',
                 savedir='/home/amigos/'):
        """
        DESCRIPTION
        ===========
        This method plots a figure of single IF output.

        ARGUMENTS
        =========
        1. sis_v      : < Data of SIS-V [mV] : float list : >
        2. sis_i      : < Data of SIS-I [uA] : float list : >
        3. tsys       : < Data of Tsys [K] : float list : >
        4. phot       : < Data of Power of HOT load [Cnt] : float list : >
        5. pcold      : < Data of Power of Sky [Cnt : float list : >
        6. if_name    : < Which IF output : str : fmt = "Beam2-LCP" >
        7. datetime   : < Measurement datetime : str >
        8. labelfs    : < label font size : int : default=15 : see the official docs of matplotlib >
        9. titlefs    : < title font size : int : default=12 : same as above >
        10. legendfs  : < legend font size : int : default=10 : same as above >
        11. legendloc : < legend location : str : default="upper left" : use - upper, lower, right, left >
        12. savedir   : < save directory : str : fmt = "/xx/xx/xx/" (end in "/") >

        RETURNS
        =======
        Nothing.
        """

        # declare --
        fig = matplotlib.pyplot.figure(dpi=100)    # TODO : dpi value
        fig.subplots_adjust(right=0.77)
        ax = fig.add_subplot(1, 1, 1)
        ax2 = ax.twinx()
        ax3 = ax.twinx()
        ax2.spines['right'].set_position(('axes', 1.16))

        # plot part --
        p1, = ax3.plot(sis_v, sis_i, label='SIS-IV', color='black')
        p2, = ax2.plot(sis_v, phot, label='P-hot', color='red', ls='-')
        p3, = ax2.plot(sis_v, pcold, label='P-cold', color='blue', ls='-')
        p4, = ax.plot(sis_v, tsys, label='Tsys', color='green', ls='-')
        lines = [p1, p2, p3, p4]

        # each settings --
        ax.set_xlim([sis_v[0], sis_v[-1]])
        ax.set_ylim([0, 200])    # TODO: ylim
        ax.set_ylabel('Tsys [K]', fontsize=labelfs)
        ax.set_xlabel('V [mV]', fontsize=labelfs)
        ax2.set_ylabel('Power [cnt]', fontsize=labelfs)
        ax3.set_ylabel('I [uA]', fontsize=labelfs)
        ax3.set_title('Tsys : {} - {}'.format(if_name, datetime), fontsize=titlefs)
        ax.grid(which='both', color='gray', linestyle='--')
        ax.legend(lines, [l.get_label() for l in lines], loc=legendloc, prop={'size': legendfs}, numpoints=1)

        # set label colors --
        ax.yaxis.label.set_color('green')
        ax.tick_params(axis='y', colors='green')
        ax2.yaxis.label.set_color('red')
        ax2.tick_params(axis='y', colors='red')
        ax3.yaxis.label.set_color('black')
        ax3.tick_params(axis='y', colors='black')

        # set background color --
        ax.patch.set_facecolor('lightgray')
        ax.patch.set_alpha(0.05)

        figname = savedir + 'Tsys_{}_{}.png'.format(if_name, datetime)
        fig.savefig(figname)
        return

    def Trx_plot_v2(self, AD_data, initV, finV, datetime):




        return

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
