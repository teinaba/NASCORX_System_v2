#! /usr/bin/env python

# import modules
import csv
import time
import NASCORX_System.base.Multi_IF as Multi_IF
import NASCORX_System.bin.config_handler as config_handler


class IF_controller(Multi_IF.multi_IF):
    method = 'IF controller'
    ver = '2017.11.01'
    logdir = '/home/amigos/NASCORX_Measurement/IF-log/'

    def __init__(self):
        super().__init__()
        self.config = config_handler.Config_handler()
        season = self.config.get_season()
        self.logfile = self.logdir + 'IF_log-{}.csv'.format(season)
        pass

    def set_att_wlog(self, att=[11]*12):
        # Set attenuation
        # ---------------
        att_before = self.patt_ctrl.query_attenuation()
        self.patt_ctrl.set_attenuation(att=att)
        time.sleep(1)
        att_after = self.patt_ctrl.query_attenuation()

        # Logging
        # -------
        self.log(att_before)
        self.log(att_after)
        return

    def set_att_config(self, log=True):
        # Get configuration --
        att = self.config.load_if_config(ret='list')

        # Set attenuation
        # ---------------
        if log is True: self.set_att_wlog(att=att)
        else: self.patt_ctrl.set_attenuation(att=att)
        return

    def log(self, result):
        with open(self.logfile, "a") as f:
            writecsv = csv.writer(f)
            writecsv.writerow(result)
        return

# History
# -------
# written by T.Inaba
