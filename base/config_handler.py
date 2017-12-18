#!/usr/bin/env python

# import modules
import numpy
import configparser


class Config_handler(object):
    season = None
    dirpath = '/home/amigos/NASCORX_System-master/NASCORX_System/configuration/'

    def __init__(self):
        # Load master configfile
        # ---------------------
        mastercnf = configparser.ConfigParser()
        mastercnf.read(self.dirpath+'NASCO-master.cnf')

        # Set directory
        # -------------
        self.season = mastercnf['Configuration']['Season']
        dirname = 'NASCO-{}/'.format(self.season)
        self.cnfdir = self.dirpath + dirname

        # Set config path
        # ---------------
        sisfname = mastercnf['Configuration']['SIS-config']
        sysfname = mastercnf['Configuration']['SYS-config']
        self.sis_conf = self.cnfdir + sisfname                                            # sis config
        self.sys_conf = self.cnfdir + sysfname                                            # system config
        self.device_table = self.dirpath + 'device_table.cnf'                             # device table
        pass

    def read(self, file):
        # Set config file --
        config = configparser.ConfigParser()
        config.read(file)
        return config

    def load_ip(self, device):
        config = self.read(self.device_table)
        ip = str(config[device]['ip'])
        return ip

    def load_ip_port(self, device):
        config = self.read(self.device_table)
        ip = str(config[device]['ip'])
        port = int(config[device]['port'])
        return ip, port

    def load_ip_port_connection(self, device):
        config = self.read(self.device_table)
        ip = str(config[device]['ip'])
        port = int(config[device]['port'])
        connection = str(config[device]['connection'])
        return ip, port, connection

    def load_ndev(self, device):
        config = self.read(self.device_table)
        ndev = str(config[device]['ndev'])
        return ndev

    def load_sis_params(self, ret='dict'):
        config = self.read(self.sis_conf)
        sections = config.sections()
        unitlist = [_sec for _sec in sections if 'beam' in _sec]

        # Load parameters
        # ---------------
        params_dict = {}
        params_list = []
        for unit in unitlist:
            SISV = float(config[unit]['sis-v'])
            VD = float(config[unit]['vd'])
            VG1 = float(config[unit]['vg1'])
            VG2 = float(config[unit]['vg2'])
            Lo_att = float(config[unit]['lo_att'])
            # for dict --
            params_dict[unit] = {'sisv': SISV, 'vd': VD, 'vg1': VG1, 'vg2': VG2, 'lo_att': Lo_att}
            # for list --
            params_temp = [SISV, VD, VG1, VG2, Lo_att]
            params_list.append(params_temp)

        if ret == 'array': return numpy.array(params_list)
        else: return params_dict

    def sis_config_converter(self, params_dict):
        params_list = []
        unitlist = params_dict.keys()
        for unit in unitlist:
            SISV = float(params_dict[unit]['sisv'])
            VD = float(params_dict[unit]['vd'])
            VG1 = float(params_dict[unit]['vg1'])
            VG2 = float(params_dict[unit]['vg2'])
            Lo_att = float(params_dict[unit]['lo_att'])

            params_temp = [SISV, VD, VG1, VG2, Lo_att]
            params_list.append(params_temp)
        return numpy.array(params_list)

    def check_sis_state(self):
        config = self.read(self.sis_conf)
        sections = config.sections()
        unitlist = [_sec for _sec in sections if 'beam' in _sec]

        # Load parameters
        # ---------------
        state_list = []
        for unit in unitlist:
            if config[unit]['state'] == 'ON': state = 1
            elif config[unit]['state'] == 'OFF': state = 0
            else: print('-- Invalid State --')
            print(' {} : {}'.format(unit, state))
            state_list.append(state)
        return state_list

    def write_sis_state(self, state_list):
        config = self.read(self.sis_conf)
        sections = config.sections()
        unitlist = [_sec for _sec in sections if 'beam' in _sec]

        # Set state
        # -----------
        for i, unit in enumerate(unitlist):
            state = state_list[i]
            if state == 0: config.set(unit, 'state', 'OFF')
            elif state == 1: config.set(unit, 'state', 'ON')
            print(' {} : {}'.format(unit, config[unit]['state']))

        # Write to cnf file
        # -----------------
        with open(self.sis_conf, 'w') as configfile:
            config.write(configfile)
        return

    def load_if_config(self, ret='dict'):
        config = self.read(self.sis_conf)
        params_list = []
        params_dict = {}

        # Load parameters
        # ---------------
        for key in config.options('IF_PATT'):
            att = int(config['IF_PATT'][key])
            params_list.append(att)
            params_dict[key] = att
        if ret == 'list': return params_list
        else: return params_dict

    def write_if_config(self, att_list):
        config = self.read(self.sis_conf)
        keys = config.options('IF_PATT')

        # Set attenuation
        # ---------------
        for i, key in enumerate(keys):
            config.set('IF_PATT', key, str(att_list[i]))
            print(' {} : {}'.format(key, str(att_list[i])))

        # Write to cnf file
        # -----------------
        with open(self.sis_conf, 'w') as configfile:
            config.write(configfile)
        return

    def get_season(self):
        return self.season


# History
# -------
# written by T.Inaba
# 2017/12/14 T.Inaba: create device table method. add read method.
