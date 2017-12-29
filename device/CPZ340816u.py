

import pyinterface


class InvalidChRangeError(Exception):
    pass


class InvalidVoltageRangeError(Exception):
    pass


class InvalidOnOffError(Exception):
    pass




class cpz340816u(object):


    def __init__(self, dev=0):
        self.dev = dev
        self.bname = 3408
        self.driver = pyinterface.open(self.bname, self.dev)
        self.initialize()



    def initialize(self):
        self.driver.initialize()
        return




    def set_output(self, onoff=0):
        onoff_eff = [0, 1]


        if onoff in onoff_eff: pass
        else:
            msg = 'Onoff must be 0 or 1 absolutelly'
            msg += 'while {0} is given.'.format(onoff)
            raise InvalidOnOffError(msg)

        self.driver._da_onoff(onoff=onoff)


    def set_voltage(self, voltage=0.0, ch=0):
        ch = 'ch{}'.format(ch+1)
        ch_lim_initial = 1
        ch_lim_final = 16
        vol_limit = 10.0
        ch_eff = ['ch{}'.format(i) for i in range(ch_lim_initial, ch_lim_final+1)]

        if ch in ch_eff: pass
        else:
            msg = 'Ch range is in ch{0} - ch{1}'.format(ch_lim_initial, ch_lim_final)
            msg = 'while {0} is given'.format(ch)
            raise InvalidChRangeError(msg)


        if abs(voltage)<=10.0:
            self.driver.output_da_sim(ch, voltage)
        else:
            msg = 'Voltage must be in {0}[V] - {1}[V].'.format(-vol_limit, vol_limit)
            msg += 'while {0}[V] is given.'.format(voltage)
            raise InvalidVoltageRangeError(msg)
        return



    def close_board(self):
        self.driver.finalize()
        return