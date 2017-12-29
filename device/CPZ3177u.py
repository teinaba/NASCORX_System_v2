

import pyinterface


class InvalidModeError(Exception):
    pass




class cpz3177u(object):


    def __init__(self, dev=0):
        self.dev = dev
        self.bname = 3177
        self.driver = pyinterface.open(self.bname, self.dev)




    def set_mode(self, mode='diff'):
        mode_eff = ['single', 'diff']
        
        if mode in mode_eff: pass
        else:
            msg = 'Mode must be single or diff mode.'
            msg += ' while {0} mode is given.'.format(mode)
            raise InvalidModeError(msg)


        self.driver.set_sampling_config(singlediff=mode)
        return




    def query_mode(self):
        ret = self.driver.get_sampling_config()
        return ret




    def query_input(self, singlediff='diff'):
        mode_eff = ['single', 'diff']
        
        if singlediff in mode_eff: pass
        else:
            msg = 'Mode must be single or diff mode.'
            msg += ' while {0} mode is given.'.format(singlediff)
            raise InvalidModeError(msg)


        ret = self.driver.input_ad_master(singlediff=singlediff)
        ret = [val/2. for val in ret]
        
        return ret

    def close_board(self):
        pass
