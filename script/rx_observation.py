#! /usr/bin/env python

# import modules
# --------------
import time

import n2db
from base import config_handler
from base import sis
from base import Multi_Lo
from base import Multi_IF


class observation(object):
    running = False
    driver = None
    firstlo = None
    secondlo = None
    att = None
    filepath = '/home/amigos/NASCORX_Measurement/SIStune/log.txt'

    def __init__(self):
        self.config = config_handler.Config_handler()
        self.db = n2db.n2database.N2db()
        pass

    def switch(self, req):
        if req.switch == 'True': self.running = True
        elif req.switch == 'False': self.running = False

    def set_driver(self):
        self.driver = sis.multi_box()
        self.firstlo = Multi_Lo.multi_firstlo()
        self.secondlo = Multi_Lo.multi_secondlo()
        self.att = Multi_IF.multi_prog_att()
        return

    def set_sisv(self):
        sisv = self.config.load_sis_params(ret='list')
        self.driver.set_sisv(Vmix=sisv)
        return

    def set_loatt(self):
        params = self.config.load_sis_params()
        loatt = params['lo_att']
        self.driver.set_loatt(att=loatt)
        return

    def set_hemt(self):
        params = self.config.load_sis_params()
        self.driver.set_Vd(voltage=params['vd'])
        self.driver.set_Vg1(voltage=params['vg1'])
        self.driver.set_Vg2(voltage=params['vg2'])
        return

    def set_prog_att(self):
        loatt = self.config.load_if_config(ret='list')
        self.att.set_attenuation(att=loatt)
        return

    def set_1stsg(self):
        freq, power = self.config.load_1stsg_config(ret='list')
        self.firstlo.start_osci(freq=freq, power=power)
        return

    def set_2ndsg(self):
        freq, power = self.config.load_2ndsg_config(ret='list')
        self.secondlo.start_osci(freq=freq, power=power)
        return

    def waiting_loop(self):
        while True:
            if self.running is True:
                self.start_observation()
            else:
                continue
        return

    def start_observation(self):
        self.set_driver()
        self.set_sisv()
        self.set_hemt()
        self.set_loatt()
        self.set_1stsg()
        self.set_2ndsg()
        self.set_prog_att()
        self.observing_loop()
        return

    def observing_loop(self):
        while True:
            if self.running:
                time.sleep(1)
                self.monitor_sis(filepath=self.filepath)
                continue
            else:
                self.driver.close_box()
        return

    def end_observation(self):
        self.running = False
        return

    def ad_sis(self):
        ret = []
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        ad = self.driver.monitor_sis()
        ret.append(timestamp)
        ret.append(ad[0]*1e+1)      # AD[V] --> bias [mV])
        ret.append(ad[1]*1e+3)    # AD[V] --> current [uA]
        return ret

    def monitor_sis(self, filepath):
        data = []
        ad = self.ad_sis()
        data.append(ad)
        f = open(filepath, 'a')
        f.write(data)
        f.close()
        return

if __name__ == '__main__':
    import rospy
    from std_msgs.msg import String
    from necst.msg import Status_drive_msg

    rospy.init_node("sis_observation")
    rospy.loginfo("sis_observation start.")
    obs = observation()
    rospy.Subscriber("sis_obs", String, obs.switch)
    rospy.Subscriber("sis_obs", String, obs.contactor)
    rospy.spin()

# History
# -------
# 2017/12/18 : written by T.Inaba
