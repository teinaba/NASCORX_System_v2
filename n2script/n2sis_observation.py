#! /usr/bin/env python

# import modules
# --------------
import time

import n2db
from base import config_handler
from base import sis


class observation(object):
    running = False
    driver = None

    def __init__(self):
        self.config = config_handler.Config_handler()
        self.db = n2db.n2database.N2db()
        pass

    def switch(self, req):
        if req.switch == 'True': self.running = True
        elif req.switch == 'False': self.running = False

    def set_driver(self):
        self.driver = sis.mixer
        return

    def set_sisv(self):
        params = self.config.load_sis_params()
        sisv = params['beam1']['sisv']
        self.driver.set_sisv(Vmix=sisv, ch=0)
        return

    def set_loatt(self):
        params = self.config.load_sis_params()
        loatt = params['lo_att']
        self.driver.set_loatt(att=loatt, ch=0)
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
        self.set_loatt()
        self.observing_loop()
        return

    def observing_loop(self):
        while True:
            if self.running:
                time.sleep(1)
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

    def monitor_sis(self, sleep=1):
        while True:
            data = []
            for i in range(100):
                if self.running is False:
                    self.driver.close_box()
                    # TODO : data upload
                else:
                    pass
                ad = self.ad_sis()
                data.append(ad)
                time.sleep(sleep)
            try:
                self.db.INSERT(pjt='NASCORX', table='SIS', data=data)
            except:
                # retry --
                self.db.authorize()
                self.db.INSERT(pjt='NASCORX', table='SIS', data=data)
            # TODO : txt での Backup も作成する ??
        return


if __name__ == '__main__':
    import rospy
    from std_msgs.msg import String
    from necst.msg import Status_drive_msg

    rospy.init_node("sis_observation")
    rospy.loginfo("sis_observation start.")
    obs = observation()
    rospy.Subscriber("sis_obs", String, obs.switch)
    rospy.Subscriber("sis_obs_", String, obs.contactor)
    rospy.spin()

# History
# -------
# 2017/12/18 : written by T.Inaba
