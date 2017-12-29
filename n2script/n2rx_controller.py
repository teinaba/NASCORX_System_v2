#! /usr/bin/env python3

# import modules
import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import time
from datetime import datetime as dt
import rospy

# ROS messages
from std_msgs.msg import Bool
from NASCORX.msg import sisiv_msg
from NASCORX.msg import sisvsweep_msg
from NASCORX.msg import sislosweep_msg


class Controller(object):

    def __init__(self):
        rospy.init_node('RXcontroller_client')
        # self.pub1 = rospy.Publisher('observation_start_', Bool, queue_size=10)
        self.pub2 = rospy.Publisher('sis_iv', sisiv_msg, queue_size=10)
        self.pub3 = rospy.Publisher('sis_vsweep', sisvsweep_msg, queue_size=10)
        self.pub4 = rospy.Publisher('sis_losweep', sislosweep_msg, queue_size=10)
        return

    def sis_iv(self, initv=-8.0, finv=8.0, interval=0.1, lo=0.0):
        msg = sisiv_msg()
        msg.switch = True
        msg.initv = initv
        msg.finv = finv
        msg.interval = interval
        msg.lo = lo
        rospy.loginfo(msg)
        self.pub2.publish(msg)
        return

    def sis_vsweep(self, initv=-8.0, finv=8.0, interval=0.1, lo=0.0, integ=0.1):
        msg = sisvsweep_msg()
        msg.switch = True
        msg.initv = initv
        msg.finv = finv
        msg.interval = interval
        msg.lo = lo
        msg.integ = integ
        rospy.loginfo(msg)
        self.pub3.publish(msg)
        return

    def sis_losweep(self, initi=-8.0, fini=8.0, interval=0.05, sisv=0.0, integ=0.1):
        msg = sisiv_msg()
        msg.switch = True
        msg.initi = initi
        msg.fini = fini
        msg.interval = interval
        msg.sisv = sisv
        msg.integ = integ
        rospy.loginfo(msg)
        self.pub4.publish(msg)
        return


# History
# -------
# 2017/12/28 : written by T.Inaba
