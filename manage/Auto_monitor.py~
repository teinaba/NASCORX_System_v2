#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import sys, time, datetime, os, csv
sys.path.append('/home/amigos/NASCORX-master/base/')
import Monitor

class monitor(object):
    '''
    DESCRIPTION
    ================
    This class monitors the Rx conditions.

    ARGUMENTS
    ================
    Nothing.
    '''
    def __init__(self):
        self.roomtemp = 'TR71W'
        self.vacuum = 'TPG261'
        self.cryotemp = 'L218'
        self.mon = Monitor.monitor(rt=self.roomtemp, vc=self.vacuum, ct=self.cryotemp)

    def monitoring(self, interval=5.0, logpath='/home/amigos/data/rxmonitor/'):
        '''        
        DESCRIPTION
        ================
        This function monitors the room temperature, dewar pressure and dewar temperature.
        
        ARGUMENTS
        ================
        1. interval: interval time of measurement [sec]
            Type: float
            Default: 5.0
        2. logpath: directory path of the log file
            Type: string
            Default: '/home/amigos/data/rxmonitor/'

        RETURNS
        ================
        Nothing.
        '''
        utc = datetime.datetime.utcnow()
        ts = utc.strftime('%Y%m%d')
        while 1:
            if os.path.isfile(logpath+'rxmonitor%s.csv'%(ts)) == 0:
                f = open(logpath+'rxmonitor%s.csv'%(ts), 'w')
                writecsv = csv.writer(f)
                writecsv.writerow(['#UTC', 'room_temp1[K]', 'room_temp2[K]', 'dewar_pressure[torr]', 'dewar_temp[K]'])
                f.close()
            else:
                f = open(logpath+'rxmonitor%s.csv'%(ts), 'a')
                writecsv = csv.writer(f)
                ret = self.mon.measure()
                writecsv.writerow([ret[0], ret[1], ret[2], ret[3], ret[4]])
                f.close()
                time.sleep(float(interval))
        return

#written by K.Urushihara
