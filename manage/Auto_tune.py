#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import sys, time, datetime, os, csv
sys.path.append('/home/amigos/NASCORX-master/base/')
import Cryo

class mixertune(object):
    '''
    DESCRIPTION
    ================
    This class tunes the SIS mixer.

    ARGUMENTS
    ================
    Nothing.
    '''
    def __init__(self):
        self.mix = Cryo.mixer()
        self.spec = XFFTS.xffts()

    def searching(self, Vrange=[0, 10000], Vres=100, Ires=10, logpath='/home/amigos/data/SIS/'):
        '''        
        DESCRIPTION
        ================
        This function measures SIS mixer noise.
        
        ARGUMENTS
        ================
        1. Vrange: searching area of SIS voltage [uV]
            Number: 0 -- 50,000 [uV]
            Type: list[int, int]
            Default: [0, 10000]
        2. Vres: searching resolutin of SIS voltage [uV]
            Number: 1 -- 50,000 [uV]
            Type: int
            Default: 100
        3. Ires: searching resolutin of LO attenuator [mA]
            Number: 1 -- 100 [mA]
            Type: int
            Default: 10
        4. logpath: directory path of the log file
            Type: string
            Default: '/home/amigos/data/SIS/'

        RETURNS
        ================
        Nothing.
        '''        
        utc = datetime.datetime.utcnow()
        ts = utc.strftime('%Y%m%d')
        for v in range(Vrange[0], Vrange[1], Vres):
            if os.path.isfile(logpath+'mixertune%s.csv'%(ts)) == 0:
                f = open(logpath+'mixertune%s.csv'%(ts), 'w')
                writecsv = csv.writer(f)
                writecsv.writerow(['#UTC', 'voltage[mV]', 'current[uA]', 'Y-factor[dB]'])
                f.close()
            else:
                self.mix.set_sisv(voltage=v*1e-6)
                for i in range(0, 100, Ires):
                    self.mix.set_loatt(att=i)
                    ret1 = self.mix.monitor_sis()
                    ret2 = self.sepc.yfactor()
                    f = open(logpath+'mixertune%s.csv'%(ts), 'a')
                    writecsv = csv.writer(f)
                    writecsv.writerow([ret1, ret2])
                    f.close()
        return

class hemttune(object):
    '''
    DESCRIPTION
    ================
    This class tunes the HEMT amplifire.

    ARGUMENTS
    ================
    Nothing.
    '''
    def __init__(self):
        self.hemt = Cryo.hemt()
        self.spec = XFFTS.xffts()

    def searching(self, Vdrange=[0, 2500], Vdres=100, Vgrange=[-2500, 2500], Vgres=100, logpath='/home/amigos/data/HEMT/'):
        '''        
        DESCRIPTION
        ================
        This function measures HEMT amplifire noise.
        
        ARGUMENTS
        ================
        1. Vdrange: searching area of drain voltage
            Number: 0 -- 2,500 [mV]
            Type: list[int, int]
            Default: [0, 2500]
        2. Vdres: searching resolutin of drain voltage
            Number: 1 -- 2,500 [mV]
            Type: int
            Default: 100
        3. Vgrange: searching area of gate voltage
            Number: -2,500 -- 2,500 [mV]
            Type: list[int, int]
            Default: [-2500, 2500]
        4. Vdres: searching resolutin of drain voltage
            Number: 1 -- 2,500 [mV]
            Type: int
            Default: 100
        5. logpath: directory path of the log file
            Type: string
            Default: '/home/amigos/data/HEMT/'

        RETURNS
        ================
        Nothing.
        '''        
        utc = datetime.datetime.utcnow()
        ts = utc.strftime('%Y%m%d')
        for Vd in range(Vdrange[0], Vdrange[1], Vdres):
            if os.path.isfile(logpath+'hemttune%s.csv'%(ts)) == 0:
                f = open(logpath+'hemttune%s.csv'%(ts), 'w')
                writecsv = csv.writer(f)
                writecsv.writerow(['#UTC', 'Vd[mV]', 'Id[mA]', 'Vg1[mV]', 'Vg2[mV]', 'Y-factor[dB]'])
                f.close()
            else:
                self.hemt.set_Vd(voltage=Vd*1e-6)
                for Vg in range(Vgrange[0], Vgrange[1], Vgres):
                    self.hemt.set_Vg(voltage=Vg*1e-6)
                    ret1 = self.hemt.query_Vd()
                    ret2 = self.hemt.query_Id()
                    ret3 = self.hemt.query_Vg()
                    ret4 = self.hemt.query_Vg()
                    f = open(logpath+'hemttune%s.csv'%(ts), 'a')
                    writecsv = csv.writer(f)
                    writecsv.writerow([ret1, ret2, ret3, ret4])
                    f.close()
        return

#written by K.Urushihara
