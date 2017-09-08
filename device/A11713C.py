#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import time, sys
import pymeasure

class a11713c(object):
    '''
    DESCRIPTION
    ================
    This class cntrols the Agilent 11713C

    ARGUMENTS
    ================
    1. IP: IP address of the 11713C
        Type: string
        Default: '192.168.100.1'
    2. port: port number of the 11713C
        Type: int
        Default: 5025
    '''

    def __init__(self, IP='192.168.100.1', port=5025):
        self.IP = IP
        self.port = port
        self.com = pymeasure.ethernet(IP, port)
        
    def set_model(self, model, ch):
        """        
        DESCRIPTION
        ================
        This function sets the model of the programmable attenuator.
        
        ARGUMENTS
        ================
        1. model: model of the attenuator
            Number: 'NA', 'AG8494g', 'AG8495g', 'AG8495k',
                    'AG8496g', 'AG8497k', 'AG84904k', 'AG84905m',
                    'AG84906k', 'AG84907k' or 'AG84908m'
                    *for AG849xh, use 'AG849xg'
                    *for AG84904l/m, use 'AG84904k'
                    *for AG84906l, use 'AG84906k'
                    *for AG84907l, use 'AG84907k'
            Type: string
            Default: 'AG8494g'

        2. ch: channel of the A11713C
            Number: '1X', '1Y', '2X' or '2Y'
            Type: string
            Default: Nothing.
        
        RETURNS
        ================
        Nothing.
        """
        modellist = ['NA', 'AG8494g', 'AG8495g', 'AG8495k', 'AG8496g', 'AG8497k',
                     'AG84904k', 'AG84905m', 'AG84906k', 'AG84907k', 'AG84908m']
        if model in modellist:
            self.com.open()
            if ch=='1X':
                self.com.send('CONFigure:BANK1:X %s' %(model))
            elif ch=='1Y':
                self.com.send('CONFigure:BANK1:Y %s' %(model))
            elif ch=='2X':
                self.com.send('CONFigure:BANK2:X %s' %(model))
            elif ch=='2Y':
                self.com.send('CONFigure:BANK2:Y %s' %(model))
            else:
                print('!!!!ERROR!!!!')
                print('invalid ch: '+str(ch))
                print('available ch: "1X", "1Y", "2X" or "2Y"')
            self.com.close()
        else:
            print('!!!!ERROR!!!!')
            print('invalid model: '+str(model))
            print('available model: ')
            print(modellist)
        return 

    def query_model(self):
        """        
        DESCRIPTION
        ================
        This function queries the model of the programmable attenuator.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. model: model of the attenuator
            Type: list [1X, 1Y, 2X, 2Y]
                *for AG849xh, use 'AG849xg'
                *for AG84904l/m, use 'AG84904k'
                *for AG84906l, use 'AG84906k'
                *for AG84907l, use 'AG84907k'
        """
        self.com.open()
        self.com.send('CONFigure:BANK1:X?')
        ret1 = self.com.readline()
        self.com.send('CONFigure:BANK1:Y?')
        ret2 = self.com.readline()
        self.com.send('CONFigure:BANK2:X?')
        ret3 = self.com.readline()
        self.com.send('CONFigure:BANK2:Y?')
        ret4 = self.com.readline()
        self.com.close()
        att1X = str(ret1.replace('\n', ''))
        att1Y = str(ret2.replace('\n', ''))
        att2X = str(ret3.replace('\n', ''))
        att2Y = str(ret4.replace('\n', ''))
        model = [att1X, att1Y, att2X, att2Y]
        return model

    def set_level(self, level, ch):
        """        
        DESCRIPTION
        ================
        This function sets the attenuation level.
        
        ARGUMENTS
        ================
        1. level: attenuation level
            Number: 0, 1, 2, ...
            Type: int
            Default: Nothing.

        2. ch: channel of the A11713C
            Number: '1X', '1Y', '2X' or '2Y'
            Type: string
            Default: Nothing.
        
        RETURNS
        ================
        Nothing.
        """
        if level>=0 and type(level)==int:
            self.com.open()
            if ch=='1X':
                self.com.send('ATTenuator:BANK1:X %s' %(level))
            elif ch=='1Y':
                self.com.send('ATTenuator:BANK1:Y %s' %(level))
            elif ch=='2X':
                self.com.send('ATTenuator:BANK2:X %s' %(level))
            elif ch=='2Y':
                self.com.send('ATTenuator:BANK2:Y %s' %(level))
            else:
                print('!!!!ERROR!!!!')
                print('invalid ch: '+str(ch))
                print('available ch: "1X", "1Y", "2X" or "2Y"')
            self.com.close()
        else:
            print('!!!!ERROR!!!!')
            print('invalid level: '+str(level))
            print('available level: 0, 1, 2, ...')
        return 

    def query_level(self):
        """        
        DESCRIPTION
        ================
        This function queries the attenuation level.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. level: attenuation level
            Type: list [1X, 1Y, 2X, 2Y]
        """
        self.com.open()
        self.com.send('ATTenuator:BANK1:X?')
        ret1 = self.com.readline()
        self.com.send('ATTenuator:BANK1:Y?')
        ret2 = self.com.readline()
        self.com.send('ATTenuator:BANK2:X?')
        ret3 = self.com.readline()
        self.com.send('ATTenuator:BANK2:Y?')
        ret4 = self.com.readline()
        self.com.close()
        att1X = int(ret1)
        att1Y = int(ret2)
        att2X = int(ret3)
        att2Y = int(ret4)
        level = [att1X, att1Y, att2X, att2Y]
        return level

    def set_voltage(self, voltage, bank):
        """        
        DESCRIPTION
        ================
        This function sets the supply voltage for each bank.
        
        ARGUMENTS
        ================
        1. voltage: supply voltage
            Number: 'OFF', '5V', '15V', '24V' or 'USER'
            Type: string
            Default: Nothing.

        2. bank: bank of the A11713C
            Number: 1 or 2
            Type: int
            Default: Nothing.
        
        RETURNS
        ================
        Nothing.
        """
        vlist = ['OFF', '5V', '15V', '24V', 'USER']
        if voltage in vlist:
            self.com.open()
            if bank==1:
                self.com.send('CONFigure:BANK1 %s' %(voltage))
            elif bank==2:
                self.com.send('CONFigure:BANK2 %s' %(voltage))
            else:
                print('!!!!ERROR!!!!')
                print('invalid bank: '+str(bank))
                print('available bank: 1 or 2')
            self.com.close()
        else:
            print('!!!!ERROR!!!!')
            print('invalid voltage: '+str(voltage))
            print('available voltage: ')
            print(vlist)
        return 

    def query_voltage(self):
        """        
        DESCRIPTION
        ================
        This function queries the supply voltage for each bank.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. voltage: supply voltage
            Type: string list ('OFF', 'P5', 'P15', 'P24' or 'USER')

        """
        self.com.open()
        self.com.send('CONFigure:BANK1?')
        ret1 = self.com.readline()
        ret1 = ret1.replace('\n', '')
        if ret1=='OFF':
            bank1='OFF'
        elif ret1=='P5':
            bank1='5V'
        elif ret1=='P15':
            bank1='15V'
        elif ret1=='P24':
            bank1='24V'
        elif ret1=='USER':
            bank1='USER'
        else:
            bank1='UNKNOWN'
        self.com.send('CONFigure:BANK2?')
        ret2 = self.com.readline()
        self.com.close()
        ret2 = ret2.replace('\n', '')
        if ret2=='OFF':
            bank2='OFF'
        elif ret2=='P5':
            bank2='5V'
        elif ret2=='P15':
            bank2='15V'
        elif ret2=='P24':
            bank2='24V'
        elif ret2=='USER':
            bank2='USER'
        else:
            bank2='UNKNOWN'
        voltage = [bank1, bank2]
        return voltage

# written by K.Urushihara
# 2017/07/31 T.Inaba: fix a mistake (add "," to modellist)
# 2017/09/08 T.Inaba: delete sys.path to pymeasure