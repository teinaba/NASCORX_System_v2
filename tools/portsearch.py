#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


#import modules
import time, sys
sys.path.append('/home/amigos/pymeasure2-master/')
import pymeasure


def Ethernet_search(IP, sport=0, eport=65535):
    '''
    DESCRIPTION
    ================
    This function searches the port responding to '*IDN?'

    ARGUMENTS
    ================
    1. IP: IP adress
        Type: string
    2. sport: searching start port
        Number: 0 -- 65535
        Type: int
    3. eport: searching stop port
        Number: 0 -- 65535
        Type: int
    
    RETURNS
    ================
    Nothing.
    '''
    for i in range(sport, eport+1, 1):
        com = pymeasure.ethernet(IP, i)
        try:
            com.open()
            com.send('*IDN?')
            ret = com.readline()
            com.close()
        except Exception:
            sys.stdout.write('\rtest port: %d' %i)
            sys.stdout.flush()
        else:
            print('  (o _ o)!!! ')
            print('port = '+str(i))
            print(ret)

def GPIB_search(IP, sport=0, eport=30):
    '''
    DESCRIPTION
    ================
    This function searches the GPIB port responding to '*IDN?'

    ARGUMENTS
    ================
    1. IP: IP adress
        Type: string
    2. sport: searching start port
        Number: 0 -- 30
        Type: int
    3. eport: searching stop port
        Number: 0 -- 30
        Type: int

    RETURNS
    ================
    Nothing.
    '''
    for i in range(sport, eport+1, 1):
        com = pymeasure.gpib_prologix(IP, i)
        try:
            com.open()
            com.send('*IDN?')
            ret = com.readline()
            com.close()
        except Exception:
            sys.stdout.write('\rtest port: %d' %i)
            sys.stdout.flush()
        else:
            print('  (o _ o)!!! ')
            print('port = '+str(i))
            print(ret)



#written by K.Urushihara


