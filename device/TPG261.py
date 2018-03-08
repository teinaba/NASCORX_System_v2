#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


import time
import socket


class tpg261(object):
    """
    DESCRIPTION
    ================
    This class controls the TPG261.
    ////TPG261 Data Format////
    start bit: 1
    data bit: 8
    parity: No
    stop bit:1
    bau rate: 9600
    hardware handshake: No

    ////FA-210 Setting////
    General settings
        1) Start up
            1) Gratuitous ARP - Enable
            2) Bootp control - Run only at factory default
        2) Ethernet physical I/F
            1) Auto-negotiation - Enable
            2) Speed(bps) - 100M
            3) Duplex mode - Full duplex
        3) TCP/IP
        4) Syslog
        5) Real time clock
        6) Administrator

    Conversion settings - TCP Transparent mode
        1) Serial port
            1) Speed(bps) - 9600
            2) Data bits - 8
            3) Parity - None
            4) Stop bits - 1
            5) Flow control - RTS/CTS
            6) XON code - 11 (hex)
            7) XOFF code - 13 (hex)
            8) Frame decision, Idle time - 3 msec
        2) Connection type - Server
        3) Server connection
            1) TCP port - 9600
            2) Ping keepalive - Disable
            3) Ping interval - 60 sec
            4) Ping reply timer - 10 sec
            5) Ping maximum retries for disconnect - 1
        4) Client connection
        5) Timer
        6) DTR/RTS signal
        7) Ethernet link monitor

    ARGUMENTS
    ================
    1. IP: IP address of the FA-210
        Type: string
        Default: '192.168.100.1'
    2. port: port number of the FA-210
        Type: int
        Default: 9600
    """
    def __init__(self, IP='192.168.100.1', port=9600):
        self.IP = IP
        self.port = port
        self.clientsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.clientsock.connect((self.IP, self.port))

    def query_pressure(self):
        """
        DESCRIPTION
        ================
        This function queries the pressure value.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. pressure: pressure value of the TPG261
            Type: float [Bar, Torr or Pascal]
        """
        self.clientsock.sendall(bytes('PR1 \r\n', 'utf8'))
        time.sleep(0.5)
        self.clientsock.sendall(bytes('\x05', 'utf8'))
        time.sleep(0.5)
        raw = self.clientsock.recv(256)
        raw_dec = raw.decode('utf8')
        line = raw_dec.split('\r\n')
        ret = line[1].split(',')
        pressure = float(ret[1])
        return pressure

    def query_unit(self):
        """
        DESCRIPTION
        ================
        This function queries the unit of pressure.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. unit: unit of pressure
            Type: string ('bar', 'torr', or 'pascal')
        """
        parameter = ['bar', 'torr', 'pascal']
        self.clientsock.sendall(bytes('UNI \r\n', 'utf8'))
        time.sleep(0.5)
        self.clientsock.sendall(bytes('\x05', 'utf8'))
        time.sleep(0.5)
        raw = self.clientsock.recv(256)
        raw_dec = raw.decode('utf8')
        line = raw_dec.split('\r\n')
        ret = line[1].split(',')
        index = int(ret[0])
        if index == 0 or 1 or 2:
            unit = parameter[index]
        else:
            unit = 'UNKNOWN'
        return unit

    def set_unit(self, unit='torr'):
        """
        DESCRIPTION
        ================
        This function sets the unit of pressure.
        
        ARGUMENTS
        ================
        1. unit: unit of pressure
            Number: 'bar', 'torr' or 'pascal'
            Type: string
            Default: 'torr'
        
        RETURNS
        ================
        Nothing.
        """
        if unit == 'bar':
            self.clientsock.sendall(bytes('UNI,0 \r\n', 'utf8'))
        elif unit == 'torr':
            self.clientsock.sendall(bytes('UNI,1 \r\n', 'utf8'))
        elif unit == 'pascal':
            self.clientsock.sendall(bytes('UNI,2 \r\n', 'utf8'))
        else:
            print('!!!!ERROR!!!!')
            print('invalid unit: '+str(unit))
            print('available unit: "bar", "torr" or "pascal"')
            quit()
        time.sleep(0.5)
        self.clientsock.sendall(bytes('\x05', 'utf8'))
        time.sleep(0.5)
        raw = self.clientsock.recv(256)
        raw_dec = raw.decode('utf8')
        return 

    def query_baurate(self):
        """
        DESCRIPTION
        ================
        This function queries the bau rate of the TPG261.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. bau: bau rate of the TPG261
            Type: int (9600, 19200 or 38400) or 0 (IF index is INVALID)
        """
        parameter = [9600, 19200, 38400]
        self.clientsock.sendall(bytes('BAU \r\n', 'utf8'))
        time.sleep(0.5)
        self.clientsock.sendall(bytes('\x05', 'utf8'))
        time.sleep(0.5)
        raw = self.clientsock.recv(256)
        raw_dec = raw.decode('utf8')
        line = raw_dec.split('\r\n')
        ret = line[1].split(',')
        index = int(ret[0])
        if index == 0 or 1 or 2:
            bau = int(parameter[index])
        else:
            bau = 0
        return bau

    def set_baurate(self, rate=9600):
        """
        DESCRIPTION
        ================
        This function sets the bau rate of the TPG261.
        
        ARGUMENTS
        ================
        1. bau: bau rate of the TPG261
            Number: 9600, 19200 or 38400
            Type: int
            Default: 9600 (Recommendation)
        
        RETURNS
        ================
        Nothing.
        """
        if rate == 9600:
            self.clientsock.sendall(bytes('BAU,0 \r\n', 'utf8'))
        elif rate == 19200:
            self.clientsock.sendall(bytes('BAU,1 \r\n', 'utf8'))
        elif rate == 38400:
            self.clientsock.sendall(bytes('BAU,2 \r\n', 'utf8'))
        else:
            print('!!!!ERROR!!!!')
            print('invalid bau rate: '+str(rate))
            print('available bau rate: 9600, 19200 or 38400')
            quit()
        time.sleep(0.5)
        self.clientsock.sendall(bytes('\x05', 'utf8'))
        time.sleep(0.5)
        raw = self.clientsock.recv(256)
        raw_dec = raw.decode('utf8')
        return

    def query_gauge(self):
        """
        DESCRIPTION
        ================
        This function queries the pressure value.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        1. sensor: sensor type connected to the TPG261
            Type: list ([0]: ch1, [1]: ch2)
        """
        self.clientsock.sendall(bytes('TID\r\n', 'utf8'))
        time.sleep(0.5)
        self.clientsock.sendall(bytes('\x05', 'utf8'))
        time.sleep(0.5)
        raw = self.clientsock.recv(1024)
        raw_dec = raw.decode('utf8')
        line = raw_dec.split('\r\n')
        ret = line[1].split(',')
        return ret


'''
Last Update 2017/10/23

***HISTORY***
*2015/11/26 K.Urushihara
create file
*2016/01/15 K.Urushihara
add measure()
add gauge_query()
*2017/10/23 T.Inaba
adapt to Python 3
*************
'''
