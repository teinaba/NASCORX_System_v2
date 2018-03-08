#! /usr/bin/env python
# -*- coding: utf-8 -*-

# import modules
import time
import sys
import os
import numpy
import threading
import matplotlib.pyplot

from NASCORX_XFFTS.data_client import data_client

client = data_client()

class allan(object):
    spec = None
    conti = None
    btemp = None

    def __init__(self):
        pass

    def get_spectrum(self, integtime, repeat):
        data = client.oneshot(integtime=integtime, repeat=repeat)
        self.spec = data
        return

    def get_continuum(self, integtime, repeat):
        data = client.conti_oneshot(integtime=integtime, repeat=repeat)
        self.conti = data
        return

    def get_btemp(self, integtime, repeat):
        sec = int(integtime * repeat)
        data = client.btemp_oneshot(sec=sec)
        self.btemp = data
        return

    def measure(self, integtime, repeat):
        th1 = threading.Thread(target=self.get_spectrum(integtime=integtime, repeat=repeat))
        th2 = threading.Thread(target=self.get_spectrum(integtime=integtime, repeat=repeat))
        th3 = threading.Thread(target=self.get_btemp(integtime=integtime, repeat=repeat))
        th1.setDaemon(True)
        th2.setDaemon(True)
        th3.setDaemon(True)
        th1.start()
        th2.start()
        th3.start()
        th1.join()
        th2.join()
        th3.join()

        print('\n'
              'End Measurement'
              '\n')
        return self.spec, self.conti, self.btemp

    def tau_calc(self, repeat, integtime):
        tau = numpy.linspace(1, int(repeat/2) - 1, int(repeat/2) - 1) * integtime
        return tau

    def calc_adev(self, data, integtime):
        repeat = len(data)
        tau = numpy.linspace(1, int(repeat/2) - 1, int(repeat/2) - 1) * integtime
        allan_result = []
        data_cum = numpy.cumsum(data)  # data cumsum
        for i in range(1, int(repeat/2)):
            data_sum = 0
            for j in range(0, repeat - 2*i):
                temp = (data_cum[j+2*i] - 2 * data_cum[j+i] + data_cum[j]) ** 2
                data_sum = data_sum + temp

            sigma = 1. / (2 * (repeat - 2*i) * (integtime * i) ** 2) * data_sum
            allan_result.append(sigma)
        allan = numpy.concatenate((numpy.array([tau]), numpy.array([allan_result])), axis=0)
        return allan

    def calc(self, data, integtime):
        

    def fit(self):
        return

    def plot(self, allan):
        return








