#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import os
import numpy
import matplotlib.pyplot as plt



class allan(object):
    def __init__(self):
        pass

    def tau_calc(self, repeat, integtime):
        tau = numpy.linspace(1, int(repeat/2) - 1, int(repeat/2) - 1) * integtime
        return tau

    def allan_calc(self, data, integtime):
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

    def allan_fitting(self):
        return








