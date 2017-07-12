#! /usr/bin/env python
# -*- coding: utf-8 -*-
#ver. 1.0


import time, sys, os, csv
sys.path.append('/home/amigos/NASCORX_System-master/device/')
import A11713A
sys.path.append('/home/amigos/NECRX_system/pymeasure2-master/')
import pymeasure
import numpy as np
import matplotlib.pyplot as plt

datapath = '/home/amigos/data/speana/data'
savepath = '/home/amigos/data/speana/gragh'

if os.path.isdir(datapath):
	pass
else:
	os.makedirs(datapath)

if os.path.isdir(savepath):
	pass
else:
	os.makedirs(savepath)

savedatadir = "/home/amigos/data/speana/data"
savegraghdir = "/home/amigos/data/speana/gragh"

#att = A11713A.a11713a()
#sp = IF_spectrum.if_spectrum()

IP_SA = "192.168.100.130"
port_SA = 49153
com_SA = pymeasure.ethernet(IP_SA, port_SA)
sp = pymeasure.Agilent.N9343C(com_SA)


#save file
ts_start = time.strftime("%m%d%H%M")
filename = 'spectrum'+ts_start+".csv"

sp.com.send('FREQ:STAR?')
start_freq = float(sp.com.readline())
sp.com.send('FREQ:STOP?')
stop_freq = float(sp.com.readline())
sp.com.send('SWE:POIN?')
trace_point = float(sp.com.readline())
freq = np.linspace(start_freq, stop_freq, trace_point)
sa = np.array([freq])

#sa means "spec array"

time.sleep(10)
sp.com.send('TRAC? TRAC1')
spec_raw = sp.com.readline()
spec = np.array([spec_raw.split(',')], float)
sa = np.r_[sa, spec]
print "hot dB finished"
time.sleep(10)
sp.com.send('TRAC? TRAC1')
spec_raw = sp.com.readline()
spec2 = np.array([spec_raw.split(',')], float)
sa = np.r_[sa, spec2]
print "cold dB finished"

np.savetxt(filename, sa, delimiter=",")

#plot part

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
ax1.plot(sa[0], sa[1], label="hot[dB]")
ax1.plot(sa[0], sa[2], label="cold[dB]")
ax1.set_xlabel("frequency[GHz]")
ax1.set_ylabel("Amplitude [dBm]")
ax1.set_title(title+"_raw data")
ax1.set_xlim([0, np.amax(sa[0])])
ax1.set_ylim([np.amin(sa[12])-10, np.amax(sa[1])+10])
ax1.grid()
ax1.legend(bbox_to_anchor=(1.01, 1.012), loc='upper left', prop={'size': 15})
plt.subplots_adjust(left=0.1, right=0.8)
plt.savefig(savegraghdir + ts_start + "_rawdata.png")

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
ax1.plot(sa[0], sa[2]-sa[1], label="hot - cold[dB]")
ax1.set_xlabel("frequency[GHz]")
ax1.set_ylabel("Amplitude [dBm]")
ax1.set_title(title)
ax1.set_xlim([0, np.amax(sa[0])])
ax1.grid()
ax1.legend(bbox_to_anchor=(1.01, 1.012), loc='upper left', prop={'size': 15})
plt.hlines(np.linspace(-15, 2, 35), 0, np.amax(sa[0]), linestyles="dashed")
plt.yticks(np.linspace(-15, 3, 19))
ax1.set_ylim(np.amin(sa[12]-sa[1])-1, np.amax(sa[2]-sa[1]+1))
plt.subplots_adjust(left=0.1, right=0.8)
plt.savefig(savegraghdir + ts_start + "_sadata.png")
