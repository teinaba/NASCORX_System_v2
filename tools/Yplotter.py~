# ! /usr/bin/env python
# _*_ coding: UTF-8 _*_


# import modules
import time, sys, datetime, os, csv
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.dates as mdates
import numpy as np

def plotter(filename, datapath='/home/amigos/data/SIS/', savepath='/home/amigos/data/SIS/Ymap/', gragh=1):
    '''
    DESCRIPTION
    ================

    ARGUMENT
    ================
        1. : 
            Type: 

c20170602075003    RETURN
    ================
    Nothing.
    '''
    #==== Data File Search ====
    filepath = datapath+filename
    if os.path.isfile(filepath) == 0:
        print(filepath)
        print('ERROR: No such file !!!')
    else:
    	print('INPUT Room Temperature [℃]')
        ret = raw_input()
        Tamb = float(ret) + 273.2
        # Data Read
        data = np.genfromtxt(filepath, dtype=None, comments="#", delimiter=",")
        ut = np.array([])
        freq = np.array([])
        freq_a = ([])
        vol = np.array([])
        cur = np.array([])
        yfac = np.array([])
        trx = np.array([])
        LOatt = np.array([])
        v_list = np.array([])
        LOatt_list = np.array([])
        for i in range(len(data)):
            ut = np.append(ut, data[i][0]) #Universal Time
            freq = np.append(freq, data[i][1]) # LO Frequency [GHz]
            vol = np.append(vol, data[i][2]) # Mixer Voltage [mV]
            cur = np.append(cur, data[i][3]) # Mixer Current [uA]
            yfac = np.append(yfac, (float(data[i][4])-float(data[i][5]))) # Y-factor [dB]
            LOatt = np.append(LOatt, data[i][6]) # LO attenator [%]
            
        #==== Splitting Data by LO Freq. ====
        # Get LO Border Index
        freq_idx = np.array([])
        j = 0
        for i in range(len(freq)):
            if freq[j] != freq[i]:
                freq_idx = np.append(freq_idx, j)
                freq_a = np.append(freq_a, freq[j])
                j = i
            else:
                pass
        freq_idx = np.append(freq_idx, j)
        freq_a = np.append(freq_a, freq[j])
        
        # Splitting Data
        for i in range(len(freq_idx)):
            if i == len(freq_idx)-1:
                vol_col = vol[int(freq_idx[i]):]
                cur_col = cur[int(freq_idx[i]):]
                yfac_col = yfac[int(freq_idx[i]):]
                LOatt_col = LOatt[int(freq_idx[i]):]
            else:
                vol_col = vol[int(freq_idx[i]):int(freq_idx[i+1])]
                cur_col = cur[int(freq_idx[i]):int(freq_idx[i+1])]
                yfac_col = yfac[int(freq_idx[i]):int(freq_idx[i+1])]
                LOatt_col = LOatt[int(freq_idx[i]):int(freq_idx[i+1])]
            #calculate Trx&serch best state
            yfac_max = max(yfac_col)
            yfac_max1 = 10**(yfac_max/10)
            trx1 = (Tamb - 77*yfac_max1)/(yfac_max1 - 1)
            trx = np.append(trx, trx1)
            yfac_list = yfac_col.tolist()
            ind = yfac_list.index(max(yfac_list))
            vol_max = vol_col[ind]
            LOatt_max = LOatt_col[ind]
            v_list = np.append(v_list, vol_max)
            LOatt_list = np.append(LOatt_list, LOatt_max)
            #==== Make 2D Array ====
            # Get SIS Voltage Border Index
            vol_idx = np.array([])
            j = 0
            for k in range(len(vol_col)):
                if vol_col[j] != vol_col[k]:
                    vol_idx = np.append(vol_idx, j)
                    j = k
                else:
                    pass
            vol_idx = np.append(vol_idx, j)
            
            # Define Array Length
            row_length = int(vol_idx[1] - vol_idx[0])
            column_length = int(len(vol_col)/row_length)
            print(row_length)
            print(column_length)
            
            # Reshape Array
            X = np.reshape(vol_col, (column_length, row_length))
            X = X.T
            Y = np.reshape(cur_col, (column_length, row_length))
            Y = Y.T
            Z = np.reshape(yfac_col, (column_length, row_length))
            Z = Z.T
            #==== Plot Y-factor Map ====
            plt.pcolormesh(X, Y, Z, cmap=cm.jet , shading='flat', edgecolors='None', alpha=1, vmin=Z.min(), vmax=Z.max())
            plt.axis([X.min(), X.max(), 15, 80])
            plt.title('Y-factor Map (LO = '+str(round(freq[int(freq_idx[i])], 2))+' GHz)')
            plt.xlabel('Mixer Voltage [mV]')
            plt.ylabel('Mixer Current [uA]')
            CB = plt.colorbar()
            CB.set_label('Y-factor')
            plt.grid()
            plt.annotate('Vsis='+str(vol_max), xy=(0.95, -0.05), xycoords='axes fraction', fontsize=12,horizontalalignment='left', verticalalignment='bottom')
            plt.annotate('LOatt='+str(LOatt_max), xy=(0.95, -0.10), xycoords='axes fraction', fontsize=12,horizontalalignment='left', verticalalignment='bottom')
            plt.savefig(savepath+filename.replace('.csv', '_LO')+str(round(freq[int(freq_idx[i])], 2))+'.png')
            if gragh == 1:
                plt.show()
            else:
        	    pass
        	    
            plot = plt.scatter(vol_col, cur_col, c=yfac_col, cmap=cm.jet, edgecolors='none', s=100, vmin=Z.min(), vmax=Z.max(), marker='o')
            CB = plt.colorbar(plot)
            CB.set_label("Y-factor")
            plt.title('Y-factor plot')        
            plt.xlabel('Mixer Voltage [mV]')
            plt.ylabel('Mixer Current [uA]')
            plt.grid(True)
            plt.xlim(X.min(), X.max()) #X.min(),X.max()
            plt.ylim(15, 80) #Y.min(),Y.max()
            plt.savefig(savepath+filename.replace('.csv', '_LO')+str(round(freq[int(freq_idx[i])], 2))+'_scatter.png')
            if gragh == 1:
                plt.show()
            else:
        	    pass
        #plot Trx
        plt.plot(freq_a, trx, marker="D")
        plt.title('Trx')
        plt.xlabel('Frequency [GHz]')
        plt.ylabel('Trx [K]')
        plt.savefig(savepath+'Trx/'+filename.replace('.csv', '_LO')+'_Trx.png')
        if gragh == 1:
            plt.show()
        else:
        	pass
print('INPUT WHEN FILE')
ret = raw_input()
filename = 'mixerYmap' + ret + '.csv'
plotter(filename, gragh=1)

# written by K.Urushihara
