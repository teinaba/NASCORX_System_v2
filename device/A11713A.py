#! /usr/bin/env python
# _*_ coding: UTF-8 _*_

import time, sys
import pymeasure

class a11713a:
    com = pymeasure.gpib_prologix("172.20.0.16", 8)
        
    def attenuator(self, IF1=11, IF2=11):
        """
        This command sets the attenuation.
        
        Args
        ====
        < IF1, IF2 : int : 0-11 >
            Attenuation value
            default = 11
        
        Returnes
        ========
        Nothing.
        
        Examples
        ========
        >>> at.attenuator(5, 5)
        >>> at.attenuator()
        """
        table1 = ['B1234','A1B234', 'A2B134', 'A12B34', 'A3B124', 
                  'A13B24', 'A23B14', 'A123B4', 'A34B12', 'A134B2',
                  'A234B1', 'A1234']
        table2 = ['B5678','A5B678', 'A6B578', 'A56B78', 'A7B568', 
                  'A57B68', 'A67B58', 'A567B8', 'A78B56', 'A578B6',
                  'A678B5', 'A5678']
        self.com.open()
        self.com.send(table1[IF1]+table2[IF2])
        f = open('/home/amigos/NECRX_system/device_cntrl/latest.txt', 'w')
        f.write(str(IF1)+','+str(IF2))
        f.close()
        return

'''
Last Update 2016/02/13

***HISTORY***
*2015/01/05 K.Urushihara
add prog_att()
*2016/01/15 K.Urushihara
add comments about prog_att()
*2016/02/13 K.Urushihara
change IP and GPIB address.
*************
'''

# 2017/09/08 T.Inaba: delete sys.path to pymeasure