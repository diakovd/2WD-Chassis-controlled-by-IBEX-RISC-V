# -*- coding: utf-8 -*-
"""
Created on Mon Dec 28 22:38:30 2020

@author: Dmitry
"""

import serial
import time

ser = serial.Serial(port = 'COM3', baudrate=921600,timeout=1, parity=serial.PARITY_ODD)
if(ser.isOpen() == False):
    ser.open()
     
dat = bytearray('cmd',encoding = 'utf-8') 
dat.append(0x1)
dat.append(0x4)
ser.write(dat)

#t1 = time.time()
'''
dat = bytearray('cmd',encoding = 'utf-8') 
dat.append(0x2)
dat.append(0xbf)
ser.write(dat)

dat = bytearray('cmd',encoding = 'utf-8') 
dat.append(0x3)
dat.append(0xbf)
ser.write(dat)
'''
line = ''
rx = ''
rpm = 0
cnt = 0
for i in range(1500):
    rx = ser.read(1).decode('utf-8')
    if(rx == '\n'):
        st  = line.index('=') + 1
        end = len(line)
        rpm = rpm + int(line[st:end])
        cnt = cnt + 1
        #print(line)
        line = ''
    else:
        line = line + rx
rpm = rpm/cnt
print('rpm',rpm)

dat = bytearray('cmd',encoding = 'utf-8') 
dat.append(0x1)
dat.append(0x0)
ser.write(dat)

ser.close()