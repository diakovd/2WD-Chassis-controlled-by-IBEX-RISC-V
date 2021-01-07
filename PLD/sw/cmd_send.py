# -*- coding: utf-8 -*-
"""
Created on Mon Dec 28 22:38:30 2020

@author: Dmitry
"""

import serial
import time

ser = serial.Serial(port = 'COM4', baudrate=921600, parity=serial.PARITY_ODD)
if(ser.isOpen() == False):
    ser.open()
    
dat = bytearray('cmd',encoding = 'utf-8') 
dat.append(0x1)
dat.append(0x1)
ser.write(dat)

t1 = time.time()
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
  
print(ser.read(5))
t2 = time.time()

print(t2-t1)
ser.close()