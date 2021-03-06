# -*- coding: utf-8 -*-
"""
Created on Mon Dec 28 22:38:30 2020

@author: Dmitry
"""

import serial
import time


fw= open("rpmBpwm.txt","w")

ser = serial.Serial(port = 'COM5', baudrate=9600,timeout=1, parity=serial.PARITY_NONE)
if(ser.isOpen() == False):
    ser.open()
    
def cmd_send(cmd,val):
    dat = bytearray('cmd',encoding = 'utf-8') 
    dat.append(cmd)
    dat.append(val)
    print(dat)
    ser.write(dat)

# Whell on enable bit  
Left  = 0x4
Right = 0x8

# cmd define
drive = 0x1
SpeedLeft  = 0x2
SpeedRight = 0x3
Cnt_Left   = 0x4    #set count of left  whell decoder
Cnt_Right  = 0x5    #set count of Right whell decoder


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

rx = ''
lineWR = 'pwm rpm \n'
fw.write(lineWR)

def pwmTOrpm():
    for j in range(0xfe,0,-1):
        
        cmd_send(PWM_left, j)
        print('pwm',hex(j))
        fw.write((hex(j) + ' '))
        cmd_send(0x1, 0x04)
        
        rpm = 0
        cnt = 0 
        line = ''
        
        for i in range(1500):
            rx = ser.read(1).decode('utf-8')
            if(rx == ''): break
            elif(rx == '\n'):
                #print(line)
                st = line.index('=') + 1        
                #for c in line:
                #    if(c == '='): st = line.index('=') + 1
                #    else: break
                end = len(line)
                rpm = rpm + int(line[st:end])
                cnt = cnt + 1
                line = ' '
            else:
                line = line + rx
                
        if(rx == ''): break
        rpm = rpm/cnt
        rpm = int(rpm)
        print('rpm',rpm)
        fw.write((str(rpm) + '\n'))
        
        cmd_send(drive, 0x0)
        time.sleep(1)
        ser.flushInput()

#pwmTOrpm()
    
j = 100
cmd_send(SpeedLeft,  j)
cmd_send(SpeedRight, j)

cmd_send(Cnt_Left , 1 * 20) #set n rotation of whell 
cmd_send(Cnt_Right, 1 * 20) #set n rotation of whell 

cmd_send(drive, Left | Right | 0x3)

time.sleep(1)
ser.flushInput()

#cmd_send(drive, 0)
ser.close()
fw.close()