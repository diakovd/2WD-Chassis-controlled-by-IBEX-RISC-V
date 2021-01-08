# -*- coding: utf-8 -*-
"""
Created on Sun Dec 27 17:39:42 2020

@author: Dmitry
"""

m = 0.03333
n = 400000
n2 = 45000000
t = m*n
t2 = m*n2
f = 60000000/(t*20)
print(m)
print(t)
print(t2)
print(f)


rpm_min = 0;
rpm_max = 190;



minU = 0xA0;
maxU = 0xFF;
goalSpeed = 10;

Kp = 0.5; 
Ki = 0.0;
Kd = 0.0;

integral = 0.0;
pred_err = 0.0;
min_integral = -100.0;
max_integral = 100.0;

maxY = Kp*rpm_max + Ki*max_integral + Kd*rpm_max;
minY = -maxY

u = (y-minY)/(maxY-minY)*(maxU-minU)+minU;
print("maxY", maxY)