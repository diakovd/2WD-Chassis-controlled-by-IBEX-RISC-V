# -*- coding: utf-8 -*-
"""
Created on Tue Dec 29 14:43:29 2020

@author: dyakov
"""

import matplotlib.pyplot as plt
import numpy as np
import math

def h(x):
    if(x <= 0): y = 0
    else: y = math.exp(-(1/x))
    return y   

def l(x):
    if(x <= 0): y = 0
    else: y = math.exp(-(1/x))
    return y    
v = 2
Tfall = 4
tf = v

def m(x):
    t = 1 - x
    z = tf - t
    y = l(t)/(l(t)+l(z))
    return y 

def g(x):
    z = tf - x
    y = h(x)/(h(x)+h(z))
    return y 

#xpoints = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2]
xpoints = []
x = 0.0
for i in range(60):
    x = x + 0.1
    xpoints.append(x)
    
ypoints = []
print(xpoints)

for i in xpoints :
    if(i < Tfall)  : ypoints.append(v*g(i))
    else: ypoints.append(v*m(i - (Tfall + tf - 1)))
print(ypoints)

plt.plot(xpoints, ypoints)
plt.show()

