# -*- coding: utf-8 -*-
"""
Created on Sat Jan  9 19:23:00 2021

@author: Dmitry
"""

rpm_min = 0;
rpm_max = 190;



minU = 0xA0;
maxU = 0xFF;
goalSpeed = 150;

Kp = 0.5; 
Ki = 0.0;
Kd = 0.0;

integral = 0.0;
pred_err = 0.0;
min_integral = -100.0;
max_integral = 100.0;

def Eval(err):
    y = 0.0
    rdiff = 0.0
    global integral
    global pred_err
  
    integral = integral + err # добавить ошибку в сумму ошибок
    if(integral > max_integral):
        integral = max_integral
    if(integral < min_integral):
        integral = min_integral
      
    rdiff = Kd*(err - pred_err)
  
    # вычисление управляющего воздействия
    y = (Kp*err + Ki*integral + rdiff)
    pred_err = err #текущая ошибка стала "прошлой ошибкой"
  
    #для след. вычисления
    return y

error = 0#150 - 135
rpm = 150
error = goalSpeed - rpm;

y = Eval(error)
print('Eval = ', y)

maxY = Kp*rpm_max + Ki*max_integral + Kd*rpm_max;
minY = -maxY

u = (y-minY)/(maxY-minY)*(maxU-minU)+minU;
print("u = ", u)