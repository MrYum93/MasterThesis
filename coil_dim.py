#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import matplotlib.pyplot as plt
import sys
from plot_system import PlotSystem
import time

h = 0.004
rho = 600
r = 0.05
circle_circum = 0
r_list = []
motor_alpha_list = []
rope_speed_list = []
rope_in_one_coil_l = []
rope_len_one_sec_l = []
while r < 0.4:
    v=math.pi*r**2*h
    m=v*rho
    motor_torque = 7.6

    I=0.5*m*r**2


    motor_max_alpha = motor_torque/(I+0.000084)
    circle_circum = motor_max_alpha*r
    omega = motor_max_alpha*1
    theta = omega*1
    
    rope_len = theta*r
    rope_len_one_sec_l.append(rope_len)
    r_list.append(r)
    rope_in_one_coil_l.append(circle_circum)
    motor_alpha_list.append(motor_max_alpha)
    r += 0.01

fig = plt.figure
plt.plot(r_list, rope_in_one_coil_l, label="Motor force")
plt.plot(r_list, motor_alpha_list, "r--", label="Motor acceleration")
plt.plot(r_list, rope_len_one_sec_l, "b--", label="Rope length after 1 sec")
title = "Coil dim"
plt.ylabel('Different units')
plt.xlabel("Radius")
plt.legend()
plt.show()