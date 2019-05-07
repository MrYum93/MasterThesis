#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import matplotlib.pyplot as plt
import sys
from plot_system import PlotSystem
import time

h = 0.016 # one sheet is 4 mm -> 0.004 m. We need at least 4 sheets
#h = 2 # This is to test if the fore on the plane can be changed by making the coil taller
rho = 600
r = 0.15 # best between 0.2 and 0.25
delta_r = 0.05 # with 0.0125 it should be run 4 times
t = 0
delta_t = 0.01
circle_circum = 0
omega = 0
theta = 0
motor_torque = 1.5
force = 0
rope_speed = 0

r_l = []
t_l = []
motor_alpha_list = []
motor_omega_l = []
rope_speed_list = []
rope_in_one_coil_l = []
rope_len_l = []
rope_speed_l = []
force_l = []

cnt = 0
while r < 0.35:
    vol = math.pi*r**2*h
    m = vol*rho
    I=0.5*m*r**2
    motor_max_alpha = motor_torque/(I+0.000084) # max acceleration rad/s**2

    while t < 0.5:
        omega += motor_max_alpha*delta_t
        theta += omega*delta_t

        rope_speed = omega*2*r*math.pi
        rope_len = theta*r*2*math.pi

        force = motor_torque/r

        r_l.append(r)
        rope_len_l.append(rope_len)
        rope_speed_l.append(rope_speed)
        motor_alpha_list.append(motor_max_alpha)
        motor_omega_l.append(omega)
        force_l.append(force)
        t_l.append(t+cnt)

        t += delta_t

    omega = 0
    theta = 0
    t = 0
    cnt += 1
    r += delta_r

fig = plt.figure
plt.plot(t_l, r_l, "y", label="coil rad")
plt.plot(t_l, force_l, "k", label="force on plane")
plt.plot(t_l, motor_omega_l, "r--", label="Motor vel")
# plt.plot(t_l, motor_alpha_list, "g--", label="Motor acc")
plt.plot(t_l, rope_len_l, "b--", label="Rope length")
plt.plot(t_l, rope_speed_l, "c", label="rope speed")
title = "Coil dim"
plt.ylabel('Different units')
plt.xlabel("Time")
plt.legend()
plt.show()
