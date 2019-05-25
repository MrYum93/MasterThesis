#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import matplotlib.pyplot as plt
import sys
from plot_system import PlotSystem
import time

h = 0.012 # one sheet is 4 mm -> 0.004 m. We need at least 4 sheets
rho = 600
r = 0.1 # best between 0.2 and 0.25
delta_r = 0.05 # with 0.0125 it should be run 4 times
start_r = r
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
motor_alpha_l = []
motor_omega_l = []
rope_len_l = []
rope_speed_l = []
force_l = []

container_l = []

cnt = 0

while r >= start_r and r < start_r + 4 * delta_r:
    print("cnt", cnt)
    cnt += 1
    omega = 0
    theta = 0
    t = 0

    r_l = []
    t_l = []
    motor_alpha_l = []
    motor_omega_l = []
    rope_len_l = []
    rope_speed_l = []
    force_l = []

    h = 1/r**2 / 300
    vol = math.pi*r**2*h
    m = vol*rho
    I=0.5*m*r**2
    motor_max_alpha = motor_torque/(I+0.000084) # max acceleration rad/s**2
    print("H, I, M", h, I, m)
    while t < 0.5:
        omega += motor_max_alpha*delta_t
        theta += omega*delta_t

        rope_speed = omega*2*r*math.pi
        rope_len = theta*r*2*math.pi

        force = motor_torque/r

        r_l.append(r)
        rope_len_l.append(rope_len)
        rope_speed_l.append(rope_speed)
        motor_alpha_l.append(motor_max_alpha)
        motor_omega_l.append(omega)
        force_l.append(force)
        t_l.append(t)
        if r > start_r and r < start_r + 5*delta_r:
            circumference = r*2*math.pi
            rope_total_len = 20
            rope_on_one_coil = (20 - 5) / 2

            print(rope_on_one_coil)
            print(circumference)
            print("rope on one coil: ", rope_on_one_coil/circumference)
            #exit()

        t += delta_t

    container_l.append([r_l, rope_len_l, rope_speed_l, motor_alpha_l, motor_omega_l, force_l, t_l])

    cnt += 1
    r += delta_r


# to get rad: r_l[int(len(r_l)*1/4)-1], r_l[int(len(r_l)*2/4)-1], r_l[int(len(r_l)*3/4)-1], r_l[int(len(r_l)*4/4)-1]
r0 = '{:0.3f}'.format(container_l[0][int(len(container_l)/4)-1][0])
r1 = '{:0.3f}'.format(container_l[1][int(len(container_l)/4)-1][0])
r2 = '{:0.3f}'.format(container_l[2][int(len(container_l)/4)-1][0])
r3 = '{:0.3f}'.format(container_l[3][int(len(container_l)/4)-1][0])

plt.subplot(2, 2, 1)
plt.plot(container_l[0][6], container_l[0][4], "r-", label=r0)
plt.plot(container_l[0][6], container_l[1][4], "g-", label=r1)
plt.plot(container_l[0][6], container_l[2][4], "b-", label=r2)
plt.plot(container_l[0][6], container_l[3][4], "c-", label=r3)
plt.ylabel('Rad/s')
plt.xlabel("Time")
plt.title("coil vel")
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(container_l[0][6], container_l[0][3], "r-", label=r0)
plt.plot(container_l[0][6], container_l[1][3], "g-", label=r1)
plt.plot(container_l[0][6], container_l[2][3], "b-", label=r2)
plt.plot(container_l[0][6], container_l[3][3], "c-", label=r3)
plt.ylabel('Rad/s^2')
plt.xlabel("Time")
plt.title("coil acc")
plt.legend()

plt.subplot(2, 2, 3)
plt.plot(container_l[0][6], container_l[0][1], "r-", label=r0)
plt.plot(container_l[0][6], container_l[1][1], "g-", label=r1)
plt.plot(container_l[0][6], container_l[2][1], "b-", label=r2)
plt.plot(container_l[0][6], container_l[3][1], "c-", label=r3)
plt.ylabel('m')
plt.xlabel("Time")
plt.title("Rope len")
plt.legend()

plt.subplot(2, 2, 4)
plt.plot(container_l[0][6], container_l[0][2], "r-", label=r0)
plt.plot(container_l[0][6], container_l[1][2], "g-", label=r1)
plt.plot(container_l[0][6], container_l[2][2], "b-", label=r2)
plt.plot(container_l[0][6], container_l[3][2], "c-", label=r3)
plt.ylabel('m/s')
plt.xlabel("Time")
plt.title("Rope speed")
plt.legend()

plt.show()
