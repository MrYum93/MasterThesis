#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This spring used: https://www.thespringstore.com/pe028-375-94768-mw-3160-mh-n-in.html?unit_measure=me

Revision
YYYY-MM-DD
2018-11-08 MB First
2018-11-09 MW Added spring parameters
'''

import math
import numpy as np
import matplotlib.pyplot as plt
import plotly.plotly as py
import plotly.graph_objs as go
from plotly.tools import FigureFactory as FF
import pandas as pd
import scipy

#We're using SI units
class ar_model(object):
    def __init__(self):
        # lists
        self.plane_speed_list = []
        self.time_list = []
        self.arm_ang_list = []
        self.rope_len_list = []
        self.plane_acc_list = []

        self.time = 0
        self.plane_init_speed = 17.0
        self.plane_speed = 17.0
        self.plane_weight = 2.3
        self.spring_constant = 42
        self.displacement_plane = 0
        self.init_plane_e = 0
        self.inner_spring_arm = 0.105
        self.outer_spring_arm = 0.2165
        self.spring_start_stretch = self.outer_spring_arm - self.inner_spring_arm
        self.equilibrium = 0.080 # spring resting pos
        self.max_spring_stretch = 0.279400
        self.max_spring_load = 12.900
        self.half_distance_arms = 1
        self.gen_cnt = 0
        self.old_time = 0
        self.old_vel = 0

    def spring_forcediagram(self):
        start_f = 0
        end_f = self.max_spring_load
        curr_f = 0
        f_list = []
        curr_dist = 0
        dist_list = []
        while curr_f < end_f:
            curr_f = self.spring_constant * curr_dist
            curr_dist += 0.01
            f_list.append(curr_f)
            dist_list.append(curr_dist)

        fig = plt.figure()
        plt.plot(dist_list, f_list, 'r')
        fig.suptitle('Spring force / displacement', fontsize=20)
        plt.xlabel('Distance[m]', fontsize=18)
        plt.ylabel('Force', fontsize=16)
        plt.show()

    def force_to_acc(self, force, weight):
        return force/weight

    def spring_energy(self, displacement):
        return 0.5*self.spring_constant*(displacement)**2

    def plane_energy(self, plane_speed):
        return 0.5*self.plane_weight*(plane_speed)**2

    def angle_to_spring_stretch(self, angle):
        return math.sqrt((self.inner_spring_arm*math.sin(angle))**2+(-self.inner_spring_arm*math.cos(angle)+self.outer_spring_arm)**2)

    def displacement_to_angle(self, displacement):
        return math.atan(displacement/self.half_distance_arms)

    def pos_to_acc(self, pos):
        return -(self.spring_constant / self.plane_weight) * pos

    def plane_pos_to_rope_len(self, plane_pos):
        rope_len = math.sqrt(1**2 + plane_pos**2)
        return rope_len



    def arm_angle_to_vel_acc(self, time_list, ang_list):
        x = time_list
        y = ang_list
        dy = np.zeros(y.__len__(), np.float)
        dyy = np.zeros(y.__len__(), np.float)
        dy[0:-1] = np.diff(y) / np.diff(x)
        dyy[0:-1] = np.diff(dy) / np.diff(x)
        dy[-1] = (y[-1] - y[-2]) / (x[-1] - x[-2])
        dyy[-1] = (dy[-1] - dy[-2]) / (x[-1] - x[-2])

        # Remove the last two points bc of spike in acceleration
        x = x[:-2]
        y = y[:-2]
        dy = dy[:-2]
        dyy = dyy[:-2]

        plt.subplot(3, 1, 1)
        plt.plot(time_list, ang_list, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Arm ang pos', fontsize=16)

        plt.subplot(3, 1, 2)
        plt.plot(time_list, dy, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Arm ang vel', fontsize=16)

        plt.subplot(3, 1, 3)
        plt.plot(time_list, dyy, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Arm ang acc', fontsize=16)

        plt.show()

    def pos_to_vel_acc(self, time_list, pos_list, title):
        x = time_list
        y = pos_list
        dy = np.zeros(y.__len__(), np.float)
        dyy = np.zeros(y.__len__(), np.float)
        dy[0:-1] = np.diff(y) / np.diff(x)
        dyy[0:-1] = np.diff(dy) / np.diff(x)
        dy[-1] = (y[-1] - y[-2]) / (x[-1] - x[-2])
        dyy[-1] = (dy[-1] - dy[-2]) / (x[-1] - x[-2])

        # Remove the last two points bc of spike in acceleration
        x = x[:-2]
        y = y[:-2]
        dy = dy[:-2]
        dyy = dyy[:-2]

        plt.subplot(3, 1, 1)
        plt.title(title)
        plt.plot(x, y, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Pos', fontsize=16)

        plt.subplot(3, 1, 2)
        plt.plot(x, dy, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Vel', fontsize=16)

        plt.subplot(3, 1, 3)
        plt.plot(x, dyy, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Acc', fontsize=16)

        plt.show()

    def mass_spring_odeint(self, init_state, t):
        # unpack the state vector
        x = init_state[0]
        xd = init_state[1]

        # these are our constants
        k = -self.spring_constant  # Newtons per metre
        m = self.plane_weight  # Kilograms
        g = self.plane_init_speed #plane_acc  # metres per second

        # compute acceleration of plane xdd
        xdd = ((k * x) / m) + g

        # return the two state derivatives
        self.old_time = t
        return [xd, xdd]

    def main(self):
        plane_mass = 2
        spring_c = 42
        plane_vel = 17
        eq = 0.2
        pos = 0.3
        delta_t = 0.01
        time_l = []
        pos_l = []
        t = 0
        while t < 3:
            acc = (eq - pos) * (spring_c / plane_mass)
            plane_vel = plane_vel + acc * delta_t
            pos = pos + plane_vel * delta_t
            t += delta_t
            pos_l.append(pos)
            time_l.append(t)

        self.pos_to_vel_acc(time_l, pos_l, "spring")


if __name__ == "__main__":
    model = ar_model()
    model.main() # spring_forcediagram()